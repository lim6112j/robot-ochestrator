//! `gateway` — VDA 5050 → canonical model → REST/WebSocket + digital twin dashboard.
//!
//! The binary and the library share the same modules: `main` wires them together,
//! integration tests exercise them directly.

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;

use axum::Router;
use clap::Parser;
use serde_json::Value;
use tokio::net::TcpListener;
use tracing::{info, warn};
use vda5050_types::MAJOR_VERSION;

use gateway::api;
use gateway::auth::ApiToken;
use gateway::ingest::{self, IngestConfig};
use gateway::orders::OrderPublisher;
use gateway::store::Store;

/// Command line arguments.
#[derive(Debug, Parser)]
#[command(name = "gateway", version, about)]
struct Args {
    /// MQTT broker host.
    #[arg(long, env = "BROKER_HOST", default_value = "localhost")]
    broker_host: String,

    /// MQTT broker port.
    #[arg(long, env = "BROKER_PORT", default_value = "1883")]
    broker_port: u16,

    /// HTTP listen address.
    #[arg(long, env = "HTTP_ADDR", default_value = "0.0.0.0:8080")]
    http_addr: SocketAddr,

    /// VDA 5050 interface name to subscribe to.
    #[arg(long, env = "INTERFACE", default_value = "vda5050")]
    interface: String,

    /// VDA 5050 major version to subscribe to.
    #[arg(long, env = "MAJOR_VERSION", default_value_t = MAJOR_VERSION)]
    major_version: u8,

    /// Also ingest v2 fleets published on the legacy `uomg` interface.
    #[arg(long, env = "ACCEPT_V2", default_value = "true")]
    accept_v2: bool,

    /// Robots not seen for this long are reported offline.
    #[arg(long, env = "HEARTBEAT_TIMEOUT_S", default_value = "10")]
    heartbeat_timeout_s: f64,

    /// Building map served to the dashboard.
    #[arg(long, default_value = "deploy/maps/plant-b.l1.json")]
    map: PathBuf,

    /// Shared secret required on every `/api/*` route and the WebSocket.
    ///
    /// Unset leaves the gateway open, which is only acceptable against a
    /// loopback dev broker: dispatching a mission drives physical hardware.
    #[arg(long, env = "API_TOKEN")]
    api_token: Option<String>,

    #[arg(long, default_value = "info")]
    log: String,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new(args.log.clone())),
        )
        .with_target(false)
        .compact()
        .init();

    let map_json: Value = std::fs::read_to_string(&args.map)
        .ok()
        .and_then(|raw| serde_json::from_str(&raw).ok())
        .unwrap_or_else(|| serde_json::json!({ "mapId": "unknown", "nodes": {}, "edges": [] }));

    let store = Store::new(args.heartbeat_timeout_s);

    let token = ApiToken::new(args.api_token.clone());
    if token.is_enabled() {
        info!("api token required on /api/* and /ws");
    } else {
        warn!(
            "api token not set — anyone who can reach port {} can dispatch missions.              set --api-token (or API_TOKEN) to require authentication",
            args.http_addr.port()
        );
    }

    let ingest_config = IngestConfig {
        interface: args.interface.clone(),
        major_version: args.major_version,
        accept_v2: args.accept_v2,
        heartbeat_timeout_s: args.heartbeat_timeout_s,
    };

    // One broker connection serves both directions: the ingest loop drives its
    // event loop, which is what flushes orders published from the dashboard.
    let (mqtt_client, mqtt_eventloop) =
        ingest::connect(&args.broker_host, args.broker_port, "gateway");
    let mqtt_client = Arc::new(mqtt_client);

    let state = api::AppState {
        store: Arc::clone(&store),
        map_json: Arc::new(map_json),
        publisher: Arc::new(OrderPublisher::new(
            Arc::clone(&mqtt_client),
            Arc::new(vda5050_types::MessageValidator::new()?),
            args.interface.clone(),
            args.major_version,
        )),
        validator: Arc::new(vda5050_types::MessageValidator::new()?),
        token: token.clone(),
    };

    let app: Router = api::router(state.clone());

    info!(
        listen = %args.http_addr,
        interface = %args.interface,
        major_version = args.major_version,
        authenticated = token.is_enabled(),
        "gateway starting"
    );

    let listener = TcpListener::bind(args.http_addr).await?;
    let http = tokio::spawn(async move {
        axum::serve(listener, app).await.expect("http server");
    });

    let mqtt = tokio::spawn(ingest::run(
        (*mqtt_client).clone(),
        mqtt_eventloop,
        ingest_config,
        store,
    ));

    tokio::select! {
        _ = http => info!("http server stopped"),
        result = mqtt => {
            if let Err(e) = result {
                tracing::error!(error = %e, "mqtt ingest failed");
            }
        }
        _ = tokio::signal::ctrl_c() => info!("shutdown requested"),
    }

    Ok(())
}
