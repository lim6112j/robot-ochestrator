//! Local MQTT 5 broker for development and tests.
//!
//! Docker Hub is not reachable from every environment, so the repository ships
//! a Rust broker (`rumqttd`) instead of relying on a container image.
//!
//!   cargo run -p broker -- --port 1883
//!
//! Production deployments should use EMQX/HiveMQ with TLS and per-robot
//! credentials (see `deploy/broker/mosquitto.conf` for the container variant).

use clap::Parser;
use rumqttd::{Broker, Config};

/// Command line arguments.
#[derive(Debug, Parser)]
#[command(name = "broker", version, about)]
struct Args {
    /// TCP port to listen on.
    #[arg(long, default_value = "1883")]
    port: u16,

    /// Bind address.
    #[arg(long, default_value = "0.0.0.0")]
    bind: String,

    /// Maximum in-flight messages per connection.
    #[arg(long, default_value = "100")]
    inflight: u16,

    /// Maximum queued messages per connection.
    #[arg(long, default_value = "500")]
    max_queued: usize,

    #[arg(long, default_value = "info")]
    log: String,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new(args.log.clone())),
        )
        .with_target(false)
        .compact()
        .init();

    let mut settings = rumqttd::ServerSettings {
        name: format!("{}:{}", args.bind, args.port),
        listen: format!("{}:{}", args.bind, args.port)
            .parse()
            .expect("valid socket address"),
        tls: None,
        next_connection_delay_ms: 1,
        connections: rumqttd::ConnectionSettings {
            connection_timeout_ms: 60_000,
            max_payload_size: 1_048_576,
            max_inflight_count: usize::from(args.inflight),
            auth: None,
            external_auth: None,
            dynamic_filters: true,
        },
    };
    // No authentication: development only.
    let _ = &mut settings;

    let mut servers = std::collections::HashMap::new();
    servers.insert(format!("{}:{}", args.bind, args.port), settings);

    let config = Config {
        id: 0,
        router: rumqttd::RouterConfig::default(),
        v4: None,
        v5: Some(servers),
        ws: None,
        cluster: None,
        console: None,
        bridge: None,
        prometheus: None,
        metrics: None,
    };

    tracing::info!(
        listen = %format!("{}:{}", args.bind, args.port),
        inflight = args.inflight,
        "local MQTT 5 broker starting"
    );

    let mut broker = Broker::new(config);
    broker.start()?;
    Ok(())
}
