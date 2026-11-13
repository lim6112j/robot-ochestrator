//! `vda5050-sim` — a VDA 5050 v3.0.0 compliant virtual robot fleet.
//!
//! Every robot is an independent MQTT 5 client with its own last will, so the
//! gateway and the UI cannot tell simulated robots from real ones.

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use clap::Parser;
use serde_json::{json, Value};
use tokio::task::JoinSet;
use tracing::{error, info, warn};
use vda5050_types::types::{Connection, ConnectionState, InstantActions, Order};
use vda5050_types::{MessageValidator, Topic, TopicPath};

mod config;
mod mqtt;
mod robot;
mod world;

use crate::config::{Profile, RobotSpec, SimConfig};
use crate::mqtt::{build_options, connection_will, incoming_publish, publish_json, RobotLink};
use crate::robot::RobotVm;
use crate::world::MapGraph;

/// Command line arguments.
#[derive(Debug, Parser)]
#[command(name = "vda5050-sim", version, about)]
struct Args {
    /// Path to the simulator configuration (YAML).
    #[arg(long, short, default_value = "deploy/sim.yaml")]
    config: PathBuf,

    /// Override the number of robots per fleet.
    #[arg(long)]
    robots: Option<usize>,

    /// Override the state publication rate in Hz.
    #[arg(long)]
    publish_hz: Option<f64>,

    /// Deterministic seed.
    #[arg(long)]
    seed: Option<u64>,

    /// Stop after N seconds (0 = run forever).
    #[arg(long, default_value = "0")]
    duration: f64,

    /// Log level (error, warn, info, debug, trace).
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

    let mut config = SimConfig::load(&args.config)?;
    if let Some(count) = args.robots {
        for fleet in config.fleet.values_mut() {
            fleet.count = count;
        }
    }
    if let Some(hz) = args.publish_hz {
        for fleet in config.fleet.values_mut() {
            fleet.publish_hz = hz;
        }
    }
    let seed = args.seed.unwrap_or(config.seed);

    let map = match MapGraph::load(PathBuf::from(&config.world.map).as_path()) {
        Ok(map) => Arc::new(map),
        Err(e) => {
            error!(error = %e, "cannot load map, robots will stay at the origin");
            Arc::new(fallback_map())
        }
    };

    let specs = config.robots();
    if specs.is_empty() {
        warn!("no fleets configured, nothing to simulate");
        return Ok(());
    }

    info!(
        robots = specs.len(),
        map = %map.map_id(),
        broker = %format!("{}:{}", config.broker.host, config.broker.port),
        seed,
        "starting simulated fleet"
    );

    let mut tasks: JoinSet<()> = JoinSet::new();
    for (index, spec) in specs.into_iter().enumerate() {
        let broker = config.broker.clone();
        let map = Arc::clone(&map);
        let tick_hz = config.world.tick_hz;
        let start_node = start_node_for(&map, index, &config.world.start_nodes);
        tasks.spawn(run_robot(
            spec,
            broker,
            map,
            tick_hz,
            seed + index as u64,
            start_node,
        ));
    }

    if args.duration > 0.0 {
        tokio::time::sleep(Duration::from_secs_f64(args.duration)).await;
        info!("duration reached, shutting down");
        tasks.abort_all();
    } else {
        while tasks.join_next().await.is_some() {}
    }
    Ok(())
}

fn fallback_map() -> MapGraph {
    MapGraph {
        map_id: "unknown".to_owned(),
        nodes: std::collections::HashMap::new(),
        edges: Vec::new(),
    }
}

/// Interface name and major version a profile speaks.
fn interface_for(profile: Profile) -> (&'static str, u8) {
    match profile {
        Profile::V3Full | Profile::V3Partial => ("vda5050", 3),
        Profile::V2Only => ("uomg", 2),
    }
}

/// Run one virtual robot until the task is aborted.
/// Spread robots over the map so they do not all start on the same node.
/// Start node for the `index`-th robot: `startNodes[i]` when configured,
/// otherwise the alphabetically first node (deterministic).
fn start_node_for(map: &MapGraph, index: usize, start_nodes: &[String]) -> String {
    if let Some(node) = start_nodes.get(index) {
        if map.nodes.contains_key(node) {
            return node.clone();
        }
        warn!(index, node, "startNodes entry not on the map, falling back");
    }
    let mut ids: Vec<&String> = map.nodes.keys().collect();
    ids.sort();
    ids[index % ids.len().max(1)].clone()
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
async fn run_robot(
    spec: RobotSpec,
    broker: crate::config::BrokerConfig,
    map: Arc<MapGraph>,
    tick_hz: f64,
    seed: u64,
    start_node: String,
) {
    let (interface, major) = interface_for(spec.profile);
    // MQTT client ids must be globally unique: two fleets may use the same
    // serial number, so include the manufacturer (and fleet) as well.
    let client_id = format!(
        "{}-{}-{}",
        broker.client_prefix, spec.manufacturer, spec.serial_number
    );

    let mut vm = RobotVm::new(spec.clone(), (*map).clone(), &start_node, seed);

    let connection_path = TopicPath::new(
        interface,
        major,
        spec.manufacturer.clone(),
        spec.serial_number.clone(),
        Topic::Connection,
    );
    let order_path = TopicPath::new(
        interface,
        major,
        spec.manufacturer.clone(),
        spec.serial_number.clone(),
        Topic::Order,
    );
    let instant_path = TopicPath::new(
        interface,
        major,
        spec.manufacturer.clone(),
        spec.serial_number.clone(),
        Topic::InstantActions,
    );

    let last_will_payload: Value = json!({
        // `headerId` is an integer per the v3.0.0 schema.
        "headerId": 0,
        "timestamp": "2026-01-01T00:00:00.000Z",
        "version": if major == 3 { "3.0.0" } else { "2.1.0" },
        "manufacturer": spec.manufacturer,
        "serialNumber": spec.serial_number,
        "connectionState": "CONNECTION_BROKEN"
    });
    let will = connection_will(&connection_path.to_string(), &last_will_payload);

    let link = RobotLink::new(build_options(&client_id, &broker), will, 32);
    let RobotLink {
        client,
        mut eventloop,
    } = link;

    // Subscribe to commands addressed to this robot.
    if let Err(e) = client
        .subscribe(order_path.to_string(), rumqttc::QoS::AtMostOnce)
        .await
    {
        error!(robot = %spec.serial_number, error = %e, "cannot subscribe to order");
        return;
    }
    if let Err(e) = client
        .subscribe(instant_path.to_string(), rumqttc::QoS::AtMostOnce)
        .await
    {
        error!(robot = %spec.serial_number, error = %e, "cannot subscribe to instantActions");
        return;
    }

    // Announce ourselves.
    if let Err(e) = publish_json(
        &client,
        &factsheet_path(&spec, interface, major),
        Topic::Factsheet,
        true,
        &vm.factsheet_message(),
    )
    .await
    {
        error!(robot = %spec.serial_number, error = %e, "cannot publish factsheet");
        return;
    }
    publish_connection(&client, &spec, interface, major, &mut vm).await;

    info!(
        robot = %spec.serial_number,
        fleet = %spec.fleet,
        profile = ?spec.profile,
        "robot online"
    );

    let tick = Duration::from_secs_f64((1.0 / tick_hz.max(1.0)).max(0.001));
    let mut interval = tokio::time::interval(tick);
    let dt = tick.as_secs_f64();

    loop {
        tokio::select! {
            event = eventloop.poll() => {
                match event {
                    Ok(received) => {
                        if let Some((topic, payload)) = incoming_publish(&received) {
                            handle_command(&client, &spec, interface, major, &mut vm, &topic, &payload).await;
                        }
                    }
                    Err(e) => {
                        warn!(robot = %spec.serial_number, error = %e, "connection error, retrying");
                        tokio::time::sleep(Duration::from_millis(500)).await;
                    }
                }
            }
            _ = interval.tick() => {
                if let Some(limit) = spec.faults.drop_connection_after_s {
                    if vm.elapsed_seconds() >= limit {
                        info!(robot = %spec.serial_number, "fault injection: dropping connection");
                        publish_connection_state(&client, &spec, interface, major, &mut vm, ConnectionState::OFFLINE).await;
                        return;
                    }
                }

                vm.tick(dt);
                if vm.should_publish() && !vm.is_silent() {
                    publish_state(&client, &spec, interface, major, &mut vm).await;
                    vm.mark_published();
                }
            }
        }
    }
}

fn factsheet_path(spec: &RobotSpec, interface: &str, major: u8) -> String {
    TopicPath::new(
        interface,
        major,
        spec.manufacturer.clone(),
        spec.serial_number.clone(),
        Topic::Factsheet,
    )
    .to_string()
}

async fn publish_connection(
    client: &rumqttc::AsyncClient,
    spec: &RobotSpec,
    interface: &str,
    major: u8,
    vm: &mut RobotVm,
) {
    publish_connection_state(client, spec, interface, major, vm, ConnectionState::ONLINE).await;
}

#[allow(clippy::too_many_arguments)]
async fn publish_connection_state(
    client: &rumqttc::AsyncClient,
    spec: &RobotSpec,
    interface: &str,
    major: u8,
    vm: &mut RobotVm,
    state: ConnectionState,
) {
    let path = TopicPath::new(
        interface,
        major,
        spec.manufacturer.clone(),
        spec.serial_number.clone(),
        Topic::Connection,
    );
    let message: Connection = vm.connection_message(state);
    if let Err(e) = publish_json(
        client,
        &path.to_string(),
        Topic::Connection,
        false,
        &serde_json::to_value(&message).unwrap_or(Value::Null),
    )
    .await
    {
        warn!(robot = %spec.serial_number, error = %e, "cannot publish connection");
    }
}

#[allow(clippy::too_many_arguments)]
async fn publish_state(
    client: &rumqttc::AsyncClient,
    spec: &RobotSpec,
    interface: &str,
    major: u8,
    vm: &mut RobotVm,
) {
    if spec.profile == Profile::V2Only {
        // v2 robots publish the v2 shape on the v2 topic.
        let state_path = TopicPath::new(
            interface,
            major,
            spec.manufacturer.clone(),
            spec.serial_number.clone(),
            Topic::State,
        );
        let value = vm.state_as_v2_value();
        if let Err(e) =
            publish_json(client, &state_path.to_string(), Topic::State, false, &value).await
        {
            warn!(robot = %spec.serial_number, error = %e, "cannot publish v2 state");
        }
        return;
    }

    let state = vm.state_message();
    let state_path = TopicPath::new(
        interface,
        major,
        spec.manufacturer.clone(),
        spec.serial_number.clone(),
        Topic::State,
    );
    if let Err(e) = publish_json(
        client,
        &state_path.to_string(),
        Topic::State,
        false,
        &serde_json::to_value(&state).unwrap_or(Value::Null),
    )
    .await
    {
        warn!(robot = %spec.serial_number, error = %e, "cannot publish state");
        return;
    }

    // High frequency position + planned path for visualization systems.
    let vis_path = TopicPath::new(
        interface,
        major,
        spec.manufacturer.clone(),
        spec.serial_number.clone(),
        Topic::Visualization,
    );
    let vis = vm.visualization_message(state.header_id);
    if let Err(e) = publish_json(
        client,
        &vis_path.to_string(),
        Topic::Visualization,
        false,
        &serde_json::to_value(&vis).unwrap_or(Value::Null),
    )
    .await
    {
        warn!(robot = %spec.serial_number, error = %e, "cannot publish visualization");
    }
}

#[allow(clippy::too_many_arguments)]
async fn handle_command(
    client: &rumqttc::AsyncClient,
    spec: &RobotSpec,
    interface: &str,
    major: u8,
    vm: &mut RobotVm,
    topic: &str,
    payload: &[u8],
) {
    let Ok(path) = TopicPath::parse(topic) else {
        warn!(topic, "ignoring message on unexpected topic");
        return;
    };

    // Validate against the official schema before acting on it.
    let validator = MessageValidator::default();
    match path.topic {
        Topic::Order => match validator.validate_bytes(Topic::Order, payload) {
            Ok(value) => match serde_json::from_value::<Order>(value) {
                Ok(order) => {
                    let accepted = vm.accept_order(order);
                    info!(
                        robot = %spec.serial_number,
                        order = %vm.current_order_id(),
                        accepted,
                        "order received"
                    );
                    if !accepted {
                        publish_state(client, spec, interface, major, vm).await;
                    }
                }
                Err(e) => warn!(robot = %spec.serial_number, error = %e, "order payload mismatch"),
            },
            Err(rejected) => warn!(
                robot = %spec.serial_number,
                reasons = ?rejected.reasons,
                "order violates schema, ignored"
            ),
        },
        Topic::InstantActions => match validator.validate_bytes(Topic::InstantActions, payload) {
            Ok(value) => match serde_json::from_value::<InstantActions>(value) {
                Ok(actions) => {
                    let applied = vm.apply_instant_action(&actions);
                    info!(
                        robot = %spec.serial_number,
                        actions = applied.len(),
                        "instant actions applied"
                    );
                    publish_state(client, spec, interface, major, vm).await;
                }
                Err(e) => warn!(robot = %spec.serial_number, error = %e, "instantActions mismatch"),
            },
            Err(rejected) => warn!(
                robot = %spec.serial_number,
                reasons = ?rejected.reasons,
                "instantActions violate schema, ignored"
            ),
        },
        other => info!(topic = %other, "ignoring topic"),
    }
}
