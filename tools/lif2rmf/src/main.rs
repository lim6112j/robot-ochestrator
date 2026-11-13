//! `lif2rmf` — turn a VDA 5050 map into an RMF building map.
//!
//! The warehouse map is the single source of truth; RMF needs its own format,
//! and hand-maintaining both is how they drift apart.

use std::path::PathBuf;

use clap::Parser;
use lif2rmf::{convert, to_yaml, Options};

/// Command line arguments.
#[derive(Debug, Parser)]
#[command(name = "lif2rmf", version, about)]
struct Args {
    /// VDA 5050 map (node/edge JSON).
    #[arg(long, default_value = "deploy/maps/plant-b.l1.json")]
    map: PathBuf,

    /// Output `.building.yaml` for RMF. Defaults to the map name.
    #[arg(long, short)]
    out: Option<PathBuf>,

    /// RMF level name.
    #[arg(long, default_value = "L1")]
    level: String,

    /// Node ids with this prefix become RMF doors (shared resources).
    #[arg(long, default_value = "DOOR")]
    door_prefix: String,

    /// Half-width of the door segment, in metres.
    #[arg(long, default_value = "0.5")]
    door_width: f64,

    /// Lane speed for edges that carry none.
    #[arg(long, default_value = "1.0")]
    default_speed: f64,

    /// Print the result instead of writing it.
    #[arg(long, short)]
    dry_run: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_target(false)
        .compact()
        .init();

    let raw = std::fs::read_to_string(&args.map)?;
    let map: lif2rmf::SourceMap = serde_json::from_str(&raw)?;

    let options = Options {
        level: args.level.clone(),
        door_prefix: args.door_prefix.clone(),
        door_width: args.door_width,
        default_speed: args.default_speed,
    };

    let building = convert(&map, &options)?;
    let yaml = to_yaml(&building);
    let level = &building.levels[0];

    if args.dry_run {
        print!("{yaml}");
        return Ok(());
    }

    let out = args.out.unwrap_or_else(|| {
        let stem = args
            .map
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("map");
        PathBuf::from(format!("{stem}.building.yaml"))
    });
    std::fs::write(&out, &yaml)?;

    tracing::info!(
        map = %building.name,
        out = %out.display(),
        vertices = level.vertices.len(),
        lanes = level.lanes.len(),
        doors = level.doors.len(),
        "RMF building map written"
    );
    Ok(())
}
