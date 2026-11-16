//! `lif2rmf` — turn a VDA 5050 map into an RMF building map.
//!
//! The warehouse map is the single source of truth; RMF needs its own format,
//! and hand-maintaining both is how they drift apart.
//!
//! Input is either this project's own map JSON (`--map`) or a VDMA LIF file
//! (`--lif`). LIF is the format a facility hands over, so importing it is a
//! conversion and not a rename — see [`lif2rmf::lif`] for what does and does
//! not carry over.

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

    /// VDMA LIF file to import instead of `--map`.
    ///
    /// LIF is what a facility hands over. It has no charger concept, so
    /// `--chargers` must be supplied alongside it.
    #[arg(long, conflicts_with = "map")]
    lif: Option<PathBuf>,

    /// Node ids that are charging waypoints.
    ///
    /// Required with `--lif`: LIF does not describe chargers, and RMF refuses
    /// to add a robot to a fleet whose nav graph has none.
    #[arg(long, value_delimiter = ',')]
    chargers: Vec<String>,

    /// Also write the imported map back out in this project's own JSON.
    ///
    /// Only meaningful with `--lif`: the gateway and the simulator read the
    /// native format, so importing a LIF file for them means converting it
    /// once here rather than teaching every consumer a second format.
    #[arg(long)]
    emit_map: Option<PathBuf>,

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

/// The map id a LIF document should be served under.
///
/// LIF names a project, a layout and a level; this project serves one map id
/// per file, and that id becomes the RMF building map's `name`. It therefore
/// has to match what a native map would use — the layout id, which is the
/// project's own name for the area. The *level* name is a separate thing and
/// comes from `--level`, so reaching for `layoutLevelId` here would produce a
/// building map named `L1` where the native one is named `plant-b/l1`.
fn map_id_of(doc: &lif2rmf::lif::Lif) -> String {
    let layout = &doc.layouts[0];
    if layout.layout_id.is_empty() {
        doc.meta_information.project_identification.clone()
    } else {
        layout.layout_id.clone()
    }
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

    let map: lif2rmf::SourceMap = if let Some(lif_path) = &args.lif {
        let raw = std::fs::read_to_string(lif_path)?;
        let doc: lif2rmf::lif::Lif = serde_json::from_str(&raw)?;
        let mut map = lif2rmf::lif::to_source_map(&doc, &map_id_of(&doc), args.default_speed)?;
        // LIF has no charger concept, so they cannot be read from the file.
        map.chargers.clone_from(&args.chargers);
        tracing::info!(
            chargers = map.chargers.len(),
            "imported LIF; chargers came from --chargers, not the file"
        );
        map
    } else {
        let raw = std::fs::read_to_string(&args.map)?;
        serde_json::from_str(&raw)?
    };
    let options = Options {
        level: args.level.clone(),
        door_prefix: args.door_prefix.clone(),
        door_width: args.door_width,
        default_speed: args.default_speed,
    };

    if let Some(path) = &args.emit_map {
        std::fs::write(path, serde_json::to_string_pretty(&map)?)?;
        tracing::info!(out = %path.display(), "native map written");
    }

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
