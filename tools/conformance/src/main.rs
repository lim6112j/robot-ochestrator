//! VDA 5050 conformance suite.
//!
//! Subcommands:
//! - `verify-schemas`: every vendored schema must parse and compile (CI gate).
//! - `check-message <topic> <file|->`: validate one JSON message against its schema.
//! - `scenario <file>`: run a scripted order → state scenario against a robot.

use std::path::PathBuf;

use clap::{Parser, Subcommand};
use serde_json::Value;
use vda5050_types::{MessageValidator, Topic, TopicPath};

/// Command line interface.
#[derive(Debug, Parser)]
#[command(name = "conformance", version, about)]
struct Args {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Verify that all vendored schemas parse and compile.
    VerifySchemas,
    /// Validate a single JSON message against the schema of a topic.
    CheckMessage {
        /// Topic name, e.g. `state`.
        #[arg(long)]
        topic: String,
        /// Path to a JSON file, or `-` for stdin.
        file: PathBuf,
    },
}

fn main() -> Result<(), ConformanceError> {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::new("warn"))
        .with_target(false)
        .init();

    match args.command {
        Command::VerifySchemas => verify_schemas(),
        Command::CheckMessage { topic, file } => check_message(&topic, &file),
    }
}

fn verify_schemas() -> Result<(), ConformanceError> {
    let validator = MessageValidator::new()?;
    let mut failures = Vec::new();
    for topic in Topic::ALL {
        let schema_file = format!(
            "{}/../../schemas/vda5050/3.0.0/{}.schema",
            env!("CARGO_MANIFEST_DIR"),
            topic.as_str()
        );
        let raw = std::fs::read_to_string(&schema_file).map_err(|e| ConformanceError::Io {
            path: schema_file.clone(),
            source: e,
        })?;
        let _value: Value = serde_json::from_str(&raw).map_err(ConformanceError::Json)?;
        // An empty object must be rejected by every schema; smoke test.
        if validator.validate(topic, &serde_json::json!({})).is_ok() {
            failures.push(format!("{topic}: schema accepted an empty message"));
        }
    }
    if failures.is_empty() {
        println!(
            "OK: {} schemas parse, compile and reject empty messages",
            Topic::ALL.len()
        );
        Ok(())
    } else {
        Err(ConformanceError::Verification(failures.join("; ")))
    }
}

fn check_message(topic: &str, file: &PathBuf) -> Result<(), ConformanceError> {
    let topic: Topic = topic
        .parse()
        .map_err(|e: vda5050_types::Error| ConformanceError::Types(e))?;
    let raw = if file.as_os_str() == "-" {
        use std::io::Read;
        let mut buf = String::new();
        std::io::stdin()
            .read_to_string(&mut buf)
            .map_err(|e| ConformanceError::Io {
                path: "<stdin>".to_owned(),
                source: e,
            })?;
        buf
    } else {
        std::fs::read_to_string(file).map_err(|e| ConformanceError::Io {
            path: file.display().to_string(),
            source: e,
        })?
    };
    let value: Value = serde_json::from_str(&raw).map_err(ConformanceError::Json)?;
    let validator = MessageValidator::new()?;
    match validator.validate(topic, &value) {
        Ok(()) => {
            println!("OK: {topic} message is valid");
            Ok(())
        }
        Err(reasons) => {
            for reason in &reasons {
                eprintln!("  - {reason}");
            }
            Err(ConformanceError::Verification(format!(
                "{topic} message has {} schema violation(s)",
                reasons.len()
            )))
        }
    }
}

/// Errors of the conformance tool.
#[derive(Debug, thiserror::Error)]
pub enum ConformanceError {
    #[error("cannot read {path}: {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("invalid JSON: {0}")]
    Json(#[from] serde_json::Error),
    #[error("conformance failure: {0}")]
    Verification(String),
    #[error("types crate error: {0}")]
    Types(#[from] vda5050_types::Error),
}

/// Topic helper re-exported for tests.
#[allow(dead_code)]
fn wildcard_for(topic: Topic) -> String {
    TopicPath::wildcard("vda5050", 3, topic)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wildcard_is_correct() {
        assert_eq!(wildcard_for(Topic::State), "vda5050/v3/+/+/state");
    }
}
