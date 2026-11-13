//! HTTP API and WebSocket stream for the digital twin dashboard.

use serde_json::Value;
use std::sync::Arc;

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::{Path, State};
use axum::http::StatusCode;
use axum::response::{Html, IntoResponse};
use axum::routing::{get, post};
use axum::{Json, Router};
use futures_util::{SinkExt, StreamExt};
use serde::Serialize;
use serde_json::json;

use crate::orders::{OrderAssembler, OrderPublisher, OrderRequest};
use crate::store::Store;
use vda5050_types::MessageValidator;

/// Combined application state.
#[derive(Clone, Debug)]
pub struct AppState {
    pub store: Arc<Store>,
    pub map_json: Arc<Value>,
    /// Publishes missions on the robots' own `order` topics.
    pub publisher: Arc<OrderPublisher>,
    /// Compiles the vendored schemas; shared with the ingest loop.
    pub validator: Arc<MessageValidator>,
}

/// Snapshot returned by `/api/robots`.
#[derive(Debug, Serialize)]
pub struct FleetSnapshot {
    pub robots: Vec<vda5050_types::CanonicalRobot>,
    pub online: usize,
    pub total: usize,
}

/// Build the router.
pub fn router(state: AppState) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/api/robots", get(robots))
        .route("/api/robots/{robot_id}", get(robot))
        .route("/api/events", get(events))
        .route("/api/metrics", get(metrics))
        .route("/api/dlq", get(dlq))
        .route("/api/map", get(map))
        .route("/api/orders", post(publish_order))
        .route("/ws", get(ws_handler))
        .with_state(state)
}

/// Serve the digital twin dashboard.
async fn index() -> Html<&'static str> {
    Html(include_str!("../static/index.html"))
}

/// All robots.
async fn robots(State(state): State<AppState>) -> Json<FleetSnapshot> {
    let robots = state.store.robots().await;
    let online = robots
        .iter()
        .filter(|r| r.connection == vda5050_types::ConnectionState::Online)
        .count();
    let total = robots.len();
    Json(FleetSnapshot {
        robots,
        online,
        total,
    })
}

/// One robot.
async fn robot(
    State(state): State<AppState>,
    Path(robot_id): Path<String>,
) -> Result<Json<vda5050_types::CanonicalRobot>, axum::http::StatusCode> {
    state
        .store
        .robot(&robot_id)
        .await
        .map(Json)
        .ok_or(axum::http::StatusCode::NOT_FOUND)
}

/// Recent events.
async fn events(State(state): State<AppState>) -> Json<Vec<serde_json::Value>> {
    let events: Vec<serde_json::Value> = state
        .store
        .events(100)
        .await
        .iter()
        .filter_map(|e| serde_json::to_value(e).ok())
        .collect();
    Json(events)
}

/// Metrics counters.
async fn metrics(State(state): State<AppState>) -> Json<serde_json::Value> {
    let metrics = state.store.metrics().await;
    Json(serde_json::to_value(metrics).unwrap_or(serde_json::Value::Null))
}

/// Recent DLQ entries.
async fn dlq(State(state): State<AppState>) -> Json<Vec<serde_json::Value>> {
    let entries: Vec<serde_json::Value> = state
        .store
        .dlq(50)
        .await
        .iter()
        .map(|e| {
            serde_json::json!({
                "topic": e.topic,
                "topicKind": e.topic_kind.to_string(),
                "reasons": e.reasons,
            })
        })
        .collect();
    Json(entries)
}

/// The building map used by the dashboard.
async fn map(State(state): State<AppState>) -> Json<Value> {
    Json((*state.map_json).clone())
}

/// WebSocket endpoint streaming gateway events.
async fn ws_handler(ws: WebSocketUpgrade, State(state): State<AppState>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_socket(socket, state))
}

/// A rejection the dashboard can show verbatim.
#[derive(Debug, Serialize)]
pub struct OrderRejection {
    pub error: &'static str,
    pub reasons: Vec<String>,
}

impl IntoResponse for OrderRejection {
    fn into_response(self) -> axum::response::Response {
        (StatusCode::BAD_REQUEST, Json(self)).into_response()
    }
}

/// Turn a JSON body that did not even parse into the same shape as an order
/// rejection, so the dashboard can show it without special-casing.
impl From<axum::extract::rejection::JsonRejection> for OrderRejection {
    fn from(rejection: axum::extract::rejection::JsonRejection) -> Self {
        Self {
            error: "malformed_request",
            reasons: vec![rejection.to_string()],
        }
    }
}
/// `POST /api/orders` — issue a mission from the dashboard.
///
/// The request carries the route only: `headerId`, `sequenceId` parity and map
/// geometry are derived, the result is schema-validated, and only then is it
/// published on `vda5050/v3/<manufacturer>/<serialNumber>/order`.
async fn publish_order(
    State(state): State<AppState>,
    request: Result<axum::Json<OrderRequest>, axum::extract::rejection::JsonRejection>,
) -> Result<Json<Value>, OrderRejection> {
    let axum::Json(request) = request?;
    let assembler = OrderAssembler::new(&state.map_json);
    let order = crate::orders::submit(&assembler, &state.publisher, &state.store, &request)
        .await
        .map_err(|reasons| OrderRejection {
            error: "invalid_order",
            reasons,
        })?;
    state.store.count_published().await;

    Ok(Json(json!({
        "status": "published",
        "robotId": format!("{}:{}", order.manufacturer, order.serial_number),
        "orderId": order.order_id,
        "headerId": order.header_id,
        "waypoints": order.nodes.len(),
    })))
}
async fn handle_socket(socket: WebSocket, state: AppState) {
    let (mut sender, mut receiver) = socket.split();
    let mut rx = state.store.subscribe();

    // Initial snapshot: robots plus the recent event log so a freshly opened
    // dashboard is not empty.
    let snapshot = state.store.robots().await;
    let events: Vec<serde_json::Value> = state
        .store
        .events(30)
        .await
        .iter()
        .filter_map(|e| serde_json::to_value(e).ok())
        .collect();
    if let Ok(json) = serde_json::to_string(&serde_json::json!({
        "type": "snapshot",
        "robots": snapshot,
        "events": events,
    })) {
        let _ = sender.send(Message::Text(json.into())).await;
    }

    let send_task = tokio::spawn(async move {
        while let Ok(event) = rx.recv().await {
            if sender.send(Message::Text(event.into())).await.is_err() {
                break;
            }
        }
    });

    while let Some(Ok(message)) = receiver.next().await {
        if matches!(message, Message::Close(_)) {
            break;
        }
    }
    send_task.abort();
}
