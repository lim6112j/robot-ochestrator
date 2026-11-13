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

use crate::orders::{
    InstantActionRequest, OrderAssembler, OrderPublisher, OrderRequest, INSTANT_ACTIONS,
};
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
    /// Shared secret guarding the HTTP surface, when configured.
    pub token: crate::auth::ApiToken,
}

/// Snapshot returned by `/api/robots`.
#[derive(Debug, Serialize)]
pub struct FleetSnapshot {
    pub robots: Vec<vda5050_types::CanonicalRobot>,
    pub online: usize,
    pub total: usize,
}

/// Build the router.
///
/// The dashboard itself is served without a token — it has to load before it
/// can ask for one. Everything else, including `/ws`, sits behind the gate.
pub fn router(state: AppState) -> Router {
    let protected = Router::new()
        .route("/api/robots", get(robots))
        .route("/api/robots/{robot_id}", get(robot))
        .route("/api/events", get(events))
        .route("/api/metrics", get(metrics))
        .route("/api/dlq", get(dlq))
        .route("/api/map", get(map))
        .route("/api/orders", post(publish_order))
        .route("/api/adapters", get(adapters))
        .route("/api/shared-resources", get(shared_resources))
        .route("/api/instant-actions", get(instant_action_catalog))
        .route(
            "/api/robots/{robot_id}/instant-actions",
            post(publish_instant_actions),
        )
        .route("/ws", get(ws_handler))
        .route_layer(axum::middleware::from_fn_with_state(
            state.token.clone(),
            crate::auth::require_token,
        ));

    Router::new()
        .route("/", get(index))
        .merge(protected)
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

/// Registered fleet adapters and what they can do.
async fn adapters(State(state): State<AppState>) -> Json<Vec<serde_json::Value>> {
    let mut out = Vec::new();
    for adapter in state.store.adapters().await {
        let robots = adapter.robots().await;
        out.push(serde_json::json!({
            "manufacturer": adapter.manufacturer(),
            "robots": robots.len(),
            "robotIds": robots.iter().map(|r| r.robot_id.clone()).collect::<Vec<_>>(),
            "sharedResources": adapter.shared_resources().await,
        }));
    }
    Json(out)
}

/// Shared resources (doors, lifts) robots may contend for.
///
/// The dashboard uses this to explain why a robot is waiting: a robot stopped
/// at a door and a robot that lost its connection look identical in a
/// position plot.
async fn shared_resources(State(state): State<AppState>) -> Json<Vec<serde_json::Value>> {
    let mut out = Vec::new();
    for adapter in state.store.adapters().await {
        for resource in adapter.shared_resources().await {
            let mut held_by = Vec::new();
            for robot in adapter.robots().await {
                if adapter.holds(&robot.robot_id, &resource.id).await {
                    held_by.push(robot.robot_id);
                }
            }
            out.push(serde_json::json!({
                "id": resource.id,
                "kind": format!("{:?}", resource.kind),
                "contenders": resource.contenders,
                "heldBy": held_by,
                "manufacturer": adapter.manufacturer(),
            }));
        }
    }
    Json(out)
}

/// The instant actions the control screen offers.
async fn instant_action_catalog() -> Json<&'static [&'static str]> {
    Json(INSTANT_ACTIONS)
}

/// `POST /api/robots/{robot_id}/instant-actions` — pause, resume or cancel.
///
/// This is the counterpart to a mission: without it an operator who sent a
/// wrong order has no way to stop the robot short of killing the simulator.
async fn publish_instant_actions(
    State(state): State<AppState>,
    Path(robot_id): Path<String>,
    request: Result<axum::Json<InstantActionRequest>, axum::extract::rejection::JsonRejection>,
) -> Result<Json<Value>, OrderRejection> {
    let axum::Json(request) = request?;
    let message =
        crate::orders::submit_instant(&state.publisher, &state.store, &robot_id, &request)
            .await
            .map_err(|reasons| OrderRejection {
                error: "invalid_instant_action",
                reasons,
            })?;

    Ok(Json(json!({
        "status": "published",
        "robotId": robot_id,
        "headerId": message.header_id,
        "actions": message.actions.iter().map(|a| a.action_type.clone()).collect::<Vec<_>>(),
    })))
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
