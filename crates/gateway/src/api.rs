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
use utoipa::OpenApi;
use utoipa_swagger_ui::SwaggerUi;

use crate::orders::{
    InstantActionRequest, OrderAssembler, OrderPublisher, OrderRequest, INSTANT_ACTIONS,
};
use crate::store::{GatewayEvent, Metrics, Store};
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
#[derive(Debug, Serialize, utoipa::ToSchema)]
pub struct FleetSnapshot {
    pub robots: Vec<vda5050_types::CanonicalRobot>,
    pub online: usize,
    pub total: usize,
}

/// The API contract, generated from the handlers above.
///
/// Because it is derived from the code, it cannot drift: adding a field to a
/// response changes the spec on the next build. That is the whole point — a
/// hand-written README goes stale the moment a handler changes.
#[derive(OpenApi, Debug)]
#[openapi(
    info(
        title = "Robot control system gateway",
        version = "0.1.0",
        description = "VDA 5050 v3.0.0 fleet gateway: robot state, missions, \
                       instant actions and the digital-twin stream.",
    ),
    // Every `/api/*` route and `/ws` sits behind a bearer token; only the
    // dashboard at `/` is served without one.
    security(("bearer" = [])),
    paths(
        robots,
        robot,
        events,
        metrics,
        dlq,
        map,
        adapters,
        shared_resources,
        instant_action_catalog,
        publish_order,
        publish_instant_actions,
    ),
    components(schemas(
            FleetSnapshot,
            AdapterView,
            SharedResourceView,
            DlqView,
            MissionPublished,
            InstantActionsPublished,
            crate::orders::OrderRequest,
            crate::orders::Waypoint,
            crate::orders::ActionSpec,
            crate::orders::InstantActionRequest,
            crate::orders::InstantActionSpec,
            OrderRejection,
            Metrics,
            GatewayEvent,
            vda5050_types::CanonicalRobot,
        ))
)]
pub struct ApiDoc;

/// The spec with its bearer scheme filled in.
///
/// The `#[openapi]` attribute accepts `security(("bearer" = []))` to *require*
/// the scheme but has no way to *define* it, so without this the requirements
/// name a scheme that does not exist: Swagger UI shows no way to supply a
/// token, and the spec references a dangling component.
fn openapi_spec() -> utoipa::openapi::OpenApi {
    let mut spec = ApiDoc::openapi();
    let scheme = utoipa::openapi::security::SecurityScheme::Http(
        utoipa::openapi::security::HttpBuilder::new()
            .scheme(utoipa::openapi::security::HttpAuthScheme::Bearer)
            .bearer_format("opaque token")
            .description(Some(
                "The gateway's --api-token, sent as `Authorization: Bearer <token>`.",
            ))
            .build(),
    );
    if let Some(components) = spec.components.as_mut() {
        components.add_security_scheme("bearer", scheme);
    }
    spec
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
        // Swagger UI owns both the docs page and the spec it reads, at the
        // utoipa convention. Registering our own `/api/openapi.json` as well
        // would collide with the route `url()` creates.
        .merge(SwaggerUi::new("/api/docs").url("/api-docs/openapi.json", openapi_spec()))
        .merge(protected)
        .with_state(state)
}

/// Serve the digital twin dashboard.
async fn index() -> Html<&'static str> {
    Html(include_str!("../static/index.html"))
}

/// All robots.
#[utoipa::path(
    get,
    path = "/api/robots",
    responses((status = 200, body = FleetSnapshot)),
    security(("bearer" = []))
)]
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
#[utoipa::path(
    get,
    path = "/api/robots/{robot_id}",
    params(("robot_id" = String, Path, description = "manufacturer:serialNumber")),
    responses(
        (status = 200, body = vda5050_types::CanonicalRobot),
        (status = 404, description = "no robot with that id")
    ),
    security(("bearer" = []))
)]
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
#[utoipa::path(
    get,
    path = "/api/events",
    responses((status = 200, body = Vec<GatewayEvent>)),
    security(("bearer" = []))
)]
async fn events(State(state): State<AppState>) -> Json<Vec<GatewayEvent>> {
    Json(state.store.events(100).await)
}

/// Metrics counters.
///
/// Field names are `snake_case` on this endpoint — that is what it has always
/// emitted and what the dashboard reads.
#[utoipa::path(
    get,
    path = "/api/metrics",
    responses((status = 200, body = Metrics)),
    security(("bearer" = []))
)]
async fn metrics(State(state): State<AppState>) -> Json<Metrics> {
    Json(state.store.metrics().await)
}

/// One rejected message, as the DLQ shows it.
#[derive(Debug, Clone, serde::Serialize, utoipa::ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct DlqView {
    pub topic: String,
    pub topic_kind: String,
    pub reasons: Vec<String>,
}

/// Recent DLQ entries.
#[utoipa::path(
    get,
    path = "/api/dlq",
    responses((status = 200, body = Vec<DlqView>)),
    security(("bearer" = []))
)]
async fn dlq(State(state): State<AppState>) -> Json<Vec<DlqView>> {
    let entries = state
        .store
        .dlq(50)
        .await
        .iter()
        .map(|e| DlqView {
            topic: e.topic.clone(),
            topic_kind: e.topic_kind.to_string(),
            reasons: e.reasons.clone(),
        })
        .collect();
    Json(entries)
}

/// The building map used by the dashboard.
///
/// Free-form: the map file is passed through verbatim, so its shape is
/// whatever the served map happens to contain.
#[utoipa::path(
    get,
    path = "/api/map",
    responses((status = 200, body = Value)),
    security(("bearer" = []))
)]
async fn map(State(state): State<AppState>) -> Json<Value> {
    Json((*state.map_json).clone())
}

/// One shared resource as `/api/adapters` reports it.
#[derive(Debug, Clone, serde::Serialize, utoipa::ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct SharedResourceEntry {
    pub id: String,
    pub kind: String,
    pub contenders: Vec<String>,
}

/// One registered fleet adapter and what they can do.
#[derive(Debug, Clone, serde::Serialize, utoipa::ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct AdapterView {
    pub manufacturer: String,
    pub robots: usize,
    pub robot_ids: Vec<String>,
    pub shared_resources: Vec<SharedResourceEntry>,
}

/// Registered fleet adapters and what they can do.
#[utoipa::path(
    get,
    path = "/api/adapters",
    responses((status = 200, body = Vec<AdapterView>)),
    security(("bearer" = []))
)]
async fn adapters(State(state): State<AppState>) -> Json<Vec<AdapterView>> {
    let mut out = Vec::new();
    for adapter in state.store.adapters().await {
        let robots = adapter.robots().await;
        out.push(AdapterView {
            manufacturer: adapter.manufacturer().to_owned(),
            robots: robots.len(),
            robot_ids: robots.iter().map(|r| r.robot_id.clone()).collect(),
            shared_resources: adapter
                .shared_resources()
                .await
                .into_iter()
                .map(|r| SharedResourceEntry {
                    id: r.id,
                    kind: format!("{:?}", r.kind),
                    contenders: r.contenders,
                })
                .collect(),
        });
    }
    Json(out)
}

/// A shared resource and who is holding it.
#[derive(Debug, Clone, serde::Serialize, utoipa::ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct SharedResourceView {
    pub id: String,
    pub kind: String,
    pub contenders: Vec<String>,
    pub held_by: Vec<String>,
    pub manufacturer: String,
}

/// Shared resources (doors, lifts) robots may contend for.
///
/// The dashboard uses this to explain why a robot is waiting: a robot stopped
/// at a door and a robot that lost its connection look identical in a
/// position plot.
#[utoipa::path(
    get,
    path = "/api/shared-resources",
    responses((status = 200, body = Vec<SharedResourceView>)),
    security(("bearer" = []))
)]
async fn shared_resources(State(state): State<AppState>) -> Json<Vec<SharedResourceView>> {
    let mut out = Vec::new();
    for adapter in state.store.adapters().await {
        for resource in adapter.shared_resources().await {
            let mut held_by = Vec::new();
            for robot in adapter.robots().await {
                if adapter.holds(&robot.robot_id, &resource.id).await {
                    held_by.push(robot.robot_id);
                }
            }
            out.push(SharedResourceView {
                id: resource.id,
                kind: format!("{:?}", resource.kind),
                contenders: resource.contenders,
                held_by,
                manufacturer: adapter.manufacturer().to_owned(),
            });
        }
    }
    Json(out)
}

/// The instant actions the control screen offers.
#[utoipa::path(
    get,
    path = "/api/instant-actions",
    responses((status = 200, body = Vec<String>)),
    security(("bearer" = []))
)]
async fn instant_action_catalog() -> Json<&'static [&'static str]> {
    Json(INSTANT_ACTIONS)
}

/// What an instant-action submission returns.
#[derive(Debug, Clone, serde::Serialize, utoipa::ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct InstantActionsPublished {
    pub status: &'static str,
    pub robot_id: String,
    pub header_id: u64,
    pub actions: Vec<String>,
}

/// `POST /api/robots/{robot_id}/instant-actions` — pause, resume or cancel.
///
/// This is the counterpart to a mission: without it an operator who sent a
/// wrong order has no way to stop the robot short of killing the simulator.
#[utoipa::path(
    post,
    path = "/api/robots/{robot_id}/instant-actions",
    params(("robot_id" = String, Path, description = "manufacturer:serialNumber")),
    request_body = InstantActionRequest,
    responses(
        (status = 200, body = InstantActionsPublished),
        (status = 400, body = OrderRejection, description = "the action was rejected before publication")
    ),
    security(("bearer" = []))
)]
async fn publish_instant_actions(
    State(state): State<AppState>,
    Path(robot_id): Path<String>,
    request: Result<axum::Json<InstantActionRequest>, axum::extract::rejection::JsonRejection>,
) -> Result<Json<InstantActionsPublished>, OrderRejection> {
    let axum::Json(request) = request?;
    let message =
        crate::orders::submit_instant(&state.publisher, &state.store, &robot_id, &request)
            .await
            .map_err(|reasons| OrderRejection {
                error: "invalid_instant_action",
                reasons,
            })?;

    Ok(Json(InstantActionsPublished {
        status: "published",
        robot_id,
        header_id: message.header_id,
        actions: message
            .actions
            .iter()
            .map(|a| a.action_type.clone())
            .collect(),
    }))
}

/// WebSocket endpoint streaming gateway events.
async fn ws_handler(ws: WebSocketUpgrade, State(state): State<AppState>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_socket(socket, state))
}

/// A rejection the dashboard can show verbatim.
#[derive(Debug, Serialize, utoipa::ToSchema)]
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
/// What a mission submission returns.
#[derive(Debug, Clone, serde::Serialize, utoipa::ToSchema)]
#[serde(rename_all = "camelCase")]
pub struct MissionPublished {
    pub status: &'static str,
    pub robot_id: String,
    pub order_id: String,
    pub header_id: u64,
    pub waypoints: usize,
}

#[utoipa::path(
    post,
    path = "/api/orders",
    request_body = OrderRequest,
    responses(
        (status = 200, body = MissionPublished),
        (status = 400, body = OrderRejection, description = "the mission was rejected before publication")
    ),
    security(("bearer" = []))
)]
async fn publish_order(
    State(state): State<AppState>,
    request: Result<axum::Json<OrderRequest>, axum::extract::rejection::JsonRejection>,
) -> Result<Json<MissionPublished>, OrderRejection> {
    let axum::Json(request) = request?;
    let assembler = OrderAssembler::new(&state.map_json);
    let order = crate::orders::submit(&assembler, &state.publisher, &state.store, &request)
        .await
        .map_err(|reasons| OrderRejection {
            error: "invalid_order",
            reasons,
        })?;
    state.store.count_published().await;

    Ok(Json(MissionPublished {
        status: "published",
        robot_id: format!("{}:{}", order.manufacturer, order.serial_number),
        order_id: order.order_id,
        header_id: order.header_id,
        waypoints: order.nodes.len(),
    }))
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
