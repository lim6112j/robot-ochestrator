//! Tests for the bearer-token gate on the gateway HTTP surface.
//!
//! `axum::Router` is driven directly with `tower::ServiceExt::oneshot`, so no
//! listener or broker is needed: the point is who gets through, not what the
//! handlers return.

use std::sync::Arc;

use axum::body::Body;
use axum::http::{Request, StatusCode};
use tower::ServiceExt;

use gateway::api::{self, AppState};
use gateway::auth::ApiToken;
use gateway::store::Store;
use serde_json::json;
use vda5050_types::MessageValidator;

/// A warehouse map so `/api/orders` has geometry to look up.
fn map_json() -> serde_json::Value {
    json!({
        "mapId": "plant-b/l1",
        "nodes": {
            "RK-AM": { "x": 18.0, "y": 4.0, "theta": 0.0 },
            "CHG-01": { "x": 0.0, "y": 24.0, "theta": 0.0 }
        },
        "edges": []
    })
}

/// Build a router with the given token, without touching a broker.
///
/// The publisher owns an unconnected MQTT client: authentication is checked
/// before any handler runs, so no publish is ever attempted in these tests.
fn router_with(token: ApiToken) -> axum::Router {
    let map = Arc::new(map_json());
    let store = Store::new(10.0);
    let (client, _eventloop) = gateway::ingest::connect("localhost", 1883, "test");
    let state = AppState {
        store,
        map_json: Arc::clone(&map),
        publisher: Arc::new(gateway::orders::OrderPublisher::new(
            Arc::new(client),
            Arc::new(MessageValidator::new().expect("schemas compile")),
            "vda5050".to_owned(),
            3,
        )),
        validator: Arc::new(MessageValidator::new().expect("schemas compile")),
        token,
    };
    api::router(state)
}

fn get(uri: &str) -> Request<Body> {
    Request::builder()
        .uri(uri)
        .body(Body::empty())
        .expect("request")
}

fn get_with_token(uri: &str, token: &str) -> Request<Body> {
    Request::builder()
        .uri(uri)
        .header("Authorization", format!("Bearer {token}"))
        .body(Body::empty())
        .expect("request")
}

fn post_order(body: serde_json::Value) -> Request<Body> {
    Request::builder()
        .method("POST")
        .uri("/api/orders")
        .header("Content-Type", "application/json")
        .body(Body::from(serde_json::to_vec(&body).expect("serialises")))
        .expect("request")
}

fn valid_order() -> serde_json::Value {
    json!({
        "manufacturer": "acme",
        "serialNumber": "AMR-001",
        "orderId": "order-auth-test",
        "nodes": [{ "nodeId": "RK-AM", "actions": [] }]
    })
}

#[tokio::test]
async fn without_a_token_the_api_is_open() {
    // Default dev posture: no token configured means nothing is rejected.
    let app = router_with(ApiToken::new(None));
    let response = app.oneshot(get("/api/robots")).await.expect("served");
    assert_eq!(response.status(), StatusCode::OK);
}

#[tokio::test]
async fn with_a_token_a_bare_request_is_rejected() {
    let app = router_with(ApiToken::new(Some("s3cret".to_owned())));
    let response = app.oneshot(get("/api/robots")).await.expect("served");
    assert_eq!(response.status(), StatusCode::UNAUTHORIZED);
}

#[tokio::test]
async fn with_a_token_the_right_one_is_accepted() {
    let app = router_with(ApiToken::new(Some("s3cret".to_owned())));
    let response = app
        .oneshot(get_with_token("/api/robots", "s3cret"))
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::OK);
}

#[tokio::test]
async fn a_wrong_token_is_rejected() {
    let app = router_with(ApiToken::new(Some("s3cret".to_owned())));
    let response = app
        .oneshot(get_with_token("/api/robots", "wrong"))
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::UNAUTHORIZED);
}

#[tokio::test]
async fn a_non_bearer_scheme_is_rejected() {
    // `Basic` must not sneak past the gate.
    let app = router_with(ApiToken::new(Some("s3cret".to_owned())));
    let request = Request::builder()
        .uri("/api/robots")
        .header("Authorization", "Basic s3cret")
        .body(Body::empty())
        .expect("request");
    let response = app.oneshot(request).await.expect("served");
    assert_eq!(response.status(), StatusCode::UNAUTHORIZED);
}

#[tokio::test]
async fn a_websocket_can_authenticate_by_query_parameter() {
    // Browsers cannot set headers on a WebSocket, so the token rides in the
    // query string.
    //
    // A real upgrade handshake is used, because without the WebSocket headers
    // axum rejects with 400 before authentication even matters. `oneshot` runs
    // on a mock connection, so a valid token cannot reach the 101 of a real
    // upgrade: it reaches 426 `ConnectionNotUpgradable` instead, which still
    // proves the gate let the request through to the WebSocket handler.
    let upgrade = |uri: &str| {
        Request::builder()
            .uri(uri)
            .header("Connection", "Upgrade")
            .header("Upgrade", "websocket")
            .header("Sec-WebSocket-Version", "13")
            .header("Sec-WebSocket-Key", "dGhlIHNhbXBsZSBub25jZQ==")
            .body(Body::empty())
            .expect("request")
    };

    let app = router_with(ApiToken::new(Some("s3cret".to_owned())));
    let response = app
        .oneshot(upgrade("/ws?token=s3cret"))
        .await
        .expect("served");
    assert_eq!(
        response.status(),
        StatusCode::UPGRADE_REQUIRED,
        "the token was accepted and the request reached the WebSocket handler"
    );

    let app = router_with(ApiToken::new(Some("s3cret".to_owned())));
    let response = app.oneshot(upgrade("/ws")).await.expect("served");
    assert_eq!(response.status(), StatusCode::UNAUTHORIZED);

    let app = router_with(ApiToken::new(Some("s3cret".to_owned())));
    let response = app
        .oneshot(upgrade("/ws?token=wrong"))
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::UNAUTHORIZED);
}

#[tokio::test]
async fn the_static_dashboard_is_served_without_a_token() {
    // The page must load so it can ask for the token in the first place.
    let app = router_with(ApiToken::new(Some("s3cret".to_owned())));
    let response = app.oneshot(get("/")).await.expect("served");
    assert_eq!(response.status(), StatusCode::OK);
}

#[tokio::test]
async fn mission_dispatch_requires_a_token() {
    // The whole point: driving hardware must not be reachable by port access.
    let app = router_with(ApiToken::new(Some("s3cret".to_owned())));
    let response = app
        .oneshot(post_order(valid_order()))
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::UNAUTHORIZED);
}

#[tokio::test]
async fn an_invalid_order_is_rejected_before_authentication_is_checked() {
    // Order validation is the handler's job; the gate runs first, so an
    // unauthenticated malformed order must not leak validation details.
    let app = router_with(ApiToken::new(Some("s3cret".to_owned())));
    let mut broken = valid_order();
    broken["nodes"][0]["nodeId"] = json!("NOWHERE");
    let response = app.oneshot(post_order(broken)).await.expect("served");
    assert_eq!(response.status(), StatusCode::UNAUTHORIZED);
}
