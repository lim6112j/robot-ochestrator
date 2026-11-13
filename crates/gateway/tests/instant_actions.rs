//! Tests for the instant-action endpoint (`/api/robots/{id}/instant-actions`).
//!
//! Driven through the router like `auth.rs`, so no broker is needed: the point
//! is that a pause/resume/cancel can be issued from the control screen, that
//! it is validated first, and that it stays behind the token gate.

use std::sync::Arc;

use axum::body::Body;
use axum::http::{Request, StatusCode};
use serde_json::{json, Value};
use tower::ServiceExt;

use gateway::api::{self, AppState};
use gateway::auth::ApiToken;
use gateway::store::Store;
use vda5050_types::MessageValidator;

fn map_json() -> Value {
    json!({
        "mapId": "plant-b/l1",
        "nodes": { "RK-AM": { "x": 18.0, "y": 4.0, "theta": 0.0 } },
        "edges": []
    })
}

fn router_with(token: ApiToken) -> axum::Router {
    let state = AppState {
        store: Store::new(10.0),
        map_json: Arc::new(map_json()),
        publisher: Arc::new(gateway::orders::OrderPublisher::new(
            Arc::new(gateway::ingest::connect("localhost", 1883, "test").0),
            Arc::new(MessageValidator::new().expect("schemas compile")),
            "vda5050".to_owned(),
            3,
        )),
        validator: Arc::new(MessageValidator::new().expect("schemas compile")),
        token,
    };
    api::router(state)
}

fn post(robot_id: &str, body: Value) -> Request<Body> {
    Request::builder()
        .method("POST")
        .uri(format!("/api/robots/{robot_id}/instant-actions"))
        .header("Content-Type", "application/json")
        .body(Body::from(serde_json::to_vec(&body).expect("serialises")))
        .expect("request")
}

fn with_token(request: Request<Body>) -> Request<Body> {
    let (mut parts, body) = request.into_parts();
    parts.headers.insert(
        "Authorization",
        "Bearer test-secret".parse().expect("header"),
    );
    Request::from_parts(parts, body)
}

#[tokio::test]
async fn the_catalog_is_published() {
    let app = router_with(ApiToken::new(None));
    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/instant-actions")
                .body(Body::empty())
                .expect("request"),
        )
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::OK);
    let body = axum::body::to_bytes(response.into_body(), usize::MAX)
        .await
        .expect("body");
    let actions: Vec<String> = serde_json::from_slice(&body).expect("json");
    assert!(actions.contains(&"cancelOrder".to_owned()));
    assert!(actions.contains(&"startPause".to_owned()));
    assert!(actions.contains(&"stopPause".to_owned()));
}

#[tokio::test]
async fn a_robot_id_must_split_into_manufacturer_and_serial() {
    let app = router_with(ApiToken::new(None));
    let response = app
        .oneshot(post(
            "no-colon",
            json!({ "actions": [{ "actionType": "cancelOrder" }] }),
        ))
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    let body = axum::body::to_bytes(response.into_body(), usize::MAX)
        .await
        .expect("body");
    let payload: Value = serde_json::from_slice(&body).expect("json");
    assert_eq!(payload["error"], json!("invalid_instant_action"));
    assert!(payload["reasons"][0]
        .as_str()
        .expect("string")
        .contains("manufacturer:serialNumber"));
}

#[tokio::test]
async fn an_empty_action_list_is_rejected() {
    let app = router_with(ApiToken::new(None));
    let response = app
        .oneshot(post("acme:AMR-001", json!({ "actions": [] })))
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::BAD_REQUEST);
}

#[tokio::test]
async fn a_blank_action_type_is_rejected() {
    let app = router_with(ApiToken::new(None));
    let response = app
        .oneshot(post(
            "acme:AMR-001",
            json!({ "actions": [{ "actionType": "   " }] }),
        ))
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::BAD_REQUEST);
}

#[tokio::test]
async fn duplicate_action_ids_are_rejected() {
    let app = router_with(ApiToken::new(None));
    let response = app
        .oneshot(post(
            "acme:AMR-001",
            json!({ "actions": [
                { "actionType": "startPause", "actionId": "dup" },
                { "actionType": "stopPause",  "actionId": "dup" }
            ] }),
        ))
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::BAD_REQUEST);
}

#[tokio::test]
async fn instant_actions_require_a_token() {
    let app = router_with(ApiToken::new(Some("test-secret".to_owned())));
    let response = app
        .oneshot(post(
            "acme:AMR-001",
            json!({ "actions": [{ "actionType": "cancelOrder" }] }),
        ))
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::UNAUTHORIZED);
}

#[tokio::test]
async fn the_assembled_message_is_schema_valid() {
    // Same shape `submit_instant` builds, checked against the official schema:
    // `actionId`, `actionType` and `blockingType` are all required.
    let validator = MessageValidator::new().expect("schemas compile");
    let message = json!({
        "headerId": 1,
        "timestamp": "2026-01-01T00:00:00.000Z",
        "version": "3.0.0",
        "manufacturer": "acme",
        "serialNumber": "AMR-001",
        "actions": [
            { "actionType": "cancelOrder", "actionId": "ia1",
              "blockingType": "NONE",
              "actionParameters": [{ "key": "kill", "value": "false" }] }
        ]
    });
    validator
        .validate(vda5050_types::Topic::InstantActions, &message)
        .expect("valid instantActions");
}

#[tokio::test]
async fn a_missing_blocking_type_is_invalid() {
    // The schema requires it even though every instant action is all-or-nothing.
    let validator = MessageValidator::new().expect("schemas compile");
    let message = json!({
        "headerId": 1,
        "timestamp": "2026-01-01T00:00:00.000Z",
        "version": "3.0.0",
        "manufacturer": "acme",
        "serialNumber": "AMR-001",
        "actions": [{ "actionType": "cancelOrder", "actionId": "ia1" }]
    });
    validator
        .validate(vda5050_types::Topic::InstantActions, &message)
        .expect_err("blockingType is required");
}

#[tokio::test]
async fn an_instant_action_must_not_be_blocking() {
    // v3: "The blocking type of an instant action is always NONE: action can
    // happen in parallel with others, including movement." Unlike order
    // actions, where HARD is the default and means "finish before driving on".
    let validator = MessageValidator::new().expect("schemas compile");
    let message = json!({
        "headerId": 1,
        "timestamp": "2026-01-01T00:00:00.000Z",
        "version": "3.0.0",
        "manufacturer": "acme",
        "serialNumber": "AMR-001",
        "actions": [{
            "actionType": "cancelOrder", "actionId": "ia1", "blockingType": "HARD"
        }]
    });
    let reasons = validator
        .validate(vda5050_types::Topic::InstantActions, &message)
        .expect_err("HARD is not allowed for instant actions");
    assert!(reasons.iter().any(|r| r.contains("NONE")), "{reasons:?}");
}

#[tokio::test]
async fn an_authenticated_request_reaches_the_broker() {
    // The publisher owns an unconnected client, so the publish fails at the
    // broker rather than at the gate: reaching that point is what matters.
    let app = router_with(ApiToken::new(Some("test-secret".to_owned())));
    let response = app
        .oneshot(with_token(post(
            "acme:AMR-001",
            json!({ "actions": [{ "actionType": "startPause" }] }),
        )))
        .await
        .expect("served");
    // Past authentication and past schema validation, the only thing left is
    // the broker, and this test owns an unconnected client.
    assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    let body = axum::body::to_bytes(response.into_body(), usize::MAX)
        .await
        .expect("body");
    let payload: Value = serde_json::from_slice(&body).expect("json");
    assert_eq!(payload["error"], json!("invalid_instant_action"));
    assert!(
        payload["reasons"][0]
            .as_str()
            .expect("string")
            .contains("broker"),
        "the failure is the broker, not the gate or the schema: {payload}"
    );
}
