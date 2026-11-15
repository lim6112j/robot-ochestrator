//! The generated `OpenAPI` spec.
//!
//! The point of deriving the spec from the handlers is that it cannot drift
//! from the code. That claim is only worth something if something checks it,
//! so these tests read the spec and compare it against what the routes
//! actually serve.

use std::sync::Arc;

use axum::body::Body;
use axum::http::{Request, StatusCode};
use serde_json::{json, Value};
use tower::ServiceExt;

use gateway::api::{self, AppState};
use gateway::auth::ApiToken;
use gateway::store::Store;
use vda5050_types::MessageValidator;

fn router_with(token: ApiToken) -> axum::Router {
    let map = Arc::new(json!({
        "mapId": "plant-b/l1",
        "nodes": {
            "RK-AM": { "x": 18.0, "y": 4.0, "theta": 0.0 },
            "CHG-01": { "x": 0.0, "y": 24.0, "theta": 0.0 }
        },
        "edges": []
    }));
    let store = Store::new(10.0);
    let (client, _eventloop) = gateway::ingest::connect("localhost", 1883, "test");
    let state = AppState {
        store,
        map_json: map,
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

/// Fetch `/api-docs/openapi.json` through the real router.
async fn spec(token: ApiToken) -> Value {
    let response = router_with(token)
        .oneshot(
            Request::builder()
                .uri("/api-docs/openapi.json")
                .body(Body::empty())
                .expect("request"),
        )
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::OK);
    let bytes = axum::body::to_bytes(response.into_body(), 1 << 20)
        .await
        .expect("body");
    serde_json::from_slice(&bytes).expect("valid json")
}

#[tokio::test]
async fn every_route_the_router_serves_is_in_the_spec() {
    // A handler added without an `#[utoipa::path]` attribute would otherwise be
    // invisible to every client that reads the spec instead of the source.
    let spec = spec(ApiToken::new(None)).await;
    let documented: Vec<&str> = spec["paths"]
        .as_object()
        .expect("paths is an object")
        .keys()
        .map(String::as_str)
        .collect();

    for path in [
        "/api/robots",
        "/api/robots/{robot_id}",
        "/api/events",
        "/api/metrics",
        "/api/dlq",
        "/api/map",
        "/api/orders",
        "/api/adapters",
        "/api/shared-resources",
        "/api/instant-actions",
        "/api/robots/{robot_id}/instant-actions",
    ] {
        assert!(
            documented.contains(&path),
            "{path} is routed but missing from the spec"
        );
    }
    assert_eq!(
        documented.len(),
        11,
        "the spec should describe exactly the routed API, got {documented:?}"
    );
}

#[tokio::test]
async fn every_security_reference_names_a_defined_scheme() {
    // `security(("bearer" = []))` on a handler *requires* the scheme but does
    // not define it. Without the definition Swagger UI shows no way to send a
    // token, and the spec points at a component that is not there.
    let spec = spec(ApiToken::new(None)).await;
    let defined: Vec<String> = spec["components"]["securitySchemes"]
        .as_object()
        .expect("securitySchemes is an object")
        .keys()
        .cloned()
        .collect();
    assert_eq!(defined, vec!["bearer".to_owned()]);

    let mut referenced = Vec::new();
    for operation in spec["paths"].as_object().expect("paths").values() {
        for (method, op) in operation.as_object().expect("operation") {
            for requirement in op["security"].as_array().into_iter().flatten() {
                for name in requirement.as_object().expect("requirement").keys() {
                    referenced.push(format!("{method} {name}"));
                }
            }
        }
    }
    assert!(
        referenced.iter().all(|r| r.ends_with("bearer")),
        "unexpected scheme referenced: {referenced:?}"
    );
    assert_eq!(
        referenced.len(),
        11,
        "every operation should require the token, got {referenced:?}"
    );
}

#[tokio::test]
async fn the_spec_is_open_but_the_api_is_not() {
    // A client has to read the spec to learn that it needs a token, so the
    // spec cannot be behind the gate.
    let app = router_with(ApiToken::new(None));
    let response = app
        .oneshot(
            Request::builder()
                .uri("/api-docs/openapi.json")
                .body(Body::empty())
                .expect("request"),
        )
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::OK);

    // With a token configured, the API refuses a bare request — but the spec
    // still answers, because it is what tells the client a token is needed.
    let guarded = ApiToken::new(Some("s3cret".to_owned()));

    let response = router_with(guarded.clone())
        .oneshot(
            Request::builder()
                .uri("/api/robots")
                .body(Body::empty())
                .expect("request"),
        )
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::UNAUTHORIZED);

    let response = router_with(guarded)
        .oneshot(
            Request::builder()
                .uri("/api-docs/openapi.json")
                .body(Body::empty())
                .expect("request"),
        )
        .await
        .expect("served");
    assert_eq!(response.status(), StatusCode::OK);
}

#[tokio::test]
async fn a_mission_response_matches_its_documented_schema() {
    // The response type is `MissionPublished`, so the schema's properties are
    // exactly the fields the handler sets. If a field is added to one and not
    // the other, the wire format and the spec disagree.
    let spec = spec(ApiToken::new(None)).await;
    let schema = &spec["components"]["schemas"]["MissionPublished"];
    let properties = schema["properties"].as_object().expect("properties");

    for field in ["status", "robotId", "orderId", "headerId", "waypoints"] {
        assert!(
            properties.contains_key(field),
            "MissionPublished is missing {field}: {:?}",
            properties.keys().collect::<Vec<_>>()
        );
    }
    assert_eq!(properties.len(), 5, "unexpected extra fields");
    assert_eq!(schema["required"].as_array().map(Vec::len), Some(5));
}

#[tokio::test]
async fn metrics_keeps_its_snake_case_field_names() {
    // `ToSchema` documents whatever serde emits; it must not be allowed to
    // rename the wire format. The dashboard reads `orders_seen`, and a
    // camelCase rename here would silently break it.
    let spec = spec(ApiToken::new(None)).await;
    let schema = &spec["components"]["schemas"]["Metrics"];
    let properties = schema["properties"].as_object().expect("properties");

    for field in [
        "messages_seen",
        "orders_seen",
        "orders_published",
        "messages_valid",
    ] {
        assert!(
            properties.contains_key(field),
            "Metrics lost {field}: {:?}",
            properties.keys().collect::<Vec<_>>()
        );
    }
}

#[tokio::test]
async fn a_rejection_is_documented_for_both_posts() {
    // The dashboard shows these verbatim, so a client has to know the shape.
    let spec = spec(ApiToken::new(None)).await;
    for path in ["/api/orders", "/api/robots/{robot_id}/instant-actions"] {
        let responses = &spec["paths"][path]["post"]["responses"];
        assert!(
            responses.get("400").is_some(),
            "{path} has no documented 400"
        );
        assert_eq!(
            responses["400"]["content"]["application/json"]["schema"]["$ref"],
            "#/components/schemas/OrderRejection",
            "{path} 400 should reference OrderRejection"
        );
    }
}
