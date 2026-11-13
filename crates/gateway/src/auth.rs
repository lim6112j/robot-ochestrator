//! Bearer-token authentication for the gateway HTTP surface.
//!
//! Without a token the gateway is open, which is only acceptable on a
//! loopback dev broker. Once `--api-token` is set, every `/api/*` route and
//! the WebSocket require it: dispatching a mission is a *write* on physical
//! hardware, so "can reach the port" must not be enough.
//!
//! The comparison is constant-time: a token is a shared secret and timing
//! oracles on `==` are how those leak.

use axum::body::Body;
use axum::extract::State;
use axum::http::{Request, StatusCode};
use axum::middleware::Next;
use axum::response::{IntoResponse, Response};
use serde::{Deserialize, Serialize};

/// The shared secret, or `None` when authentication is disabled.
#[derive(Debug, Clone)]
pub struct ApiToken(Option<String>);

impl ApiToken {
    /// Create from a CLI value. Empty means "no authentication".
    #[must_use]
    pub fn new(raw: Option<String>) -> Self {
        Self(raw.filter(|t| !t.trim().is_empty()))
    }

    /// Whether a token is configured.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.0.is_some()
    }

    /// Constant-time comparison against the configured token.
    fn matches(&self, candidate: &str) -> bool {
        match &self.0 {
            None => false,
            Some(expected) => constant_time_eq(expected.as_bytes(), candidate.as_bytes()),
        }
    }
}

/// Byte-wise equality that does not short-circuit on the first difference.
fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut diff = 0_u8;
    for (x, y) in a.iter().zip(b.iter()) {
        diff |= x ^ y;
    }
    diff == 0
}

/// Rejection returned when a request carries no valid token.
#[derive(Debug, Serialize)]
pub struct Unauthorized {
    pub error: &'static str,
}

impl IntoResponse for Unauthorized {
    fn into_response(self) -> Response {
        (StatusCode::UNAUTHORIZED, axum::Json(self)).into_response()
    }
}

/// Reject requests without a valid token when one is configured.
///
/// Applied to the whole router: reads are not harmless either — fleet
/// positions and order history are operationally sensitive.
pub async fn require_token(
    State(token): State<ApiToken>,
    request: Request<Body>,
    next: Next,
) -> Response {
    if !token.is_enabled() {
        return next.run(request).await;
    }

    let presented = bearer_token(request.headers().get(axum::http::header::AUTHORIZATION))
        .or_else(|| query_token(request.uri().query()));

    match presented {
        Some(candidate) if token.matches(&candidate) => next.run(request).await,
        _ => Unauthorized {
            error: "missing_or_invalid_token",
        }
        .into_response(),
    }
}

/// Extract the token from an `Authorization: Bearer <token>` header.
fn bearer_token(header: Option<&axum::http::HeaderValue>) -> Option<String> {
    let value = header?.to_str().ok()?;
    let (scheme, token) = value.split_once(' ')?;
    if !scheme.eq_ignore_ascii_case("bearer") {
        return None;
    }
    Some(token.trim().to_owned())
}

/// Extract the token from a `?token=` query parameter.
///
/// Browsers cannot set headers on a `WebSocket`, so the dashboard passes the
/// token this way. The dashboard never logs it.
fn query_token(query: Option<&str>) -> Option<String> {
    let query = query?;
    query.split('&').find_map(|pair| {
        let (key, value) = pair.split_once('=')?;
        (key == "token").then(|| value.to_owned())
    })
}

/// Body of the token prompt shown by the dashboard.
#[derive(Debug, Deserialize)]
pub struct TokenPrompt {
    pub token: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_configuration_means_open() {
        assert!(!ApiToken::new(None).is_enabled());
        assert!(!ApiToken::new(Some(String::new())).is_enabled());
        assert!(!ApiToken::new(Some("   ".to_owned())).is_enabled());
    }

    #[test]
    fn a_configured_token_matches_exactly() {
        let token = ApiToken::new(Some("s3cret".to_owned()));
        assert!(token.is_enabled());
        assert!(token.matches("s3cret"));
        assert!(!token.matches("s3cre"));
        assert!(!token.matches("s3cretX"));
        assert!(!token.matches(""));
        assert!(!token.matches("S3CRET"), "tokens are case sensitive");
    }

    #[test]
    fn an_open_gateway_matches_nothing() {
        // A request must never authenticate against a disabled token, even by
        // sending an empty bearer.
        let open = ApiToken::new(None);
        assert!(!open.matches(""));
        assert!(!open.matches("anything"));
    }

    #[test]
    fn constant_time_eq_rejects_length_mismatch() {
        assert!(constant_time_eq(b"abc", b"abc"));
        assert!(!constant_time_eq(b"abc", b"abd"));
        assert!(!constant_time_eq(b"abc", b"ab"));
        assert!(!constant_time_eq(b"", b"a"));
    }

    #[test]
    fn bearer_token_is_parsed_case_insensitively() {
        let header = axum::http::HeaderValue::from_static("Bearer abc123");
        assert_eq!(bearer_token(Some(&header)).as_deref(), Some("abc123"));

        let lower = axum::http::HeaderValue::from_static("bearer abc123");
        assert_eq!(bearer_token(Some(&lower)).as_deref(), Some("abc123"));

        let basic = axum::http::HeaderValue::from_static("Basic abc123");
        assert_eq!(bearer_token(Some(&basic)), None, "only Bearer is accepted");

        let no_scheme = axum::http::HeaderValue::from_static("abc123");
        assert_eq!(bearer_token(Some(&no_scheme)), None);
        assert_eq!(bearer_token(None), None);
    }

    #[test]
    fn query_token_is_extracted_for_websockets() {
        assert_eq!(query_token(Some("token=abc")).as_deref(), Some("abc"));
        assert_eq!(
            query_token(Some("foo=1&token=abc&bar=2")).as_deref(),
            Some("abc")
        );
        assert_eq!(query_token(Some("foo=1")), None);
        assert_eq!(query_token(Some("tokenx=abc")), None);
        assert_eq!(query_token(None), None);
    }
}
