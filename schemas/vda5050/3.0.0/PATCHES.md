# v3.0.0 스키마 반입 시 수정 사항

원본: `github.com/vda5050/vda5050` 태그 `3.0.0` 의 `json_schemas/*.schema`

공식 파일 일부가 **JSON 문법 오류(trailing comma)** 를 포함하여 파서가 로드하지 못한다.
(스펙 문서와 스키마가 다를 경우 문서가 우선한다는 원칙과, "스키마 오류는 릴리스 없이 main에서
바로 수정된다"는 저장소 정책에 따라 로컬에서 복원했다.)

| 파일 | 수정 내용 |
|---|---|
| `factsheet.schema` | trailing comma 4건 제거 |
| `order.schema` | trailing comma 1건 제거 |
| `visualization.schema` | trailing comma 2건 제거 |

수정은 **구조·의미 변경 없이** 문법 복원만 수행했으며, `tools/verify-schemas`가 8개 파일 전체를
JSON 파싱 + `jsonschema` 메타스키마로 검증한다.

## v3.0.0 주의 필드명 (v2 → v3 변경)
| v2 | v3.0.0 |
|---|---|
| `header` | `headerId` |
| `agvPosition` | `mobileRobotPosition` |
| `batteryState` / `batteryCharge` | `powerSupply` / `stateOfCharge` |
| `safetyState.eStop` | `safetyState.activeEmergencyStop` |
| `positionInitialized` | `mobileRobotPosition.localized` |
| `errors[].errorType` (고정 enum) | 확장 가능 enum |
| `actionStates[].status` | `actionStates[].actionStatus` (`WAITING/INITIALIZING/RUNNING/PAUSED/RETRIABLE/FINISHED/FAILED`) |
| `pauses` | `paused` (boolean) |
| `edge.rotationAllowed` | `edge.reachOrientationBeforeEntering` (의미 반전) |
| `responses` 토픽 = 주문 수락/거부 | **존/엣지 리스 응답 전용**(`requestId`, `grantType` ∈ GRANTED/QUEUED/REVOKED/REJECTED) |
| 주문 거부 통보 | v3에서는 `responses`가 아닌 **`state.errors`**(`VALIDATION_FAILURE`, `INVALID_ORDER_ACTION` 등)로 보고 |
| `operatingMode` | `STARTUP`/`INTERVENED` 추가, `TEACH_IN`(v2 `TEACHIN`) |
| `blockingType` | `SINGLE` 추가 |
| `errorLevel` | `URGENT`/`CRITICAL` 추가 |

QoS: `order`, `instantActions`, `state`, `visualization`, `factsheet`, `zoneSet`, `responses` = **QoS 0**,
`connection` = **QoS 1**.
