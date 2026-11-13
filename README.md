# 로봇 관제시스템 — 통신·제어 표준화 (VDA 5050 + Open-RMF)

제조사가 다른 물류·청소·감시 로봇을 **단일 화면**에서 제어하기 위한 표준 프로토콜 기반 시스템.

- 설계·로드맵: [`PLAN.md`](PLAN.md)
- 로봇과 관제 사이의 유일한 계약: **VDA 5050 v3.0.0** (MQTT + JSON)
- 다중 로봇 조율: **Open-RMF** (Phase 2 예정)
- 전 스택: **Rust** (workspace)

## 현재 상태 (Phase 0 / Phase 1 일부 완료)

| 구성요소 | 상태 | 내용 |
|---|---|---|
| `crates/vda5050-types` | ✅ | v3.0.0 타입 + 공식 JSON Schema 검증, v2 인그레스 변환, 캐노니컬 모델 |
| `crates/vda5050-sim` | ✅ | 가상 플릿(물류/청소/감시/v2 레거시), order·instantActions·고장 주입 |
| `crates/gateway` | ✅ | MQTT 구독·검증·정규화·프레즌스, REST/WebSocket, 디지털 트윈 대시보드 |
| `tools/conformance` | ✅ | 스키마 검증(`verify-schemas`), 단건 메시지 검증(`check-message`) |
| `tools/broker` | ✅ | 로컬 MQTT 브로커(`rumqttd`) — Docker Hub 차단 환경용 |
| 적합성 시험 | ✅ | 스키마 8종 검증, 단건 메시지 검증, 위반 메시지 거부 확인 |
| Open-RMF 연동 | ⏳ | Phase 2 (`rclrs` fleet adapter 스파이크) |

## 빠른 시작

```bash
# 1) 빌드
cargo build

# 2) MQTT 브로커 (택일)
#   a) 컨테이너: eclipse-mosquitto:2 (Docker Hub pull 가능한 환경)
docker run -d --name rcs-mosquitto -p 1883:1883 \
  -v "$(pwd)/deploy/broker/mosquitto.conf:/mosquitto/config/mosquitto.conf:ro" eclipse-mosquitto:2
#   b) 컨테이너 없이: 저장소의 Rust 브로커
cargo run -p broker -- --port 1883 &

# 3) 가상 로봇 플릿 5대
cargo run -p vda5050-sim -- --config deploy/sim.yaml &

# 4) 게이트웨이 + 디지털 트윈 대시보드
cargo run -p gateway -- --http-addr 127.0.0.1:8090 &
#   브라우저: http://localhost:8090
#   같은 Wi-Fi의 다른 기기(폰/태블릿): http://192.168.49.71:8090
```

- 실시간 갱신: WebSocket `/ws`으로 로봇 상태 push(위치·배터리·액션·모드)
- 맵 조작: 휠 확대(커서 기준) · 드래그 팬 · 더블클릭 확대 · '전체 보기' 버튼
- 창고 맵: `deploy/maps/plant-b.l1.json` (랙 5열 × 통로 4개, 35노드/106엣지, 양방향)
- 임무 지시: 화면의 **'+ 임무 지시'** 버튼 → 로봇 선택 → 지도 노드 클릭으로 경유지 추가 → 액션/지속시간 → 전송

### 임무 지시 (관제 화면에서)

대시보드의 `+ 임무 지시` 버튼을 누르면 오른쪽에 서랍이 열립니다.

1. **로봇** 선택 (온라인 로봇만, 오프라인은 표시만)
2. **임무 ID** 는 자동 생성 (`order-ui-...`). 같은 ID 로 다시 보내면 갱신
3. 지도의 **노드를 클릭**해 경유지를 순서대로 추가 (서랍이 열린 동안 맵이 보이는
   영역에 맞게 축소되므로 클릭이 가능합니다)
4. 경유지별 **액션**(pick/drop/charge/initPosition/patrol 등)과 **지속시간** 지정
5. **임무 전송**

### 즉시 제어 (정지 / 재개 / 취소)

선택한 로봇의 패널에 세 버튼이 생깁니다.

| 버튼 | 액션 | 동작 |
|---|---|---|
| ⏸ 정지 | `startPause` | 주행을 멈추고 제자리에 대기. 임무는 유지 |
| ▶ 재개 | `stopPause` | 대기 해제, 남은 경로 계속 |
| ✖ 임무 취소 | `cancelOrder` | 진행 중 임무를 폐기하고 정지 (확인 질문 있음) |

버튼은 로봇 상태에 따라 활성/비활성이 바뀝니다(이미 정지 중이면 정지 버튼
비활성, 임무가 없으면 취소 버튼 비활성). 즉시액션 처리 결과는 로봇의
`instantActionStates` 로 돌아와 상세 패널에 표시됩니다.

```bash
# API 로 직접
curl -s --noproxy '*' -X POST \
  -H "Authorization: Bearer $TOKEN" \
  http://127.0.0.1:8090/api/robots/acme:AMR-001/instant-actions \
  -H 'Content-Type: application/json' \
  -d '{"actions":[{"actionType":"cancelOrder"}]}'
# 지원 액션 목록
curl -s --noproxy '*' -H "Authorization: Bearer $TOKEN" http://127.0.0.1:8090/api/instant-actions
```

> **VDA 5050 v3 주의**: 즉시액션의 `blockingType` 은 스키마가 `"NONE"` 만
> 허용합니다(이동과 병렬 실행). 임무 액션의 기본값이 `HARD`("다 끝내고 나서
> 이동")인 것과 정반대이므로, 순서 액션용 코드를 그대로 쓰면 거부됩니다.

운영자가 건네줄 것은 *경로* 뿐입니다. `headerId`(정수 증가), `sequenceId`(노드 짝수·
엣지 홀수), 노드 좌표·엣지 길이(서비스 중인 맵에서 조회)는 게이트웨이가 채우고,
공식 스키마로 검증한 뒤에 발행합니다. 거부되면 사유를 한 번에 전부 돌려줍니다.

```bash
# 동일한 작업을 API 로 직접 할 수도 있습니다
curl -s --noproxy '*' -X POST http://127.0.0.1:8090/api/orders \
  -H 'Content-Type: application/json' -d '{
    "manufacturer": "acme", "serialNumber": "AMR-001",
    "orderId": "order-curl-1",
    "nodes": [
      { "nodeId": "RK-AM", "actions": [{ "actionType": "pick", "durationS": 4 }] },
      { "nodeId": "CHG-01", "actions": [] }
    ]
  }'
# → {"headerId":...,"orderId":"order-curl-1","robotId":"acme:AMR-001","status":"published","waypoints":2}
```

```bash
# 창고형 맵으로 기동
cargo run -p vda5050-sim -- --config deploy/sim-warehouse.yaml &
cargo run -p gateway -- --map deploy/maps/plant-b.l1.json --http-addr 0.0.0.0:8090 &
```

### 공유 자원 (문)

`deploy/rmf/door_adapter` 가 RMF `door_states` 를 발행하고 `door_requests` 를
수락합니다. `DOOR-IN`/`DOOR-OUT` 은 `lif2rmf` 가 이미 RMF 문으로 모델링하고
내비 그래프의 차선 8개에 `door_name` 을 붙였지만, 상태를 발행하는 어댑터가
없으면 문은 영원히 닫혀 있어 문을 지나는 임무가 전부 정체합니다.

`MODE_OPEN` 은 "물리적으로 열림" 이 아니라 **"사용 가능"** 입니다. 영구
`MODE_CLOSED` 는 교착을, 무조건 `MODE_OPEN` 은 예약 자체를 무효화합니다.
요청 시 열고 hold-off(기본 6초) 후 다시 닡습니다.

```bash
docker run --rm -it --network host \
  -v "$PWD/deploy/rmf/door_adapter:/rmf/door_adapter" \
  rcs-rmf:jazzy
# 컨테이너 안에서:
ros2 run vda5050_door_adapter door_adapter -c /rmf/door_adapter/config.yaml
```

관제에서 문 점유를 보려면 `GET /api/shared-resources`.

### 문 상호배타 (시뮬레이터)

시뮬레이터의 `DoorRegistry`가 플릿 전체가 공유하는 문 점유를 관리합니다.
두 제조사 로봇에게 같은 문 경유 임무를 동시에 내리면, 두 번째 로봇은 문
앞에서 대기합니다:

```
waiting for door ... holder=acme:AMR-002   (robot=sweepco:SCR-001)
door granted, resuming                     (robot=sweepco:SCR-001)
```

VDA 5050 v3에 "공유 자원 대기" 필드가 없어, 대기는 `state.errors` 의
`WAITING_FOR_DOOR` 로 전달되고 `RobotAdapter::holds()` 가 읽습니다.
`errorReferences` 는 `{referenceKey, referenceValue}` 객체 배열이라 문자열을
넣으면 스키마 검증에 걸려 메시지 전체가 버려집니다.

### 창고 맵 → RMF 내비 그래프 변환

`tools/lif2rmf` 가 VDA 5050 맵(노드/엣지 JSON)을 RMF building map(YAML)으로 바꿉니다.
두 형식을 손으로 동시에 관리하면 반드시 drift 하므로 코드로 변환합니다.

```bash
cargo run -p lif2rmf -- --map deploy/maps/plant-b.l1.json --out deploy/maps/plant-b.building.yaml
# plant-b/l1 → 정점 39(노드 35 + 문 2×2) · 차선 53 · 문 2
```

RMF 가 실제로 파싱한 결과: **내비 그래프 정점 35 · 차선 106**(53쌍 × 왕방향),
그리고 **문을 지나는 차선 8개**(DOOR-IN 4 · DOOR-OUT 4)에 `door_name` 이 붙어
공유 자원 예약이 임무에 반영됩니다.

주의할 점 두 가지:

1. **파라미터 키는 `bidirectional`** 이지 `is_bidirectional` 가 아닙니다. RMF 는
   전자를 읽고 후자는 무시해서 차선이 **일방통행**이 됩니다 — 창고 끝에 로봇이
   갇힙니다. 회귀 테스트로 고정했습니다.
2. **문 세그먼트는 통로와 수직**이어야 합니다. RMF 의 `segments_intersect` 는
   공선(collinear)을 교차로 보지 않으므로, 통로를 따라 놓은 문은 조용히
   무시되어 예약이 요청되지 않습니다.

### 인증 (권장)

게이트웨이는 토큰을 설정하면 모든 `/api/*` 와 `/ws` 에 인증을 요구합니다.
토큰을 **설정하지 않으면** 열린 상태로 뜨므로, 루프백 개발 브로커가 아닌
환경에서는 반드시 지정하세요.

```bash
cargo run -p gateway -- --api-token "$(openssl rand -hex 24)" --http-addr 0.0.0.0:8090
# 또는 API_TOKEN 환경변수
```

- API 호출: `Authorization: Bearer <token>` 헤더
- WebSocket: 브라우저는 WS 에 헤더를 보낼 수 없으므로 `?token=<token>` 쿼리 파라미터
- 대시보드: 헤더의 **인증** 배지를 클릭해 토큰을 입력하면 `localStorage` 에
  보관되어 새로고침 후에도 유지됩니다
- 대시보드 페이지(`/`) 자체는 토큰 없이도 로드됩니다 — 토큰을 물어봐야 하므로

임무 지시는 **물리 하드웨어에 대한 쓰기**이므로 "포트에 도달할 수 있다"는
이유만으로 허용하면 안 됩니다.

현재 기동 중이면 바로 열 수 있습니다:
- 관제 화면: **http://localhost:8090**
- REST API: `/api/robots`, `/api/robots/{id}`, `/api/events`, `/api/metrics`, `/api/dlq`, `/api/map`
- 실시간 스트림: WebSocket `/ws`

임무를 보내려면(예: `acme/AMR-001`에게 ST-07 픽업 → 컨베이어 A 드롭):

```bash
mosquitto_pub -h localhost -t vda5050/v3/acme/AMR-001/order -f deploy/examples/order.json
```

## 검증

```bash
cargo test                       # 단위·통합 테스트 (23개)
cargo clippy --workspace --all-targets   # 경고 0
cargo fmt --all --check
cargo run -p conformance -- verify-schemas   # 반입된 8개 스키마 검증
cargo run -p conformance -- check-message --topic order deploy/examples/order.json
```

## 디렉터리

```
crates/vda5050-types   # 표준 타입·스키마 검증·캐노니컬 모델·v2 변환
crates/vda5050-sim     # 가상 로봇 (개발·적합성·부하 시험용)
crates/gateway         # MQTT 인제스트 + REST/WS + 디지털 트윈 UI
tools/conformance      # 적합성 시험 스위트
tools/broker           # 로컬 MQTT 브로커
schemas/vda5050/3.0.0  # 공식 스키마 반입 (+ PATCHES.md)
deploy/                # sim.yaml, 맵, 브로커 설정, Docker Compose
```

## 검증된 브로커

| 브로커 | 결과 |
|---|---|
| `eclipse-mosquitto:2` (컨테이너) | ✅ 5대 동시 수렴, Last Will 오프라인 통보 정상 |
| Aedes (Node, 개발용) | ✅ 5대 동시 수렴, 임무 라운드트립 정상 |
| `rumqttd` (Rust) | ⚠️ Last Will 처리 시 패닉 버그(0.20.0) — 사용 비권장 |

## 주의 사항

- **관제는 비안전(non-safety) 계층**이다. 비상정지·충돌 방지는 로봇 온보드 안전에만 의존한다.
- 반입된 공식 스키마 3개 파일에 JSON 문법 오류(trailing comma)가 있어 로컬에서 복원했다.
  자세한 내용은 [`schemas/vda5050/3.0.0/PATCHES.md`](schemas/vda5050/3.0.0/PATCHES.md).
- VDA 5050 v3.0.0에서 `responses` 토픽은 **존/엣지 리스 응답 전용**이며, 주문 거부는
  `state.errors`(`VALIDATION_FAILURE`, `INVALID_ORDER_ACTION`)로 보고한다.
- MQTT 버전: 스펙 최소 요구는 **3.1.1**. 개발 브로커(Aedes)는 3.1.1만 지원하므로 클라이언트는
  3.1.1을 사용하며, 운영(EMQX)에서는 MQTT 5로 전환 가능하다.
- **Docker Hub 접근**: 이 머신의 OrbStack 데몬은 기본 프록시(`proxy.orb.internal:8305`)가
  Docker Hub을 차단한다. working 프록시(`192.168.49.1:8282`)로 교체하면 pull이 된다:
  ```bash
  orbctl config set network_proxy "http://192.168.49.1:8282" && orbctl restart -a
  ```
  OrbStack VM에는 **자체 DNS가 없어** 프록시 없이는 외부 레지스트리에 연결할 수 없다
  (no-proxy는 구조적으로 불가).
