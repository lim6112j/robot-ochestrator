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
