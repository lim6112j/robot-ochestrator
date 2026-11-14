# 로봇 관제시스템 — 통신·제어 표준화 적용 계획

- 작성일: 2026-11-12
- 범위: 제조사가 다른 물류(AMR/AGV)·청소·감시 로봇을 단일 관제 화면에서 제어하기 위한
  표준 프로토콜 기반 아키텍처 및 구현 로드맵
- 적용 표준: **VDA 5050 v3.0.0** (MQTT + JSON), **Open-RMF** (멀티플릿 조정), VDMA LIF (맵 교환)

---

---

## 0. 구현 진행 상황 (2026-11-12)

| Phase | 상태 | 검증 내용 |
|---|---|---|
| **Phase 0** 기준선 | ✅ 완료 | Cargo workspace(5크레이트) · 공식 v3.0.0 스키마 8개 반입(+문법 복원) · 로컬 MQTT 브로커 · 가상 플릿 5대 기동 · `cargo test` 13개 통과 · clippy 경고 0 |
| **Phase 1** 표준화 코어 | ✅ 완료 | 게이트웨이(MQTT 구독·스키마 검증·캐노니컬 정규화·프레즌스·DLQ·메트릭) · REST/WebSocket · 디지털 트윈 대시보드 · 임무 라운드트립 검증 완료 |
| **Phase 2** RMF 연동 | 🔵 결정 완료 | Jazzy(Docker) · Python 어댑터(2안) · Full Control — 2026-11-13 확정, 착수 준비 |
| **Phase 3~5** | ⏳ 대기 | 3D 트윈(Bevy) · 실제 제조사 온보딩 · 운영화 |

#### 교통 협상 프로브 — 협상 시작까지 증명 (2026-11-13, 갱신)

`deploy/rmf/traffic_probe.py`: 서로 다른 제조사 로봇 2명이 같은 차선을
정면으로 지나는 일정을 제출해 RMF 가 실제로 중재하는지 검증.

**결과: 협상이 시작되는 것까지 증명됨.** 스케줄러 로그에
`Require negotiation for: 0 1`, 협상 토픽에
`notice: 1`, `participants=[0, 1]`, 상태 트리에 양쪽 참여자 노드.

- 이전 턴에 "협상 미발생"으로 기록했는데, **원인 판정이 틀렸다**. 스케줄러가
  `/map` 을 구독하지 않는다는 사실은 맞지만(`ros2 topic info /map` 이
  25초 후에도 구독자 0), 그건 **설계다**. 스케줄러는 내비그래프 없이
  `(map, x, y, t)` 공간만으로 중재한다. 내비그래프가 빠져서 협상이 안 난 게
  아니다.
- **진짜 원인: `ItinerarySet` waypoint 의 `time` 이 절대 나노초 시각이다.**
  steady_clock 기준의 나노초 정밀도 절대값인데, 처음에 `0, 10, 20, 30, 40`
  을 넣었다. 이는 "먼 과거의 같은 순간" 으로 해석되어 일정이 통째로
  조용히 버려진다. `now + t*1e9` 로 고정하자마자 협상이 나왔다.
  일정이 거부돼도 아무 경고가 없다는 게 이 버그를 찾기 어렵게 만든다.
- 맵 이름은 **레벨 이름**이어야 한다. 우리 building map 은
  `levels: L1:` 이므로 `L1`. `plant-b/l1` 은 schedule 이 한 번도 본 적 없는
  이름이라 일정이 무시된다(역시 무음).
- **미완성**: 협상의 *완결*(conclusion)까지는 보지 못했다. 제안에 응답하는
  주체가 필요한데, 프로브는 참여자를 흉내내는 역할이라 응답을 보내도
  진행이 안 된다. 응답 주체는 `fleet_adapter` 다.

#### fleet adapter 세그폴트 — 원인 확정 및 수정 (2026-11-13)

**원인은 이중 언래핑이었다.** `RobotAPI.__init__` 이 스스로
`config_yaml['fleet_manager']` 를 읽는데, `fleet_adapter.py` 는 이미 언래핑된
`config_yaml['fleet_manager']` 를 넘기고 있었다. 즉
`fleet_manager.fleet_manager` 를 찾는 셈.

**세그폴트로 나타난 이유**: RMF 어댑터의 executor 스레드가 그 시점에 FastDDS
안에 있어서, 파이썬이 언와인드할 때 쯤이면 스택이 이미 사라져 있다. gdb 가
잡은 C++ 스택은 `pthread_mutex_lock` ←
`DataReaderHistory::get_first_untaken_info` ← `rclcpp::Executor::wait_for_work`
인데, 이건 **다른 스레드가 우연히 있던 자리**지 고장 난 곳이 아니다. red herring.

- 진단을 오래 끈 이유 두 가지, 둘 다 거짓 원인을 만들었다:
  1. 재현 스크립트의 `sys.path.insert(0, ...)` + `import RobotClientAPI` 조합이
     **그 자체로 세그폴트한다**(RMF 없이도). 이 사실을 모르는 동안 paho 콜백
     API 버전, `Adapter.start()` 후 `mqtt.Client()`, nudged 항등 변환,
     `get_known_robot_configuration` 의 콜론 네 가지를 원인으로 지목했고
     제대로 재현하면 전부 통과한다. **전부 틀린 진단.**
  2. 프로세스가 죽은 뒤 파이썬이 죽은 스택을 언와인드하며 **정상적으로 실행된
     줄**에서 `KeyError` 를 보고한다.
- gdb 를 이미지에 추가했다(`apt-get install -y gdb`; 베이스 이미지에도
  PyPI 에도 없다). 이게 비로소 실제 스택을 보여줬다.
- **수정 후 어댑터가 정상 기동한다**: `Finished configuring Easy Full Control
  adapter for fleet [vda5050Fleet]`, `Mirror handling new sync of 1 queries`.
  세그폴트 사라짐.
- **남은 것**: 참여자 등록이 0 명이다. 어댑터는 MQTT `state` 를 구독하고 상태를
  본 뒤에야 참여자를 등록하므로, 시뮬레이터가 같은 브로커에 발행해야 한다.
  따라서 교통 협상의 완결은 여전히 미검증.
- 교훈: 세그폴트가 파이썬 코드에서 났다면, 파이썬 스택은 믿을 수 없다.
  고장 난 쪽은 다른 스레드일 수 있다.

#### door adapter 추가 (2026-11-13)

`deploy/rmf/door_adapter`: RMF `door_states` 발행 + `door_requests` 수락.
`lif2rmf` 가 문을 이미 내비 그래프에 넣고 차선 8개에 `door_name` 을 붙였지만,
상태를 발행하는 어댑터가 없어 문은 영원히 닫혀 있어 문을 지나는 임무는
전부 정체했을 것이다.

- `MODE_OPEN` 은 "물리적으로 열림" 이 아니라 **"사용 가능"** 이고, 어댑터가
  결정한다. 영구 `MODE_CLOSED` 는 교착, 무조건 `MODE_OPEN` 은 예약을 무효화.
  요청 시 열고 hold-off(6초) 후 다시 닫는다 — RMF 예제와 같은 동작.
- 검증(실제 컨테이너): 초기 `MODE_CLOSED` → 요청 시 `MODE_OPEN` + 로그
  "opened for acme:AMR-001" → 7초 후 자동 `MODE_CLOSED` →
  존재하지 않는 문 요청은 WARN 과 함께 거부(상태 불변).
- **시뮬레이터에 문 로직 구현** (`src/door.rs` + `robot.rs`): `DoorRegistry` 가
  플릿 전체가 공유하는 문 점유를 관리하고, `Phase::WaitingForDoor` 로 대기한다.
  임무의 첫 이동 leg 도 문 체크를 거친다(`accept_order` 가 직접 leg 를 만들 때
  체크를 빠뜨리면 문 안으로 곧장 들어간다).
- **E2E 검증 완료**: 두 제조사 로봇(`acme:AMR-002`, `sweepco:SCR-001`)에게 같은
  문 경유 임무를 동시에 내리자 두 번째가 대기했다.
  ```
  waiting for door ... holder=acme:AMR-002  (robot=sweepco:SCR-001)
  door granted, resuming                    (robot=sweepco:SCR-001)
  ```
  관제 `/api/shared-resources` 도 핸드오버를 보여줬다:
  `DOOR-IN<-AMR-002` → `DOOR-IN<-AMR-002,SCR-001` → `DOOR-IN<-SCR-001`.
- **VDA 5050 에 대기 표현 필드가 없다.** 위치만 보면 문 앞에 멈춘 로봇과
  연결이 끊긴 로봇이 구분되지 않는다. `errorType` 이 확장 문자열이므로
  `WAITING_FOR_DOOR` 를 `state.errors` 에 넣어 전달하고,
  `RobotAdapter::holds()` 가 이를 읽는다.
- **함정**: `errorReferences` 는 `{referenceKey, referenceValue}` 객체 배열이라
  문자열을 넣으면 스키마 검증에 걸려 **state 메시지 전체가 버려진다**.
  증상은 게이트웨이 `messages_rejected` 증가. 테스트로 고정.
- hold-off 기본 6초는 대기 창이 0.25초뿐이라 폴링으로 놓쳤다.
  `deploy/sim-warehouse.yaml` 에 25초로 설정해 핸드오버가 보이게 했다.

#### 어댑터 경계 trait 추가 (2026-11-13)

`crates/robot-adapter`: `RobotAdapter` trait. ADR-004 가 Python 어댑터를 택한
이유가 "교체 가능한 얇은 경계" 였으므로, 그 경계를 실제로 Rust 쪽에 고정했다.

- 메서드: `manufacturer` · `robots` · `dispatch` · `instant_action` ·
  `shared_resources` · `holds` · (`observe_state`/`observe_connection` 선택)
- `AdapterError` 가 `Dispatch`(로봇이 거부 — 아무것도 돌지 않음) 와
  `Transport`(도달 불가 — 운영자가 알 수 없음) 를 구분한다. 합치면 운영자가
  필요한 정보가 사라진다.
- 구현체: `MqttRobotAdapter`(VDA 5050 over MQTT, Python 어댑터의 Rust 대응) ·
  `NullAdapter`(아무도 응답하지 않음 — 온보딩 전 플릿용)
- 게이트웨이: `--adapter-manufacturer` (반복 가능, env `ADAPTER_MANUFACTURER`).
  `GET /api/adapters`, `GET /api/shared-resources`.
- **조용히 실패한 버그 2건**:
  1. `observe_state` 를 inherent 메서드로 뒀더니 `Arc<dyn RobotAdapter>` 로
     부르면 트레잇 기본(no-op) 이 실행되어 어댑터가 로봇을 영원히 못 봤다.
     "아직 로봇이 없구나" 와 똑같이 보인다.
  2. `robots()` 가 시리얼 번호로 키를 잡지만 `state` 메시지에 제조사가 따로
     있다. 검증하지 않으면 **남의 플릿 로봇을 보고**해, 조율 계층이 자기에게
     속하지 않은 플릿을 대신 협상한다.

#### 맵 변환기 추가 (2026-11-13)

`tools/lif2rmf`: VDA 5050 맵(노드/엣지 JSON) → RMF building map(YAML).
수동 변환은 맵이 바뀔 때 drift 하므로 코드로 하고 테스트로 고정.

- 노드 → 정점, 엣지 → 차선(방향별 1개), `DOOR-*` 노드 → 문.
- RMF 파서로 실제 검증: 내비 그래프 **정점 39 · 차선 106 · 문 2**,
  문을 지나는 차선 8개에 `door_name` 부여 → 공유 자원 예약이 임무에 반영됨.
- **함정 1**: 문 세그먼트는 통로와 **수직**이어야 한다. RMF `segments_intersect`
  는 공선을 교차로 보지 않으므로 통로를 따라 놓은 문은 조용히 무시된다.
- 문 전용 정점은 어떤 차선도 참조하지 않으므로 내비 그래프에 섞이지 않는다.

#### 맵 변환기 형식 수정 — RMF 파서가 둘이라 (2026-11-13)

fleet adapter 를 실제로 띄우다가 드러난 버그. **RMF 에는 같은 building map
파일을 읽는 파서가 두 개이고, 기대하는 형식이 다르다.**

| | Python `building_map` | C++ `parse_graph.cpp` (nav graph) |
|---|---|---|
| vertex | `[x, y, theta, name, ...]` 0..3 만 읽음 | `[x, y, {name: ...}]` 3요소 |
| `speed_limit` | ParamValue `[3, 0.8]` | **bare double** |
| `door_name` | ParamValue `[1, "X"]` | **bare string** |
| `bidirectional` / `graph_idx` | 읽음 | **무시** |

기존 출력은 Python 파서용 형식이었다. Python 파서로는 검증이 통과했기 때문에
버그가 숨어 있었는데, C++ 파서는 `vertex[2]["name"]` 을 읽다가 theta 스칼라를
만나 죽었다. 에러 메시지는 **config 파일의 줄 번호와 거기 없는 키 이름**을
알려줘서 완전히 엉뚱한 곳을 보게 만든다.

수정 내용:

- vertex 를 3요소 `[x, y, {name: ...}]` 로. theta 자리는 없다.
- `speed_limit` 은 bare double, `door_name` 은 bare string.
- `graph_idx`/`bidirectional` 제거 — nav graph 파서가 안 읽는다.
- **양방향 차선이 없다.** `parse_graph.cpp` 는 항목마다
  `graph.add_lane(begin, end)` 를 한 번씩만 호출하고 bidirectional 을 읽지
  않는다. 무향쌍을 1개로 합치면 역방향에 차선이 없어 로봇이 되돌아올 수
  없으므로, 방향별로 별도 차선을 낸다(106개).
- 기존 테스트 2개가 거짓 전제(`is_bidirectional` 필드, 쌍 병합)에 서 있어서
  실제 동작에 맞게 교체했다. YAML round-trip 만으로는 이 형식 차이를 잡을 수
  없어 요소 구조를 직접 단언하는 테스트를 추가했다.

**교훈**: 한 파일을 두 파서가 읽으면, 한쪽으로 검증한 것은 다른쪽을 검증한 게
아니다. fleet adapter 까지 띄워 봐서야 드러났다.

#### 즉시 제어 추가 (2026-11-13)

임무는 시작할 수 있었지만 멈출 수 없어서, 잘못된 임무를 보내면 시뮬레이터를
죽이는 것 외에 방법이 없는 상태였다. 시뮬레이터는 `startPause`/`stopPause`/
`cancelOrder` 를 이미 처리하고 있었으나 **관제에서 보낼 창구가 없었던 것**이다.

- `POST /api/robots/{robot_id}/instant-actions` — 액션 배열을 받아
  `InstantActions` 메시지를 조립·스키마 검증 후
  `vda5050/v3/{mfr}/{serial}/instantActions` 에 발행.
- `GET /api/instant-actions` — 관제가 제공하는 액션 카탈로그.
- 대시보드: 선택 로봇 패널에 ⏸정지 / ▶재개 / ✖임무취소. 버튼 활성은 로봇
  상태(paused, orderId, 연결)에서 파생.
- `OrderPublisher::publish` 를 토픽 제네릭으로 일반화(기존에는 `Topic::Order`
  하드코딩) — 임무와 즉시액션이 같은 검증·발행 경로를 공유.
- **VDA 5050 v3 함정**: 즉시액션의 `blockingType` 은 스키마가 `"NONE"` 만
  허용한다(이동과 병렬). 임무 액션 기본값이 `HARD` 인 것과 정반대라, 순서
  액션 코드를 재사용하면 스키마 거부된다. 테스트로 고정.
- E2E 검증: 임무 발행 → 주행 중 `startPause` → `driving=false, paused=true`,
  위치 고정 → `stopPause` → 주행 재개 → `cancelOrder` → 임무 소멸, 제자리 유지.

#### 인증 추가 (2026-11-13)

`POST /api/orders` 가 추가되면서 네트워크에 도달하는 누구나 인증 없이 로봇에
임무를 내릴 수 있는 상태가 됐다. `--api-token`(env `API_TOKEN`)을 설정하면
모든 `/api/*` 와 `/ws` 가 bearer 토큰을 요구한다.

- 비교는 **상수 시간**(`constant_time_eq`) — 토큰은 공유 비밀이고 `==` 의
  타이밍 오라클로 새는 것을 막는다.
- WS 는 브라우저가 헤더를 못 보내므로 `?token=` 쿼리 파라미터로 받는다.
- 대시보드 페이지(`/`)는 토큰 없이도 로드된다(토큰을 물어봐야 하므로).
  처음엔 이 구분을 빼먹어 대시보드가 스스로 잠기는 버그가 있었음 — 테스트로 잡힘.
- 토큰 미설정 시 기동 로그에 경고를 남긴다(열린 상태임을 명시).
- 테스트 9건: 토큰 없음 401 / 정상 200 / 오토큰 401 / 비Bearer 스킴 401 /
  WS 쿼리 인증 / 정적 페이지 개방 / 임무 발행 차단.

### 웹 화면 검증 (2026-11-12, Playwright + Chrome 스크린샷)
- 디지털 트윈 대시보드 렌더 확인: 로봇 5대 지도 표시, 배터리 바, 연결 상태 점,
  플릿 분류(logistics/cleaning/surveillance), 이벤트 로그, 상세 패널(위치/배터리/모드/임무/
  액션/즉시액션/오류/안전/지원액션/존지원) 모두 정상
- 검증 중 발견·수정한 결함 3건:
  1. `ConnectionState` 직렬화가 `Online`으로 되어 대시보드가 온라인 수를 0으로 표시 → 스펙 철자(`ONLINE`)로 수정
  2. WebSocket 초기 스냅샷에 이벤트 로그가 없어 새로 연 화면이 비어 보임 → 스냅샷에 최근 30건 포함
  3. 플릿이 전부 `unknown` → `factsheet` 지원 액션에서 플릿 분류 유추(`from_actions`)
  4. **디지털 트윈이 실시간이 아님** — `Store::upsert`가 상태 저장만 하고 웹소켓으로는
     이벤트만 브로드캐스트해서, 대시보드가 페이지를 연 시점의 스냅샷만 계속 표시했다.
     (REST API는 실시간이라 API로 확인하면 멀쩡해 보이는 함정.) 상태 변경도
     `{"type":"robot",...}`로 브로드캐스트하도록 수정 — 브라우저 수신값과 서버 실제값이
     2초 간격으로 일치하며 갱신되는 것을 Playwright로 검증.
  6. 시뮬레이터 맵 그래프가 단방향 엣지만 추종해 랙 중앙 등 우회 경로를 찾지 못하고
     순간이동했음 → `MapGraph::route()` Dijkstra 최단경로 + 폴리라인 보간으로 교체.
     (`BinaryHeap` 은 최대힙이라 `Reverse` 로 감싸지 않으면 가장 **느린** 경로를 고르는 함정.)
  7. 대시보드 `baseTransform` 의 세로 오프셋 부호가 반대로 계산돼 줌 1에서도 맵이 캔버스
     위로 밀려났음 → 픽셀 측정으로 확인 후 수정. (게이트웨이 HTML 은 `include_str!` 로
     컴파일 시점에 박히므로 수정 후 **리빌드+재시작**이 필요했음.)
  8. 확대 시 팬 한도가 잘못 계산돼 화면이 빈 채로 남음 → 맵이 캔버스를 넘치는 크기만큼만
     이동 가능하도록 보정.
  5. 시뮬레이터가 주문을 받자마자 제자리에서 액션을 실행(첫 노드로 이동하지 않음),
     이동 중 위치가 갱신되지 않음(도착 시에만 점프), 이동 중 액션이 `FINISHED`로 오표시
     → 첫 노드까지 이동 구간 생성·엣지 구간 위치 선형 보간·이동 중 `WAITING`으로 수정

### end-to-end 검증 결과 (2026-11-12)
- 가상 플릿 5대(물류 2 · 청소 1 · 감시 1 · **v2 레거시 1**)가 단일 게이트웨이에 동시 수렴
- 스키마 위반 **0건** / 메시지 238건 수신, 119건 유효, 거부 0
- 임무 라운드트립: `order` publish → 시뮬레이터 수락 → `state`에 `orderId`/액션 상태 반영 →
  게이트웨이 `orderAccepted` 이벤트 기록 → UI 갱신
- `startPause`/`stopPause`/`cancelOrder` 즉시 액션 동작 확인
- **Last Will 검증**: 시뮬레이터 강제 종료 → Mosquitto가 `connection`=`CONNECTION_BROKEN` 발행 →
  게이트웨이가 5대 전부 오프라인 전환 + `robotOffline` 이벤트 기록
- 적합성 시험: 스키마 8종 파싱·컴파일·거부, 단건 메시지 검증, 고의 위반 거부(17건 사유 보고)
- 단위·통합 테스트 **23개 통과**, clippy 경고 **0**, `cargo fmt --check` 통과
- `startPause` instantAction → `paused: true`, `driving: false`, 액션 `PAUSED` 확인
- v2 레거시 로봇(`uomg/v2/...`, `agvPosition`/`batteryState`/`positionInitialized`)이
  캐노니컬 모델로 변환되어 동일 화면에 표시(`v2` 배지)

### 구현 중 확인·수정한 표준 세부사항
1. `headerId`는 **정수**(문자열 아님) — 시뮬레이터가 처음 문자열로 발행해 검증기가 전량 거부
2. `state.maps[]`에는 `mapId` + `mapVersion` + `mapStatus` 필수
3. `factsheet.protocolFeatures.mobileRobotActions[]`에는 `actionScopes`, `pauseAllowed`,
   `cancelAllowed` 필수이며 `pauseAllowed`/`cancelAllowed`는 **문자열**("true")
4. `factsheet.protocolLimits.timing`에는 `minimumOrderInterval`/`minimumStateInterval` 필수
5. `typeSpecification.mobileRobotKinematic`(단수) 필수
6. `optionalParameters[].support` enum은 `SUPPORTED`/`REQUIRED`만 허용
7. 토픽의 majorVersion 레벨은 **`v` 접두사**(예: `vda5050/v3/...`)
8. `responses` 토픽은 존/엣지 리스 전용 — **주문 수락은 `responses`가 아닌 state 관측**으로 판단
9. MQTT 최소 버전은 **3.1.1**(스펙 명시). 개발 브로커(Aedes)가 3.1.1만 지원해 클라이언트를 3.1.1로 전환
10. MQTT 클라이언트 ID는 제조사+시리얼로 고유화(두 플릿이 같은 `AMR-001`을 써도 충돌 없음)

### 개발 환경 제약(이 머신)
- Docker Hub이 프록시에서 차단 → 컨테이너 브로커 대신 **Rust 브로커(`tools/broker`)** 사용.
  단 `rumqttd` 0.20.0은 Last Will 처리 시 패닉 버그가 있어 개발 브로커는 **Aedes(Node)** 사용.
- crates.io 웹/API는 간헌 차단, **sparse index(`index.crates.io`)** 로 버전 확인.

## 1. 목표와 비목표

### 목표
1. **단일 인터페이스**: 어떤 제조사 로봇이든 동일한 토픽/메시지 규격으로 관제에 노출.
2. **단일 화면**: 모든 플릿의 위치·배터리·임무·알람을 한 지도/한 대시보드에서 관측·제어.
3. **충돌 없는 협업**: 복도·문·엘리베이터 등 공유 자원에서 다수 로봇의 동시 운행 조율.
4. **온보딩 표준화**: 신규 제조사 로봇 추가 시 "어댑터 + 적합성(conformance) 시험"만으로 끝나는 구조.

### 비목표 (명확히 제외)
- 안전 기능. 관제 시스템은 **비안전(non-safety) 계층**이다. 비상정지·충돌 방지는 로봇 온보드
  안전(레이저 스캐너, PLd 등)이 담당하며, 관제는 VDA 5050의 `pause`/`cancelOrder` 수준의
  **운영 제어**만 수행한다. (ISO/DIS 3691-4:2026 초안 진행 중 — 안전 요구사항은 별도 트랙)
- 로봇 온보드 내비게이션/Nav2 자체 개발, SLAM 개발.
- 상용 FMS(MiRFleet, OTTO Fleet Manager 등) 대체. 기존 플릿 매니저는 **그대로 두고
  VDA 5050 인터페이스로 연결**한다.

---

## 2. 전체 아키텍처

```
┌──────────────────────────────────────────────────────────────────────┐
│ L4  관제 UI (단일 화면, Rust→WASM Leptos)                            │
│     - 지도(위치/경로/존) / 임무 보드 / 배터리·알람 / 감사 로그           │
│     - 인증: OIDC + RBAC(관제원/작업자/관리자)                          │
└───────────────┬──────────────────────────────────┬───────────────────┘
                │ REST + WebSocket                 │ 내부 MQTT(게이트웨이 보유)
┌───────────────▼──────────────┐      ┌───────────▼───────────────────┐
│ L3  관제 BFF (Rust/axum)        │◄─────│ L3  표준 게이트웨이 (Rust)    │
│  - 임무 디스패치 → RMF REST   │내부  │  - state/visualization/         │
│  - instantAction 패스스루     │호출  │    connection/factSheet 구독    │
│  - 권한·감사(audit)          │      │  - JSON Schema 검증·정규화      │
└───────┬──────────────────────┘      └───────────┬───────────────────┘
        │ RMF REST (/dispatches,/tasks)          │ MQTT subscribe
┌───────▼────────────────────────────────────────▼───────────────────┐
│ L2  조정 계층: Open-RMF                                               │
│  - rmf_traffic(충돌 회피·협상) / rmf_task(임무 할당·스케줄)            │
│  - fleet adapter(플릿별) / door·lift adapter(공유 설비)                │
│  - rmf-web api-server + dashboard framework(임무 UI 기반)             │
└───────────────────────────────┬────────────────────────────────────┘
                                │ VDA 5050 v3 (MQTT publish/subscribe)
┌───────────────────────────────▼────────────────────────────────────┐
│ L1  표준 버스: MQTT 5 Broker (EMQX/HiveMQ, TLS + ACL)                 │
│  토픽: vda5050/v3/<manufacturer>/<serialNumber>/<topic>               │
└───────┬───────────────────────┬───────────────────────┬────────────┘
        │                       │                       │
┌───────▼──────┐      ┌─────────▼────────┐      ┌───────▼────────┐
│ 물류 플릿     │      │ 청소 플릿         │      │ 감시 플릿       │
│ VDA5050 직접  │      │ VDA5050 직접      │      │ VDA5050 직접    │
│ 또는         │      │ 또는             │      │ 또는            │
│ Vendor        │      │ Vendor            │      │ Vendor          │
│ Adapter      │      │ Adapter          │      │ Adapter        │
│ (REST/ROS2    │      │ (REST/ROS2       │      │ (REST/ROS2     │
│  → VDA5050)  │      │  → VDA5050)      │      │  → VDA5050)    │
└──────────────┘      └──────────────────┘      └────────────────┘
        ▲                       ▲                       ▲
   물리 로봇(온보드 안전·내비게이션은 각자 유지)
```

**핵심 원칙**: 로봇과 관제 사이의 유일한 계약은 **VDA 5050 v3.0.0**이다.
다중 로봇 *조율*은 **Open-RMF**가 담당하고, RMF는 "플릿 어댑터"를 통해 VDA 5050 플릿을
제어한다. 즉 **VDA 5050 = 운송(transport), Open-RMF = 조율(coordination)**.

> **스택 방향: Rust 단일 언어.** 관제 측 모든 신규 코드(BFF·게이트웨이·UI·시뮬레이터·어댑터)는
> Rust로 작성한다. Open-RMF 코어(rmf_traffic/rmf_task)는 C++/Python 생태계이므로 그대로
> 바이너리로 운용하고, 우리 쪽 경계만 Rust(`rclrs` ROS 2 Rust 바인딩, v0.8.0 —
> humble/jazzy/kilted/lyrical/rolling 지원)로 연결한다. 이 조합은 Open-RMF의 차세대
> 컴포넌트(`crossflow`, `mapf`, `rmf_site_map_server`, `rmf_reservation`)가 이미 Rust로
> 이동하고 있다는 점에서도 정렬된다.

---

## 3. 연동 모드 분기 (어댑터 아키텍처의 핵심)

| 모드 | 상황 | 연결 방법 | RMF 제어 수준 |
|---|---|---|---|
| **A. 플릿 매니저 보유** | 제조사가 자체 FMS를 갖고 있고 FMS가 VDA 5050를 지원 | RMF Fleet Adapter가 VDA 5050 **클라이언트**로서 FMS에 `order` publish, `state` 구독 | Full Control 가능 |
| **B. 로봇이 VDA 5050 직접 지원** | 로봇(또는 edge 게이트웨이)이 직접 토픽을 publish/subscribe | RMF Fleet Adapter가 각 로봇 토픽에 직접 연결 | Full Control 가능 |
| **C. 비표준 로봇** | ROS 2/Nav2 기반이지만 VDA 5050 미지원 | ① `free_fleet`(zenoh 브리지) 으로 RMF 연동, ② 또는 사내 Vendor Adapter가 VDA 5050로 래핑 | Full Control |
| **D. 상태만 제공** | 제조사 정책상 제어 불가 | 상태만 구독(Read Only 수준). **공유 공간에는 Read Only 플릿은 최대 1개** | Read Only |

- 모드 A/B가 정답 경로. 모드 C/D는 과도기·예외.
- Vendor Adapter는 **표준 VDA 5050 인터페이스만 노출**하도록 강제한다(래핑 대상 API가 바뀌어도
  관제 측 코드는 변하지 않음).

---

## 4. 메시지·토픽 규격 (VDA 5050 v3.0.0)

> 본 절은 공식 `json_schemas`와 `VDA5050_EN.md`에서 **직접 추출한 값**으로 정정했다.
> v2 지식(`agvPosition`, `batteryState`, `responses`=주문수락)은 v3에서 대부분 바뀌었다.

### 4.1 토픽 구조와 토픽 표
```
<interfaceName>/<majorVersion>/<manufacturer>/<serialNumber>/<topic>
예: vda5050/v3/acme/AMR-001/state
```
| topic | 발행 | 구독 | 용도 | 필수 | QoS |
|---|---|---|---|---|---|
| `order` | fleet control | mobile robot | 임무 명령 | 필수 | 0 |
| `instantActions` | fleet control | mobile robot | 즉시 액션 | 필수 | 0 |
| `state` | mobile robot | fleet control | 로봇 상태 | 필수 | 0 |
| `visualization` | mobile robot | 시각화 시스템 | 고빈도 위치·계획 경로 | 선택 | 0 |
| `connection` | **브로커(Last Will)** / mobile robot | fleet control | 연결 유실 통보 | 필수 | **1** |
| `factsheet` | mobile robot | fleet control | 능력·물리 파라미터 | 필수 | 0 |
| `zoneSet` | fleet control | mobile robot | 존 셋 전송 | 선택 | 0 |
| `responses` | fleet control | mobile robot | **존/엣지 리스 응답**(requestId, grantType) | 선택 | 0 |

**v3에서 달라진 점**: `responses`는 더 이상 주문 수락/거부를 전달하지 않는다. 주문 거부는
로봇이 `state.errors`에 `VALIDATION_FAILURE` / `INVALID_ORDER_ACTION` 등으로 보고하며,
수락은 **관측**(state에 해당 `orderId`의 nodeStates가 나타남)으로 판단한다.
→ 구현 시 "order publish → ACCEPTED 수신" 가정을 버려야 한다.

### 4.2 메시지 흐름 (임무 1건)
```
UI 임무 등록
  → BFF → RMF api-server (/dispatches)
  → rmf_task 할당 → fleet adapter
  → MQTT publish  vda5050/v3/<mfr>/<sn>/order   {headerId, orderId, orderUpdateId, nodes[], edges[]}
  → 로봇이 order를 수락하면 state에 해당 orderId의 nodeStates/edgeStates가 나타남 (수락 판단)
  → 로봇이 거부하면 state.errors 에 VALIDATION_FAILURE / INVALID_ORDER_ACTION 보고
  → MQTT publish  .../state     (1~10 Hz)  {mobileRobotPosition, powerSupply, operatingMode,
                                            actionStates, instantActionStates, paused, driving, safetyState, errors}
  → MQTT publish  .../visualization          {mobileRobotPosition, plannedPath, velocity}
  → 존/엣지 리스가 필요하면 state.zoneRequests → fleet control가 .../responses 로 GRANTED/QUEUED 회신
  → 완료: nodeStates 소진 + actionStates 전부 FINISHED/FAILED → rmf_task 완료 콜백 → BFF/UI 갱신, 이력 저장
```

### 4.3 v3.0.0 변경·주의 항목 (구현 시 필수 반영)
| 항목 | 내용 |
|---|---|
| 필드명 변경 | `header`→`headerId`, `agvPosition`→`mobileRobotPosition`, `batteryState`→`powerSupply`(`batteryCharge`→`stateOfCharge`), `eStop`→`activeEmergencyStop`, `positionInitialized`→`localized` |
| 계획 경로 공유 | `state.plannedPath` / `intermediatePath` (자유 주행 로봇) |
| Zones | `zoneSet` 토픽 + contour/kinematic-center 존 + `state.zoneRequests`/`responses` 리스 흐름 |
| operatingMode | `STARTUP`, `AUTOMATIC`, `SEMIAUTOMATIC`, `INTERVENED`, `MANUAL`, `SERVICE`, `TEACH_IN` |
| actionStatus | `WAITING`, `INITIALIZING`, `RUNNING`, `PAUSED`, `RETRIABLE`, `FINISHED`, `FAILED` |
| 액션 상태 분리 | `actionStates` / `instantActionStates` / `zoneActionStates` |
| 오류 | errorLevel `WARNING`/`URGENT`/`CRITICAL`/`FATAL`, errorType 확장 enum, 번역 배열 |
| 배터리 | `factsheet.battery` 객체, `powerSupply`에 `batteryVoltage`/`batteryCurrent`/`batteryHealth`/`range` |
| blockingType | `NONE`, `SOFT`, `SINGLE`, `HARD` |
| 안전 상태 | `safetyState.activeEmergencyStop`(`MANUAL`/`REMOTE`/`NONE`), `fieldViolation` |
| 기타 | `allowedDeviationXY` 타원화, 타임스탬프 1/1000s, NURBS, `cancelOrder`에 orderId, STARTUP 모드, idle 정의, `pauseAllowed`/`cancelAllowed`, `updateCertificate`, `startHibernation`/`stopHibernation`/shutdown, 확장 enum 전면 도입 |

### 4.4 내부 표준 모델(Canonical Model)
모든 로봇은 관제 내부에서 단일 형태로 정규화한다(버전·제조사 차이 흡수):
```jsonc
// canonical/robot.v1  (필드명은 v3.0.0 기준)
{
  "robotId": "acme:AMR-001",        // manufacturer:serialNumber
  "fleet": "logistics",             // logistics | cleaning | surveillance
  "interface": { "name": "vda5050", "majorVersion": 3, "minor": "3.0.0" },
  "source": "robot",                // robot | simulator  (디지털 트윈 배지용)
  "mapId": "plant-a/l1",
  "position": { "x": 12.4, "y": 3.1, "theta": 1.57, "mapId": "plant-a/l1", "localized": true },
  "velocity": { "vx": 0.0, "vy": 0.0, "omega": 0.0 },
  "power": { "stateOfCharge": 0.72, "charging": false, "batteryCurrent": -4.2 },
  "operatingMode": "AUTOMATIC",
  "driving": true, "paused": false,
  "loads": [],
  "actionStates": [ { "actionId": "a1", "actionType": "pick", "actionStatus": "RUNNING" } ],
  "errors": [ { "errorLevel": "WARNING", "errorType": "..." } ],
  "safetyState": { "activeEmergencyStop": "NONE", "fieldViolation": false },
  "connection": "ONLINE",           // ONLINE | OFFLINE | CONNECTION_BROKEN
  "lastSeen": "2026-11-12T10:00:00.123Z",
  "capabilities": { "actions": ["pick","drop","cleanZone","patrol"], "zones": true }
}
```
- `factSheet` 수신 시 `capabilities` 자동 갱신 → UI가 로봇별 가능 액션만 노출.
- 버전 하위호환: 내부 모델은 v3.0.0 기준. v2.x 로봇은 게이트웨이가 **업/다운 컨버전**
  (`agvPosition`→`mobileRobotPosition`, `batteryCharge`→`stateOfCharge`,
  `positionInitialized`→`localized`, `eStop`→`activeEmergencyStop`) 한다.

## 5. 기술 스택 (Rust 단일 언어)

> 조회일 2026-11-12 기준 crates.io 버전. crates.io 웹/API는 폐쇄망에서 간헐 차단되므로
> **sparse index(`https://index.crates.io`)** 로 버전을 확인하고, 현장(에어갭) 빌드를 위해
> `cargo vendor`로 의존성을 저장소에 반입한다.

| 계층 | 선택 | 근거 |
|---|---|---|
| 언어/툴체인 | **Rust stable + cargo workspace**, `clippy -D warnings`, `rustfmt`, `cargo-deny`, `cargo nextest` | 단일 언어·단일 산출물, 엄격한 린트로 장기 유지보수 |
| 메시지 버스 | **EMQX 5**(운영) / **Mosquitto**(로컬) / `rumqttd`(임베디드·테스트) | MQTT 5, TLS, 클라이언트 인증·ACL, 대규모 세션 |
| MQTT 클라이언트 | **`rumqttc` 0.25.1** (MQTT 5는 `v5` 모듈) 또는 `rumqttc-v5-next` 0.34.0 | tokio 기반 비동기, MQTT 5(세션 만료·공유 구독) 지원 |
| 직렬화/검증 | `serde`/`serde_json` + **`jsonschema` 0.61.2** | 공식 VDA 5050 스키마로 수신·발신 메시지 100% 검증 |
| 타입 생성 | **`typify` 0.10.0-alpha.3** | JSON Schema → Rust 타입 자동 생성(`build.rs`), 표준 개선 시 재생성 |
| 게이트웨이/BFF | **`axum` 0.8.9 + `tokio` 1.54**, `tracing`(구조화 로그), `utoipa`(OpenAPI) | REST + WebSocket, 백프레셔, 관측성 |
| 관제 UI / 디지털 트윈 | **`leptos` 0.8.23** (Rust→WASM) + 캔버스 렌더(2D: MapLibre GL JS / Canvas2D, 3D: **Bevy**) | 단일 화면(관제 + 디지털 트윈)을 러스트로 유지. 3D 트윈은 `rmf_site`와 동일 스택(Rust+Bevy, 웹/데스크톱 동시 타겟) |
| DB/이력 | `sqlx` 0.9 + **TimescaleDB(Postgres)** | state 시계열, KPI·재현, 컴파일 타임 SQL 검증 |
| RMF 연동 | **`rclrs` 0.8.0** (ROS 2 Rust) — ROS 2 **Jazzy LTS**(~2029) 또는 **Lyrical LTS**(~2031) | Open-RMF 바이너리(`ros-<distro>-rmf-dev`) 그대로 + Rust 어댑터 노드 |
| CLI/설정 | `clap` 4.6, `figment`/`serde` YAML | 어댑터·시뮬레이터 설정 |
| 테스트 | `proptest` 1.12(메시지 퍼징), `testcontainers` 0.28(브로커/DB 통합 시험), `criterion`(부하) | 적합성·통합·부하를 코드로 게이트화 |
| 인프라 | Docker(멀티스테이지, distroless) → K8s, Prometheus + Grafana | Rust 산출물로 이미지 경량화 |
| CI | GitHub Actions: fmt/clippy/test + **공식 JSON Schema 적합성 시험** | 제조사 온보딩 게이트 |

**왜 Rust인가**: (1) 게이트웨이가 모든 로봇 상태를 상시 구독하므로 초당 수만 메시지의
할당·검증 비용이 곧 비용 — Rust의 zero-copy/예측 가능한 지연. (2) Open-RMF 차세대
컴포넌트가 Rust로 이동 중이어서 장기적으로 같은 언어로 경계를 맞출 수 있다.
(3) `typify`로 표준 스키마에서 타입을 직접 생성해 "표준 = 타입"이 되어 버전 드리프트를
컴파일 타임에 잡는다.

---

## 6. 컴포넌트와 저장소 구조

```
robot-control-system/                  # Cargo workspace (단일 언어)
├─ PLAN.md                             # 본 문서
├─ Cargo.toml                          # workspace 멤버·공통 의존성·lint 프로파일
├─ crates/
│  ├─ vda5050-types/                   # typify 생성 타입 + 직렬화·검증 래퍼 (표준의 단일 진실원)
│  ├─ vda5050-codegen/                 # build.rs: 스키마 → Rust 타입 생성
│  ├─ gateway/                         # MQTT 구독·검증·정규화·프레즌스·DLQ (핵심)
│  ├─ bff/                             # axum REST/WS, 권한·감사, RMF 프록시
│  ├─ ui/                              # leptos WASM 단일 관제 화면
│  ├─ vda5050-sim/                     # 표준 준수 가상 로봇 (개발·적합성 시험용)
│  ├─ history/                         # TimescaleDB 적재·조회
│  └─ adapters/
│     ├─ rmf-fleet-adapter/            # rclrs 기반 RMF full_control fleet adapter (모드 A/B)
│     ├─ vendor-template/              # 비표준 로봇 → VDA 5050 래핑 템플릿 (모드 C)
│     └─ <vendor>-adapter/             # 제조사별 구현
├─ deploy/
│  ├─ docker-compose.yml               # broker, rmf, gateway, bff, ui, db
│  ├─ broker/                          # EMQX/Mosquitto 설정, ACL, 인증서 정책
│  └─ k8s/                             # 운영 매니페스트(선택)
├─ schemas/
│  ├─ vda5050/3.0.0/*.schema           # 공식 스키마 반입(코드gen 원본)
│  └─ canonical/*.json                 # 내부 표준 모델 스키마
├─ tools/
│  └─ conformance/                     # 제조사/어댑터 적합성 시험 스위트(바이너리)
└─ tests/                              # 통합·부하 시험(각 크레이트 tests/)
```

### 6.1 게이트웨이(표준화의 심장, Rust)
- 구독: `vda5050/+/+/state`, `+/+/visualization`, `+/+/connection`, `+/+/responses`, `+/+/factSheet`, `+/+/zoneSet`
- 수신 메시지 100% `jsonschema` 검증 → 실패 시 DLQ 토픽 + 메트릭(조용히 버리지 않음)
- 프레즌스: Last Will(`CONNECTION_BROKEN`) + 하트비트 타임아웃 + `lastSeen`
- 순서·중복: `orderId`/`orderUpdateId` 기반 주문 스티칭(order update), 멱등 키
- 정규화: v3 캐노니컬 모델로 변환, 버전 컨버전(v2↔v3), 좌표 맵 키 부착
- 부하: 토픽별 샤드 워커 + `tokio::sync::mpsc` 백프레셔, 배치 DB 적재

### 6.2 결정 기록(ADR) 후보

> **상태 범례**: `제안`(의도만 있음) · `확정`(결정·근거 기록됨) · `미작성`(아직 문서 없음)
> 현재 ADR 문서는 작성되지 않았으며(`docs/adr/` 비어 있음), 아래는 결정 목록과 근거 요약이다.

| ID | 결정 | 상태 |
|---|---|---|
| ADR-001 | VDA 5050 v3.0.0을 유일한 로봇 계약으로 채택 | 제안 |
| ADR-002 | 조율은 Open-RMF, 운송은 VDA 5050 (계층 분리) | 제안 |
| ADR-003 | 관제는 비안전 계층, eStop는 온보드 안전에만 의존 | 제안 |
| ADR-004 | **RMF 경계 언어: Python** (`fleet_adapter_template`) | **확정** (2026-11-13) |
| ADR-005 | 내부 캐노니컬 모델은 v3 기준, v2.x는 게이트웨이에서 컨버전 | 제안 |
| ADR-006 | UI는 leptos WASM 단일 화면, MQTT 소유권은 서버에 | 제안 |
| ADR-007 | 의존성은 `cargo vendor` 반입(폐쇄망 빌드) | 제안 |
| ADR-008 | 디지털 트윈 = 관제 캔버스, 시뮬레이터는 MQTT 데이터 소스로 분리 | 제안 |
| ADR-009 | 렌더는 2D 우선, 3D 트윈은 Bevy 별도 크레이트(웹/데스크톱 동시 타겟) | 제안 |

#### ADR-004 상세 (2026-11-13 확정)

- **결정**: RMF fleet adapter 는 **Python `fleet_adapter_template`** 기반. Rust 는
  게이트웨이·API·UI·임무 발행에만 집중한다. 통제 수준은 **Full Control**
  (기본값, 런타임 설정값으로 노출). 실행 환경은 **Docker 컨테이너**
  (Ubuntu 24.04 + ROS 2 Jazzy).
- **배경**: Rust 단일 언어 원칙("use rust")과 대립한다. rclrs 로 직접 `full_control`
  어댑터를 쓰면 언어는 하나로 유지되지만, **트래픽 협상 로직(입찰 추정·일정
  준수·재협상)을 우리가 직접 구현**해야 한다.
- **선택 근거**:
  1. 어댑터는 RMF 메시지 ↔ VDA 5050 MQTT 를 번역하는 **얇고 교체 가능한 경계
     부품**이다. 어댑터 경계는 이미 trait 로 고정돼 있어 나중에 갈아끼울 수 있다.
  2. Python 템플릿은 Open-RMF 레퍼런스 구현이라 협상 로직이 검증·문서화돼 있다.
     이 부분을 Rust 로 다시 쓰면 Phase 2 기간이 몇 배로 늘고 검증 부담도 커진다.
  3. rclrs 는 README 에 **"안정성 보장 없음(no stability guarantees)"** 이라고
     명시돼 있다. 물론 Rust 가 ROS 2 공식 클라이언트 라이브러리가 되는 과정
     (rolling 편입, 이후 릴리스 정식 탑재)이라 전망은 좋으나, **현재 시점의
     안정성은 그대로**다.
  4. Rust 의 이점(타입 안전·단일 언어)은 우리 서비스 쪽에서 얻는 것이 크고,
     어댑터는 정확성이 가장 중요한 곳이라 지루하고 검증된 길을 택한다.
- **재검토 조건**: rclrs 가 안정성 보장을 선언하고, RMF 메시지 생성이 우리 요구
  (주기 상태 발행, 경로 요청 수신)를 만족하면 Rust 어댑터로 교체를 검토한다.
- **ROS 2 디스트로: Jazzy** — RMF 저장소가 "현재 지원 디스트로: Jazzy" 라고
  명시한다. Lyrical(2026.05 출시, LTS ~2031)이 더 오래 가지만 RMF 바이너리
  패키지가 없어 전부 소스 빌드해야 하므로, 디스트로는 RMF 가용성이 결정한다.
  (Jazzy 지원 기간: ~2029.05)

### 6.3 VDA 5050 타입 크레이트(표준 드리프트 방지)
- `build.rs`가 `schemas/vda5050/<version>/*.schema`을 `typify`로 Rust 타입으로 생성
- 스키마 버전 업(`3.0.1`, `3.y`) 시 재생성 + `cargo check`로 파급 범위를 컴파일 타임에 확인
- 확장 enum(오류 타입·액션 타입)은 `#[serde(untagged)]` 알 수 없는 변형 허용 구조로 처리

---

## 7. 단계별 실행 계획

### Phase 0 — 기준선 검증 (1주)
- Cargo workspace 골격 + `cargo vendor` 반입(폐쇄망 빌드 대비), CI(fmt/clippy/test).
- 표준 자산 확보: vda5050/vda5050 **v3.0.0** 태그의 마크다운 + `json_schemas` 반입,
  `typify`로 타입 생성 파이프라인 구축.
- 브로커 기동(EMQX/Mosquitto, TLS, ACL) + `vda5050-sim`(Rust 바이너리) 가상 로봇 1대 작성
  (`factSheet`, `connection`, `state`, `visualization`, `order`/`instantActions` 처리).
- 산출물: 스키마 검증 통과하는 end-to-end 1건(order → ACCEPTED → state → FINISHED).
- 완료 기준: 공식 스키마 100% 통과, `cargo test` 1건, UI에 위치/배터리 표시.

### Phase 1 — 표준화 코어 (2~3주)
- `gateway`(Rust) 구현: 구독·검증·정규화·프레즌스·DLQ·메트릭.
- `bff`(axum) REST/WS, 인증(OIDC)·RBAC·감사 로그, `utoipa` OpenAPI.
- 임무 명령 경로: UI → BFF → RMF(`/dispatches`) → fleet adapter → MQTT `order`.
- `instantActions` 패스스루(startPause/stopPause/cancelOrder/trigger/updateCertificate) + 권한 정책.
- 맵 레지스트리 + VDMA LIF 임포트, 좌표계 정합 규칙(맵 이름·스케일·원점) 문서화.
- 완료 기준: 3종(물류/청소/감시) 가상 플릿이 한 화면에서 제어, 모든 명령 감사 기록.

### Phase 2 — RMF 조정 연동 (2~3주)

> **확정된 결정 (2026-11-13, ADR-004)**: 디스트로 **Jazzy**(Ubuntu 24.04) ·
> 경계 언어 **Python `fleet_adapter_template`** · 통제 수준 **Full Control** ·
> 실행 환경 **Docker 컨테이너**. 근거와 재검토 조건은 6.2절 ADR-004 상세 참조.
> 따라서 원래의 "경계 스파이크(의사결정 게이트)"는 **수행하지 않는다** — 이미 결정됨.

- **환경 구성**: `deploy/rmf/` 에 Ubuntu 24.04 + Jazzy 컨테이너. `rmf_core` ·
  `rmf_fleet_adapter` · `rmf_traffic_editor` 바이너리 패키지 설치 검증.
  (RMF 는 Jazzy 만 공식 지원하므로 디스트로는 가용성이 결정함.)
- **빌딩 맵 변환**: `deploy/maps/plant-b.l1.json` → RMF 내비 그래프
  (층, 차선 lane, 문/엘리베이터 워크셀). 변환기를 Rust 로 작성(`tools/lif2rmf`) —
  수동 변환은 맵이 바뀔 때마다 드리프트하므로 코드로 만든다.
- **`rmf-fleet-adapter` (full_control, Python)**: 모드 A/B 프로파일
  (직결 로봇 / 플릿 매니저) 모두 지원. RMF 메시지 ↔ VDA 5050 MQTT 번역.
  - 어댑터는 `RobotAdapter` trait 뒤에 둔다(Rust 쪽 경계는 그대로 유지).
  - 어댑터가 게이트웨이와 만나는 접점은 **REST `/dispatches` + MQTT `order`** 둘 다
    가능하도록 하되, 1차는 게이트웨이가 이미 검증된 MQTT `order` 발행을 재사용.
- **door/lift adapter**(공유 설비) 1종 이상 — plant-b 에 `DOOR-IN`/`DOOR-OUT` 이 있으므로
  이를 워크셀로 모델링하면 검증 시나리오가 그대로 나온다.
- **완료 기준**: 좁은 통로 교차 상황에서 서로 다른 제조사 로봇 2대가 RMF 교통 협상으로
  교착 없이 통과하고, 공유 자원(문) 예약이 임무에 반영된다.
- **주의**: 어댑터가 Python 이라는 점은 ADR-004 로 기록된 의도적 예외다. 나중에
  rclrs 가 안정성 보장을 선언하면 어댑터만 교체한다(경계 trait 불변).
### Phase 3 — 관제 UI + 디지털 트윈 (2~3주)
- 단일 화면(leptos WASM + 캔버스): **디지털 트윈 캔버스**(LIF 지오메트리 + 로봇 위치·자세·
  계획 경로·존·궤적) 위에 관제 패널(플릿/로봇 목록, 임무 보드, 배터리·오류, 감사 로그)을 오버레이.
- 상태 수신은 BFF WebSocket(서버가 MQTT를 소유) 방식 — 브라우저가 MQTT를 직접 갖지 않는다.
- 다중 제조사 가상 플릿 3종 동시 표시, 알람 우선순위(errorLevel CRITICAL/URGENT).
- **시뮬/실제 구분 배지**: 캐노니컬 모델의 `source: simulator|robot` 플래그로 표시(혼합 운용 시 오인 방지).
- 완료 기준: 관제원이 마우스만으로 3개 플릿에 임무를 내리고 상태 변화를 1초 내 확인,
  캔버스에서 50대 로봇을 60fps로 렌더.

### Phase 4 — 실제 제조사 온보딩 (지속, 제조사별 1~2주)
- 신규 제조사: (1) factSheet capability 확인 → (2) `vendor-template` 기반 어댑터 작성 또는
  표준 직접 지원 확인 → (3) **적합성 시험 통과** → (4) 좌표계/맵 정합 → (5) 스테이지 배포.
- 완료 기준: 물류 1, 청소 1, 감시 1 실제(또는 파일럿) 로봇이 표준 경로로만 관제에 연결.

### Phase 5 — 운영화 (2~3주)
- HA: 브로커 클러스터/브리지, gateway/BFF 다중 인스턴스(수평 확장), 상태 저장소 복제.
- 관측: `tracing` + Prometheus 메트릭(메시지 레이트, 지연, DLQ 수, 연결 상태 분포, 검증 실패율),
  구조화 로그(correlation = orderId), Grafana 대시보드.
- 보안: 로봇/어댑터별 브로커 인증 + ACL(자신의 `manufacturer/serialNumber`만 발행),
  UI OIDC, 임무 명령 감사 추적, 인증서 회전(`updateCertificate` 흐름).
- 성능: 50대 × 2Hz state 지속, UI 갱신 지연 p95 < 1s, 게이트웨이 CPU/메모리 프로파일.
- 완료 기준: 브로커 장애 주입 시 UI에 `CONNECTION_BROKEN` 표시, 복구 후 임무 재개.

---

## 8. 검증 전략 (Rust)

1. **적합성(Conformance) 시험** — 신규 로봇/어댑터의 통과 게이트. `tools/conformance` 바이너리:
   공식 JSON Schema 검증(`jsonschema` 크레이트) + 시나리오 — 연결 수립/Last Will, factSheet 수신,
   order 수락·거부, 노드/엣지/액션 상태 전이, startPause/stopPause/cancelOrder,
   orderUpdate 스티칭, 오류 레벨, 오프라인 복구. 결과를 JSON/JUnit으로 export해 CI 게이트.
2. **단위/속성 시험** — `proptest`로 VDA 5050 메시지 퍼징(필드 조합·경계값), 버전 컨버전 왕복.
3. **통합 시험** — `testcontainers`로 EMQX/Mosquitto + TimescaleDB를 띄워 게이트웨이·BFF·
   어댑터 end-to-end. UI는 Playwright로 임무 등록 → 지도 반영 확인.
4. **부하 시험** — `criterion` 또는 별도 바이너리로 N대 로봇 × 2Hz state, 메시지 레이트·DLQ 0·지연 분포.
5. **장애 주입** — 브로커 재시작, 어댑터 kill, 로봇 강제 offline.
6. **회귀** — 스키마 버전 업데이트(v3.0.1 등) 시 `typify` 재생성 + `cargo check`로 즉시 검출.

> 참고 구현(비교 기준/oracle): **AI-SPARC/tars** — VDA 5050 v3.0.0 타겟 오픈소스 플릿 매니저
> (Python/FastAPI + Mosquitto + 공식 스키마 검증 + 로봇 시뮬레이터). 우리 Rust 구현의 동작을
> 교차 검증하는 참고와 시뮬레이터 재사용 후보로만 사용(의존성으로 넣지 않음).

---

## 9. 주요 리스크와 완화

| 리스크 | 영향 | 완화 |
|---|---|---|
| 제조사별 VDA 5050 버전 상이(2.0/2.1 vs 3.0) | 메시지 불일치 | 게이트웨이 버전 컨버전 + `majorVersion`을 토픽에 명시, 다중 버전 동시 수용 |
| 좌표계/맵 불일치 | 위치가 엉뚱한 곳에 표시 | 맵 레지스트리 + LIF 단일 원천, 온보딩 시 좌표 정합 검증(마커 기반 캘리브레이션) |
| 제조사 전용 액션(pick/clean/patrol 의미 상이) | 임무 해석 불가 | 내부 액션 카탈로그 + 매핑 테이블, 미지원 액션은 UI에서 비활성화하고 명시적 오류 |
| RMF 통제 수준 제한(Read Only 최대 1플릿) | 충돌 회피 불가 | 가능한 Full Control 어댑터로 온보딩, Read Only은 예외적으로만 |
| 관제를 안전 계층으로 오해 | 사고 위험 | 문서·UI에 비안전 계층 명시, eStop는 온보드 안전에만 의존 |
| MQTT 보안(무인증 브로커) | 무단 명령 | TLS + 클라이언트 인증 + ACL + 명령 감사 로그 |
| 표준 진화(v3.y 확장: semantic reporting, LIF, 압축 포맷 등) | 재작업 | 스키마 반입 + 확장 enum 허용 + `unknown field` 정책(엄격/허용 모드) |
| **`rclrs` 성숙도**(안정성 보장 없음, API 변동) | RMF 연동 지연·재작업 | Phase 2 스파이크로 사전 검증, 실패 시 Python `fleet_adapter_template` 폴백(2안) |
| **ROS 2 디스트로 선택**(Jazzy LTS ~2029 / Lyrical LTS ~2031 / Kilted ~2026 말) | RMF 바이너리·rclrs 조합 불일치 | Jazzy를 기본 고정(rclrs 0.8 + rmf 바이너리 모두 지원), Lyrical은 검증 후 승격 |
| **`typify`가 alpha** | 코드생성 중단·품질 편차 | 생성 코드를 커밋하고 수동 검수, 코드생성 실패 시에도 빌드되는 수동 타입 경로 유지 |
| **폐쇄망/현장 네트워크** | cargo 의존성 다운로드 실패 | `cargo vendor` 반입 + 사내 레지스트리 미러, 오프라인 빌드 검증을 CI에 포함 |
| Rust WASM UI(leptos) 학습 비용 | 일정 지연 | 지도는 MapLibre GL JS 컴포넌트로 캡슐화, UI의 러스트 의존 범위를 상태·레이아웃으로 제한 |

---

## 10. 다음 단계

1. Phase 0 착수 승인(스택은 **Rust로 확정**).
2. ~~ROS 2 디스트로 확정: Jazzy(권장) vs Lyrical~~ → **Jazzy 확정** (2026-11-13, ADR-004 상세).
3. 실제 연결 대상 로봇 목록과 각각의 인터페이스(VDA 5050 버전, 자체 FMS 여부, ROS 2/Nav2 여부)
   — **Phase 4 선행 조건이며 Phase 0~3을 막지 않음**(11절 참조).
4. ~~RMF 경계 언어: rclrs(1안) vs Python 폴백(2안)~~ → **Python 확정** (2026-11-13, ADR-004).
   통제 수준 **Full Control**, 실행 환경 **Docker(Ubuntu 24.04 + Jazzy)**.
5. UI: leptos WASM 단일 화면(권장) vs `rmf-web` 대시보드 일부 활용.

---

---

## 11. 미정 사항(FMS 종류, VDA 5050 버전) 연기 전략

> 결론: **벤더 의존 사항은 후속 개발로 연기 가능, 표준 쪽 결정은 지금 확정해야 한다.**
> FMS와 로봇의 VDA 5050 버전이 미정이어도 Phase 0~3은 막히지 않는다(시뮬레이터로 검증).
> 실제 온보딩인 Phase 4만 그 결정을 선행 조건으로 가진다.

### 11.1 연기 가능 (후속 개발로 빼도 됨)
| 항목 | 왜 나중에 해도 되는가 | 대신 지금 할 일 |
|---|---|---|
| 실제 FMS 종류·전송 프로토콜 | 어댑터가 표준 인터페이스만 노출하므로 교체·추가가 core를 건드리지 않음 | `vendor-template` + 어댑터 경계 **trait(계약)** 확정 |
| 제조사 전용 액션 매핑 | 설정(데이터)이지 코드가 아님 | 액션 카탈로그 스키마 + 매핑 파일 포맷 정의 |
| 에러 코드 매핑 | 동일 | errorType 확장 enum + unknown 값 보존 정책 |
| 좌표 캘리브레이션 값 | 현장 데이터 | 캘리브레이션 절차 + 시험 코드 작성 |
| 실제 로봇 대수·레이트 프로파일 | 부하 시험을 시뮬레이터로 대체 가능 | 시뮬레이터가 레이트·대수를 파라미터화 |
| RMF 통제 수준의 최종값 | 런타임 설정값으로 설계하면 나중에 조정 가능 | 수준을 설정값으로 노출(기본 Full Control) |

### 11.2 연기 불가 (지금 확정해야 할 것)
| 항목 | 이유 | 권장 결정 |
|---|---|---|
| **VDA 5050 기준 버전** | 토픽 구조·스키마·필드명이 메이저 버전에서 깨짐(v2: `uomg/v2/...`, `positionInitialized` / v3: `vda5050/v3/...`, `localized`, zones) | **캐노니컬 = v3.0.0**, v2.x는 인그레스 전용 변환 |
| 토픽·ACL·인증 규칙 | 배포 후 변경이 비쌈(자격증명·ACL·방화벽) | `vda5050/v3/<manufacturer>/<serialNumber>/<topic>`, 로봇별 자격증명 |
| 맵·좌표계 정책 | UI·RMF 내비 그래프·캘리브레이션이 모두 의존 | VDMA LIF 단일 원천 + 맵 레지스트리 + 2D 유사변환 |
| 어댑터 경계 인터페이스 | 나중에 바꾸면 모든 어댑터를 다시 고쳐야 함 | Rust trait로 계약 고정, 메시지에 버전 필드 포함 |
| FMS 존재 여부(모드 A/B) | 어댑터 구조가 갈림 | 두 프로파일을 모두 두고 런타임 선택 |
| 관제↔RMF 경계 언어 | Phase 2 이후 전체가 의존 | Phase 2 스파이크(1안 rclrs / 2안 Python) |

### 11.3 왜 v3.0.0 기준이 안전한가
- 미정인 로봇이 **v2.x만 지원**해도: 게이트웨이에서 v2→v3 인그레스 변환(`positionInitialized`→`localized`,
  `eStop`→`activeEmergenceStop`, 1/100s→1/1000s, 토픽 `uomg/v2`→`vda5050/v3`)하면 되므로 캐노니컬 모델은 그대로 쓴다.
- 미정인 로봇이 **v3를 지원**해도: 변환 없이 통과한다.
- 이후 **v3.0.1/v3.y**가 나와도: 스키마 반입 + `typify` 재생성 + `cargo check`로 파급 범위를 컴파일 타임에 확인한다.
- 따라서 "버전 미정"은 연기 가능한 항목이지, 리스크가 아니다. 단, **캐노니컬 버전 자체는 미정으로 남겨두면 안 된다.**

### 11.4 이에 따른 일정 조정
- Phase 0~1: 실제 로봇 없이 `vda5050-sim` 3종(물류/청소/감시) 가상 플릿으로 완료.
- Phase 2: 실제 FMS 없이 `rmf_demos` + 가상 플릿으로 교통 협상·공유 자원 예약 검증.
- Phase 3: UI 완성(가상 데이터 기반).
- Phase 4: **FMS 확정 + 대상 로봇 버전 확정이 선행 조건.** 여기서 실제 어댑터 작성.

---

## 12. 시뮬레이터 설계 (`crates/vda5050-sim`)

### 12.1 역할 분리
시뮬레이터는 **"VDA 5050을 말하는 가상 로봇"** 이며, 내비게이션은 구현하지 않는다(그래프를 따라
이동하는 척만 한다). 목적은 네 가지다.

1. 프로토콜 적합성 시험(우리 게이트웨이·BFF가 표준을 올바르게 소비하는지)
2. UI 개발용 가상 데이터(실제 로봇 없이 Phase 0~3 완료)
3. 부하 시험(로봇 대수·레이트·메시지 크기)
4. **부분 지원·고장 프로파일** 재현(v2 전용, instantActions 미지원, order 거부, 접속 끊김 등)

> 교통 협상·충돌 회피 검증은 `rmf_demos`(Gazebo + Nav2)가 담당한다. 우리 시뮬레이터는
> **선(protocol) 검증**에 집중하고, RMF 쪽은 기존 데모를 쓴다. 두 축을 섞지 않는 것이 핵심.

### 12.2 구조 (3계층)
```
┌───────────────────────────────────────────────┐
│ MQTT Adapter (rumqttc v5)                      │  connect / Last Will / 구독 / 발행
│  - topic: vda5050/v3/{mfr}/{sn}/{order,state,…} │
├───────────────────────────────────────────────┤
│ Robot VM (VDA 5050 상태 머신)                   │  order 수명주기·액션·오류·배터리
│  - OrderExecutor: nodes/edges/actions 실행      │
│  - ActionRunner: 액션별 duration·파라미터 해석   │
│  - FaultInjector: 고장 주입                     │
├───────────────────────────────────────────────┤
│ World (맵·존·충전소·문)                          │  그래프 순회, 이동 시간 계산
└───────────────────────────────────────────────┘
```
- **결정론**: 가상 클럭(tick) + 시드 PRNG(`rand_chacha`). 벽시계·랜덤에 의존하지 않아
  적합성 시험이 재현 가능하다(`--seed 42`).
- **레이트 분리**: World 틱(예: 50Hz)과 MQTT 발행 주기(예: 2Hz)를 독립 설정.

### 12.3 상태 머신 (구현 체크리스트)
- 연결: connect → Last Will(`connection`=`CONNECTION_BROKEN`) 등록 → `ONLINE` 발행
- factSheet: 시작 시 1회 발행(retained), `mapId`/`floorId`, 지원 액션 목록, 배터리·적재 정보
- order 수명: order 수신 → 수락 시 state에 해당 `orderId`의 nodeStates/edgeStates가 나타남
  (거부 시 `state.errors`에 `VALIDATION_FAILURE`/`INVALID_ORDER_ACTION` 보고 — v3에서는
  `responses`가 주문 수락 통보에 쓰이지 않음) → 노드/엣지 순회(sequenceId 증가, released 처리)
  → actionStates(`WAITING`→`RUNNING`→`FINISHED`, 실패 시 `FAILED`, 재시도 가능 시 `RETRIABLE`)
- orderUpdate: `orderUpdateId` 증가 + 노드 스티칭(stitching) 지원
- instantActions: `startPause`/`stopPause`/`cancelOrder`(의무) + `trigger`/`updateCertificate` 선택
- 타임스탬프: **1/1000초**(v3), RFC3339 밀리초
- 시뮬레이터 자신도 공식 스키마 검증을 통과해야 한다(우리 게이트웨이와 같은 크레이트 사용)

### 12.4 설정 예시 (`sim.yaml`)
```yaml
broker: { host: "localhost", port: 1883, tls: false }
world: { map: "schemas/lif/plant-a.lif.json", tick_hz: 50 }
fleet:
  logistics:
    count: 3
    manufacturer: "acme"
    serial_prefix: "AMR"
    profile: "v3-full"        # v2-only | v3-partial | v3-full
    publish_hz: 2.0
    start_pose: { map: "plant-a/l1", x: 1.0, y: 1.0, theta: 0.0 }
    actions: [ "pick", "drop", "initPosition", "startPause", "stopPause", "cancelOrder" ]
    battery: { capacity_wh: 500, drain_w: 80, charge_w: 300 }
    faults: { reject_order_prob: 0.0, drop_connection_after_s: null, action_fail_prob: 0.0 }
  cleaning:  { count: 2, manufacturer: "sweepco", serial_prefix: "SCR", profile: "v3-partial", actions: [ "cleanZone" ] }
  surveillance: { count: 1, manufacturer: "watchco", serial_prefix: "PTZ", profile: "v3-full", actions: [ "patrol" ] }
```
- `profile`로 **부분 지원**을 재현한다: `v3-partial`은 `instantActions` 미구독·`zoneSet` 미발행,
  `v2-only`는 v2 토픽/필드명으로 발행(게이트웨이 컨버전 시험용).
- `faults.*`로 고장을 주입한다(확률·시점 기반, 시드 고정).

### 12.5 고장 주입 매트릭스 (적합성 시험의 재료)
| 주입 | 기대하는 관제 동작 |
|---|---|
| 브로커 연결 강제 종료 | Last Will로 `CONNECTION_BROKEN` 표시, 복구 후 `ONLINE` |
| order에 REJECTED 응답 | 임무 실패로 UI 반영 + 오류 사유 표시 |
| 액션 FAILED / RETRIABLE | 오류 패널 + 재시도 흐름 |
| state 발행 중단(하트비트 끊김) | 타임아웃 기반 오프라인 판정 |
| v2 전용 메시지 발행 | 게이트웨이 인그레스 변환 후 정상 표시 |
| 스키마 위반 메시지 발행 | DLQ 격리 + 메트릭 증가(조용히 버리지 않음) |
| 과도한 레이트(100Hz) | 게이트웨이 백프레셔·리샘플링, UI 렌더 유지 |
| 순서 역전/중복 order | `orderId`/`orderUpdateId` 기반 스티칭·멱등 처리 |

### 12.6 CLI
```bash
vda5050-sim --config sim.yaml                 # 설정 기반 기동
vda5050-sim --config sim.yaml --robots 50 --publish-hz 10   # 부하용 오버라이드
vda5050-sim --config sim.yaml --scenario conformance/basic.yaml --seed 42  # 결정론 시나리오
```
- 시나리오 파일은 "order 발송 → 기대 상태 전이"를 단정(assert)하는 스크립트로,
  적합성 시험 스위트(`tools/conformance`)가 이를 구동한다.
- 관제 UI와의 수동 연동은 그대로 두고(인터랙티브 모드), 자동 시험만 시나리오 모드를 쓴다.

### 12.7 모듈 트리 (권장)
```
crates/vda5050-sim/src/
├─ main.rs            # CLI(clap), 설정 로드, tokio 진입
├─ config.rs          # YAML → 구조체(figment/serde)
├─ world.rs           # 맵 그래프·존·충전소, 이동 시간 계산
├─ robot.rs           # Robot VM: order/action 수명주기, 배터리, 오류
├─ executor.rs        # nodes/edges/actions 순회·스티칭
├─ faults.rs          # 고장 주입(시드 PRNG)
├─ profile.rs         # v2-only / v3-partial / v3-full 메시지 변형
├─ clock.rs           # 가상 클럭(테스트 결정론)
└─ mqtt.rs            # rumqttc 연결·구독·발행·Last Will
```
- 메시지 타입은 `vda5050-types`(typify 생성)를 공유한다. 시뮬레이터가 스키마를 어기지
  못하도록 **타입 레벨에서 강제**하는 것이 핵심이다.
- v2-only 프로파일은 `vda5050-types`와 별도의 v2 타입 셋을 두되, 내부 실행 로직은 공유한다.


---

## 13. 디지털 트윈: 시뮬레이션 화면 = 관제 화면

### 13.1 결론부터
**가능하고, 권장한다.** 관제 UI는 게이트웨이를 통해 VDA 5050 상태만 소비하고, 시뮬레이터는
**동일한 토픽·동일한 스키마**로 발행한다. 따라서 UI는 데이터 출처(시뮬/실제)를 구분할 수 없고,
구분할 필요도 없다 — 이것이 표준 계약을 하나로 고정한 직접적인 이점이다.
즉 "시뮬레이션 화면"과 "관제 화면"은 **같은 화면**이고, 차이는 데이터 소스뿐이다.

```
[실제 로봇/FMS] ┐
                ├─ MQTT(vda5050/v3/…) ─ gateway ─ 캐노니컬 모델 ─ BFF(WS) ─ UI 캔버스
[vda5050-sim]  ┘        (source=robot)   (source=simulator)
```

### 13.2 구성 원칙
1. **시뮬레이터는 별도 프로세스, MQTT로만 연결.** 관제에 내장하지 않는다(내장하면 다중
   사용자·상태 불일치 문제가 생김). 대신 캐노니컬 모델에 `source` 플래그를 두어 UI가 배지로 표시.
2. **캔버스 하나 + 관제 패널 오버레이.** 디지털 트윈이 곧 관제의 메인 캔버스이고, 임무 보드·
   알람·감사는 그 위의 패널이다. 두 화면을 분리하지 않는다.
3. **2D 먼저, 3D는 별도 트랙.** Phase 3은 평면도 기반 2D(실용성·일정). 3D 트윈은
   `crates/twin3d`(Bevy)로 분리 — `rmf_site`가 같은 스택(Rust+Bevy)으로 웹(WASM+WebGL/WebGPU)과
   데스크톱을 동시에 내므로 검증된 길이다.
4. **렌더은 DOM이 아닌 캔버스/WebGL.** 50대 × 10Hz를 DOM 엘리먼트로 그리면 프레임이 무너진다.

### 13.3 디지털 트윈이 보여줄 신호 (관제 기능과 겹치는 것 포함)
| 분류 | 신호 | 출처 |
|---|---|---|
| 로봇 | 위치·자세·속도, 배터리/충전, 적재물(loads), operatingMode, 액션 진행률 | `state` |
| 계획 | 계획 경로·중간 경로(v3 planned path), 남은 노드/엣지 | `state` + `visualization` |
| 공간 | 존(진입·금지·우선권), 복도/차선, 충전소, 문·엘리베이터 상태 | `zoneSet` + door/lift adapter |
| 충돌 | 교통 예약(누가 어느 구간 점유 중), 협상 중 표시 | RMF + gateway 파생 |
| 이력 | 궤적 잔상, 히트맵, 타임슬라이더 리플레이 | Timescale |
| 품질 | 메시지 지연·연결 aging, 스키마 검증 실패/DLQ 수, 발행 레이트 | gateway 메트릭 |
| 현장 | 카메라/센서 스트림 오버레이(URL), 작업 셀 상태 | 디바이스 어댑터 |

### 13.4 트윈이 관제에 주는 추가 가치
- **What-if / 섀도 시뮬레이션**: 시뮬레이터를 섀도로 띄워 "이 임무를 넣으면 충돌·교착이 생기는가"를
  관제 화면에서 먼저 확인 → 실제 배포. 관제원이 시뮬 모드에서 임무를 검증하는 워크플로가 된다.
- **재훈련/회고**: Timescale 이력으로 사고 시점 재생(타임슬라이더).
- **온보딩 검증**: 신규 제조사 로봇을 실배포 전에 시뮬 프로파일로 먼저 관제에 붙여 캔버스에서 확인.

### 13.5 주의·제약
- 좌표계 정합이 전제다(맵 레지스트리 + LIF). 정합 없으면 트윈은 그릴 수 없다.
- 3D 에셋(glTF) 제작 비용이 별도로 든다(로봇 모델·설비 모델). `bevy_gltf_export` 참고.
- 시뮬/실제 혼합 표시 시 `source` 배지는 **필수**(오인 사고 방지).
- 트윈은 여전히 **비안전 계층**. eStop·충돌 방지는 온보드 안전에만 의존한다.
- 3D 트윈은 웹 빌드 크기·GPU 요구사항을 검증해야 한다(저사양 관제 단말 대비 2D 폴백 유지).


---

## 14. 운용 시나리오 (End-to-End, v3.0.0 기준)

### 환경 (`docker compose up`)
`emqx`(브로커) · `rmf-core`+`rmf-web api-server` · `rmf-fleet-adapter` · `gateway`(Rust) ·
`bff`(axum) · `ui`(leptos 디지털 트윈 캔버스) · `timescale` · `vda5050-sim`(가상 플릿 4대)
맵 `plant-a/l1`, 충전소 `DOCK-1`, 폭 1.2m 복도 `C1`(1대만 통행 가능)

### 09:00 시동 — 4대가 한 화면에 뜬다
각 로봇이 ①MQTT connect + **Last Will**(`connection`=`CONNECTION_BROKEN`, QoS 1) 등록 →
②`connection`=`ONLINE` → ③`factSheet`(지원 액션·물리 파라미터) → ④`state` 2Hz 발행.

```json
// publish → vda5050/v3/acme/AMR-001/state
{ "headerId": "h-000001", "timestamp": "2026-11-12T09:00:00.123Z",
  "version": "3.0.0", "manufacturer": "acme", "serialNumber": "AMR-001",
  "orderId": "", "orderUpdateId": 0, "lastNodeId": "", "lastNodeSequenceId": 0,
  "nodeStates": [], "edgeStates": [], "driving": false,
  "mobileRobotPosition": { "x": 1.0, "y": 1.0, "theta": 0.0, "mapId": "plant-a/l1", "localized": true },
  "powerSupply": { "stateOfCharge": 92.0, "charging": false },
  "operatingMode": "AUTOMATIC", "actionStates": [], "instantActionStates": [],
  "errors": [], "safetyState": { "activeEmergencyStop": "NONE", "fieldViolation": false } }
```
gateway가 스키마 검증 → 캐노니컬 모델(`robotId=acme:AMR-001`, `fleet=logistics`,
`source=simulator`) → bff WebSocket push → UI 캔버스에 4대 + 배터리 아이콘:
```
┌ 관제 화면 — plant-a/l1 ────────────────────────────────┐
│ 물류 ●AMR-001 92% 자동    ●AMR-002 88% 대기             │
│ 청소 ●SCR-001 76% 청소중(C1)                            │
│ 감시 ●PTZ-001 95% 순찰                                  │
│ 임무: 없음    알람: 없음    연결: 4/4 ONLINE             │
└────────────────────────────────────────────────────────┘
```

### 09:05 임무 1건 (정상 경로)
UI에서 "AMR-001: ST-07 픽업 → 컨베이어 A 드롭" 등록 → BFF(감사로그 `operator=kim`) →
RMF `POST /dispatches` → adapter가 `vda5050/v3/acme/AMR-001/order` publish:
```json
{ "headerId": "h-000100", "timestamp": "2026-11-12T09:05:00.100Z", "version": "3.0.0",
  "manufacturer": "acme", "serialNumber": "AMR-001",
  "orderId": "3f6c…", "orderUpdateId": 0,
  "nodes": [ {"nodeId":"ST-07","sequenceId":0,"released":true,
              "nodePosition":{"x":4.0,"y":2.0,"theta":0.0,"mapId":"plant-a/l1","localized":true},
              "actions":[{"actionType":"pick","actionId":"a1","blockingType":"HARD"}]},
             {"nodeId":"CONV-A","sequenceId":2,"released":true,
              "actions":[{"actionType":"drop","actionId":"a2","blockingType":"HARD"}]} ],
  "edges": [ {"edgeId":"e1","sequenceId":1,"released":true,"startNodeId":"ST-07","endNodeId":"CONV-A","actions":[]} ] }
```
→ 수락 판단: 다음 state에 `orderId="3f6c…"` + `nodeStates[0].released=true` 등장
→ `actionStates a1 WAITING→RUNNING→FINISHED` → 엣지 이동 → `a2 FINISHED`
→ 노드 소진 + 액션 전부 FINISHED → rmf_task 완료 콜백 → UI 임무 카드 "완료 1분 48초" + Timescale 이력.
거부였다면 `state.errors`에 `INVALID_ORDER_ACTION`(`errorReferences=["orderId"]`)이 떴을 것이다.

### 09:07 복도 충돌 → RMF가 끼어든다
SCR-001이 C1 청소 중인데 AMR-002도 C1을 지나야 함. SCR-001은 `v3-partial`(zoneSet 미지원)이라
**Traffic Light 수준**으로 제어:
```json
// publish → vda5050/v3/sweepco/SCR-001/instantActions
{ "headerId": "h-000210", "timestamp": "2026-11-12T09:07:12.000Z", "version": "3.0.0",
  "manufacturer": "sweepco", "serialNumber": "SCR-001",
  "actions": [ { "actionType": "startPause", "actionId": "i1", "blockingType": "NONE" } ] }
```
SCR-001 state: `paused: true`, `actionStates[cleanZone].actionStatus = PAUSED`.
AMR-002 통과 후 `stopPause` → 청소 재개. UI: SCR-001 카드에 "일시정지 — 교통 조율" 배지,
이벤트 로그 `C1 예약: AMR-002 (09:07:12~09:07:31)`.

### 09:12 배터리 임계 → 자동 충전 임무
AMR-001 `powerSupply.stateOfCharge: 18.0` → gateway 룰 → RMF에 충전 임무 디스패치
(노드 `DOCK-1`, action `charge`) → UI "충전 임무 자동 생성(배터리 18%)", 감사로그 `actor=system/rule`.

### 09:20 감시 로봇 오프라인 → Last Will이 알려준다
PTZ-001 네트워크 끊김 → **EMQX가 대신 발행**: `connection`=`CONNECTION_BROKEN`(QoS 1) →
gateway가 수 ms 내 오프라인 판정 → UI: 회색 + "오프라인" + 알람 + 팝업.
09:22 복귀 → `connection`=`ONLINE` → "복귀" 이벤트, 순찰 임무 자동 재개.

### 09:25 v2 전용 로봇 도착 → 연기 전략이 먹힌다
신규 `otherco AMR-009`는 **VDA 5050 v2.1만 지원**, FMS 없이 직접 연결.
토픽 `uomg/v2/otherco/AMR-009/state`에 `agvPosition`·`batteryState`·`positionInitialized`로 발행 →
gateway 인그레스 변환(`agvPosition`→`mobileRobotPosition`, `batteryCharge`→`stateOfCharge`,
`positionInitialized`→`localized`) → UI에 다른 로봇과 **동일하게** 표시, 카드에 "v2.1 (변환 수신)" 배지.
factSheet에 `pick`이 없어 UI의 픽 버튼이 자동 비활성.

### 09:30 긴급 취소
UI 취소 클릭 → BFF 권한 확인(role=관제원) → `cancelOrder` instantAction publish →
로봇이 주문 클리어, 미완료 액션 `FAILED`/`FINISHED` 정리 → UI "취소됨 (operator=kim)" + 감사로그.

### 이 시나리오가 검증하는 것
표준 계약 단일화(09:00) · 임무 라운드트립(09:05) · 버전 차이 흡수(09:25) ·
부분 지원 대응(09:07) · 교통 조율(09:07) · 자동 충전(09:12) · 고장·복구(09:20) · 감사 추적(09:05/09:30)

## 부록: 용어집

| 용어 | 의미 |
|---|---|
| **FMS (Fleet Management System)** | 여러 로봇(플릿)의 임무 할당·경로/충돌 관리·충전 스케줄을 담당하는 소프트웨어. 예: MiRFleet, OTTO Fleet Manager, BlueBotics ANT server, Kollmorgen NDC8, TARS(오픈소스). 제조사가 자체 FMS를 갖고 있으면 우리 관제는 그 **위**(모드 A)에 서고, 없으면 로봇에 **직접** 연결한다(모드 B) |
| 플릿(Fleet) | 한 FMS/한 제조사가 관리하는 로봇 그룹. 우리 시스템은 플릿 단위로 온보딩한다 |
| **VDA 5050** | 모바일 로봇 ↔ 중앙 관제 간 MQTT+JSON 메시지 규약(본 계획의 유일한 로봇 계약) |
| **Open-RMF** | 이기종 다수 플릿의 충돌 회피·임무 스케줄을 조율하는 오픈소스 프레임워크. 사실상의 상위 조율자 역할 |
| fleet adapter | RMF 코어와 특정 플릿(FMS/로봇) API를 연결하는 번역기. 수준: Full Control / Traffic Light / Read Only / No Interface |
| master control | VDA 5050에서 order를 내리는 주체(= 우리 관제/FMS 쪽) |
| **AMR / AGV** | 자유 주행 로봇 / (경로 고정형) 무인 운반차 |
| **LIF** | VDMA Layout Interchange Format — 빌딩/층/주행 경로를 교환하는 표준 포맷 |
| factSheet | 로봇이 자기 능력(지원 액션, 배터리, 물리 한계)을 알리는 VDA 5050 메시지 |
| order / state | 임무 명령(마스터→로봇) / 로봇 현재 상황(로봇→마스터) |
| zone | v3.0.0에서 도입된 공간 영향 영역(금지구역·우선권 등) |
| instantActions | order 외 즉시 명령(startPause, cancelOrder, updateCertificate 등) |
| DLQ | 스키마 검증 실패 등으로 격리한 메시지 보관 토픽 |

> 주의: 제조업 맥락의 **FMS = Flexible Manufacturing System(유연 제조 시스템)**과 다르다.
> 로봇 관제 맥락에서는 항상 **Fleet Management System**을 뜻한다.

---

## 부록: 제조사/FMS 연동 갭 체크리스트 (온보딩 시 필수 점검)

전송 프로토콜뿐 아니라 **의미(semantics) 갭**이 훨씬 많다. 아래를 신규 제조사마다 점검하고,
점검 결과는 `docs/onboarding-checklist.md`에 버전별로 기록한다.

| # | 갭 유형 | 전형적 차이 | 대응 | 검증 방법 |
|---|---|---|---|---|
| 1 | 전송 프로토콜 | FMS마다 REST / WebSocket / TCP / ROS 2 / OPC UA / VDA 5050 | `vendor-adapter`(Rust)가 표준 VDA 5050만 노출 | 적합성 시험 |
| 2 | 좌표계·맵 | 원점·스케일·yaw 부호·단위(mm/m)·맵 이름 불일치 | 맵 레지스트리 + LIF 단일 원천 + 2D 유사변환 | 마커 기반 좌표 캘리브레이션 시험 |
| 3 | 임무 모델 | mission-id 기반 vs node/edge+action 그래프 | 내부 임무 모델로 변환, 매핑 테이블 | 임무 라운드트립 시험 |
| 4 | 액션 의미 | pick/drop/dock 파라미터 명명·의미 상이 | 액션 카탈로그 + 파라미터 스키마, factSheet 검증 | 액션별 단건 시험 |
| 5 | 상태 모델 | state 세분도, battery 단위, charging 플래그 부재 | 캐노니컬 모델로 정규화, 미제공 필드는 UI에 "미제공" 표시 | 상태 필드 커버리지 리포트 |
| 6 | 식별자 | robot id 체계·namespace·대소문자 규칙 | `robotId = manufacturer:serialNumber` 규칙 강제 | 등록 시 중복 검출 |
| 7 | 에러 체계 | 에러 코드·레벨 체계 전혀 다름 | errorType 매핑 테이블 + errorLevel은 원본 보존 | 오류 주입 시험 |
| 8 | **부분 지원(가장 흔함)** | order는 되는데 instantActions/zone/factSheet 미제원 | capability negotiation + graceful degradation, 미지원 기능은 UI 비활성화 | factSheet capability 대비 UI 기능 매트릭스 |
| 9 | 시간 | timestamp 정밀도(v2 1/100s vs v3 1/1000s), 동기화 상태 | v3 기준 정규화, NTP 요구사항을 온보딩 조건으로 명시 | 시계 오차 허용 시험 |
| 10 | 레이트·QoS | state 주기 1Hz~10Hz, retained 여부 상이 | 게이트웨이에서 리샘플링/보정, QoS 정책 고정 | 부하 시험에서 레이트 편차 측정 |
| 11 | 보안 | 인증 방식·TLS 버전·인증서 형태 상이 | 브로커 단에서 통일, 어댑터가 자격증명 관리 | 인증 거부 시험 |
| 12 | 트래픽 제어 주체 | FMS가 자체 경로 계획을 쥐고 있으면 RMF full control 불가 | Traffic Light 수준으로 강등 + 문서화, Read Only는 예외 | 교차로 시나리오 시험 |
| 13 | ID 재사용·다중 FMS | 같은 serialNumber가 둘 이상 FMS에 등록 | 등록 시 충돌 검출 및 거부 | 중복 등록 시험 |

**공수 가이드**: 제조사 1곳당 어댑터 1~2주. 그중 대부분은 ②좌표 정합과 ④액션 의미 매핑에
쓰인다. 전송 변환(①)은 보통 1~2일. ⑧부분 지원은 일정에 예측 불가능성을 넣으므로
**온보딩 첫날 factSheet를 받아 capability 매트릭스를 먼저 확정**하는 것을 원칙으로 한다.
