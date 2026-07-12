# Current Handoff

이 파일은 새 agent가 현재 작업 상태를 빠르게 이어받기 위한 짧은 상태표다.
장기 계약은 `docs/design/`이 정본이며, milestone이 바뀌면 이 파일만 현재형으로
갱신한다.

## 현재 기준

- Runtime: Isaac Sim standalone 6.0.1 (`~/isaacsim`)뿐이다. **Isaac Lab은
  필요 없다** — manager-based task/gym 등록 없이 `isaacsim.core` API를
  직접 조립해서 쓴다.
- 패키지명은 `isaac_indy7`로 확정(이전 `isaac_gnn`에서 리네임, 2026-07-12).
  현재 워크스페이스는 Indy7 단일 경로 — 스폰/IK/그리퍼/YCB/카메라 단계다.
- 루트 실행 파일은 `indy7.py` 하나다. 재사용 코드는 `source/isaac_indy7/`,
  USD 자산은 `source/assets/` 아래에 둔다.
- Repo: https://github.com/CheolMin-Yoon/isaac_indy7v2
- FFW(AI Worker BG2/SG2/SH5) USD들은 이 워크스페이스 소속이 아니다 —
  `~/gs_rl/source/assets/ffw/`로 옮겨졌다(Genesis 기반 워크스페이스, 2026-07-12).
  isaac_indy7는 Indy7 전용으로 유지한다.

## 완료된 기반

- **Indy7 + Robotiq 2F-140 결합**: Robot Assembler로 `tcp` ↔
  `robotiq_base_link` 결합 USD 생성, articulation 인식(`IsaacRobotAPI`)과
  솔버 파라미터(`solverVelocityIterationCount` 16→4) 이슈 해결.
  계약/자산 facts: `docs/design/indy7.md`.
- **Indy7 + YCB + IK 엔트리포인트**: 루트 `indy7.py`가 로봇/YCB 스폰,
  선택적 목표 pose 추종(`--target-position`, `--target-orientation`),
  선택적 그리퍼 명령(`--gripper open|close`)을 담당한다.
- **Indy7 wrist camera**: `source/isaac_indy7/camera.py`의 `WristCamera`가
  TCP 하위 `zed_cam` Camera prim을 만들고, RGB/depth를 `output/camera/`에
  저장한다. 현재는 별도 ZED USD asset을 참조하지 않고 Isaac Sim Camera prim을
  프로그램으로 생성한다.

## 알려진 미해결 이슈

- `indy7_v2.usd`의 관절 `maxForce=100`은 실제 indy7 스펙 기준 검증값이 아니다.
  그리퍼 페이로드가 붙으면 토크 부족으로 떨림이 나타날 수 있음(관찰됨).
- `indy7_v2.usd`에 레거시 `hand`/`MPLM1630` 페이로드가 남아 있다.
- Indy7 IK 목표 pose는 월드 좌표 기준이다. 자세/좌표계 튜닝은 GUI에서 확인하며
  잡아야 한다.
- 카메라 mount pose는 Indy7 TCP 기준 초기값이다. 실제 ZED 형상/시야와 맞추려면
  mount offset, orientation, intrinsics를 조정해야 한다.

## 먼저 읽을 정본

- 실행 명령/옵션: `README.md`
- indy7 USD/articulation 계약: `docs/design/indy7.md`
- indy7 PINK IK 계약: `docs/design/indy7-ik.md`
- Isaac Sim 툴링 관련 재발 방지 노트:
  research-wiki `AI-Sessions/wiki/harness/errors/isaacsim-errors.md`
