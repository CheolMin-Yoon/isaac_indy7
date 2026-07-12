# Indy7 Asset Design

`source/assets/indy7_v2/`가 indy7 로봇 USD 계약의 owner다. articulation/솔버 파라미터
facts는 이 문서가 정본이며, IK 쪽 계약은 [`indy7-ik.md`](indy7-ik.md)를 본다.

## Files

### `source/assets/indy7_v2/indy7_v2.usd`

indy7 6축 팔 단독 USD. Owner of:

- Articulation root: `/indy7_v2` (top-level Xform) — `PhysicsArticulationRootAPI` +
  `PhysxArticulationAPI` 적용.
- Joint chain: `root_joint`(FixedJoint, world → `link0`) → `joint0..joint5`
  (RevoluteJoint, `link0→link1→...→link6`) → `tcp`(FixedJoint, `link6→tcp`).
  `dof_names`는 `["joint0", ..., "joint5"]` 6개.
- Solver params (articulation root 속성):
  - `physxArticulation:solverPositionIterationCount = 32`
  - `physxArticulation:solverVelocityIterationCount = 4` (원래 16이었으나
    낮춤 — TGS 씬은 4 초과 시 동작이 바뀐다는 PhysX 경고 있음)
  - `drive:angular:physics:stiffness = 10000`, `damping = 100`,
    `maxForce = 100` (6개 관절 전부 동일) — ⚠️ **미해결 이슈**: 이 값은
    팔 단독 무게 기준으로만 검증됐다. 그리퍼처럼 손목에 추가 페이로드를
    붙이면 필요 토크가 이 한계를 넘어 관절이 떨릴 수 있다(관찰됨). 실제
    indy7 스펙 기준으로 축별 재조정이 필요하다.
- 레거시 `hand`/`MPLM1630` 페이로드도 이 파일 안에 남아 있다(원 authoring
  시 딸려온 제네릭 그리퍼, prismatic joint 2개). 지금은 아래
  `indy7_v2_with_robotiq_2f_140.usd` 쪽 Robotiq 결합이 실사용 경로이고,
  이 페이로드는 정리 대상이지만 아직 제거하지 않았다.
- **로봇 툴링 인식**: `IsaacRobotAPI`(`isaacsim.robot.schema`)가 articulation
  root에 별도로 붙어 있어야 Robot Assembler/Robot Inspector가 이 prim을
  로봇으로 인식한다. `PhysicsArticulationRootAPI`만으로는 물리 시뮬레이션은
  되지만 툴링에는 안 잡힌다 — 전체 근거는 research-wiki
  `AI-Sessions/wiki/harness/errors/isaacsim-errors.md` 참고.

### `source/assets/indy7_v2/indy7_v2_with_robotiq_2f_140.usd`

Robot Assembler로 만든 결합 스테이지. **`indy7_v2.usd`를 상대 payload로
참조**하므로(`@./indy7_v2.usd@`) 두 파일은 항상 같은 폴더에 같이 있어야
한다 — 하나만 옮기면 깨진다.

- `/World/indy7_v2` — 위 `indy7_v2.usd`를 payload, `IsaacRobotAPI` 등 로봇
  스키마 override 추가.
- `/World/Robotiq_2F_140_config` — Nucleus의
  `Isaac/Robots/Robotiq/2F-140/Robotiq_2F_140_config.usd`를 payload로 참조.
  자체 `PhysicsArticulationRootAPI`는 `delete apiSchemas`로 제거됨(중첩
  articulation root는 PhysX가 거부하므로, indy7 쪽 root 하나만 남긴다).
- `AssemblerFixedJoint` (`robotiq_base_link` 아래) — `body0=indy7_v2/tcp`,
  `body1=Robotiq_2F_140_config/robotiq_base_link`, local pos/rot 둘 다
  거의 0. Robot Assembler가 미리 그리퍼 전체를 world 좌표에서 `tcp`에
  맞춰 옮겨놓은 뒤 용접하기 때문에 이 오프셋이 0에 가까운 게 정상이다.
- `finger_joint`의 `drive:angular:physics:targetPosition`이 조립 당시
  그리퍼가 열려 있던 포즈(45도)로 박제돼 있던 걸 0으로 수정함(2026-07-12).
  Robot Assembler로 재조립할 때마다 이 값이 다시 박힐 수 있으니, 스폰
  직후 그리퍼가 갑자기 튀면 이 필드부터 확인한다.
- 스폰하면 이 결합 articulation의 `dof_names`는 팔 6개(`joint0..joint5`)
  뒤에 그리퍼 조인트(`finger_joint` 등, mimic 조인트 포함)가 이어진다.
  `indy7-ik.md`의 `Indy7IK(num_arm_dofs=6)`는 이 순서를 전제한다.

## General Rules

- USD를 직접 authoring/수정할 때는 `PhysicsArticulationRootAPI`(물리용)와
  `IsaacRobotAPI`(툴링용)를 별개로 취급한다. 하나만 붙이고 끝내지 않는다.
- Robot Assembler로 재조립하면 `finger_joint` 같은 조립 당시 포즈가
  drive target으로 박제될 수 있다 — 재조립 후에는 항상 이상한 초기
  target이 없는지 확인한다.
- `indy7_v2.usd`와 `indy7_v2_with_robotiq_2f_140.usd`는 항상 같은 폴더에서
  같이 이동/커밋한다.
