# Indy7 IK Design

`source/isaac_indy7/indy7.py`의 `Indy7IK`가 indy7 IK 계약의 owner다. 로봇 자산/articulation
facts는 [`indy7.md`](indy7.md)를 본다.

## Why not Lula (`KinematicsSolver`)

Isaac Sim 기본 로봇 중 일부는 번들된 Lula robot descriptor가 있어
`KinematicsSolver`를 바로 쓸 수 있다. indy7은
커스텀 authoring USD라 이 descriptor가 없다 — Lula 경로를 쓰려면 descriptor를
직접 만들어야 하는데, 대신 URDF 없이도 동작하는 PINK(QP 기반 미분 IK,
pinocchio)를 택했다.

## How it works

`isaacsim.replicator.teleop.controllers.pink_ik.PinkIKController`를 감싼다.
이 컨트롤러가 생성 시점에:

1. 살아있는 스테이지에서 articulation 서브트리를 최소 URDF로 즉석
   export한다 (`pink_urdf_export._export_urdf`, 임시 디렉토리에 씀 —
   디스크에 영구 저장되는 URDF 파일은 없다).
2. USD 조인트 이름(`joint0`)과 export된 URDF 조인트 이름
   (`indy7_link0_joint0`처럼 로봇 prim 이름이 프리픽스로 붙음) 사이의
   매핑을 자동으로 처리한다.
3. pinocchio로 URDF를 로드해 QP(`osqp`) 기반 FrameTask + PostureTask로
   IK를 푼다.

### EE 프레임 이름 모호성(gotcha)

Export된 URDF에는 같은 링크에 대해 **조인트 프레임**(`<root>_link6_tcp`,
`tcp`라는 이름의 FixedJoint prim에서 옴)과 **링크(바디) 프레임**
(`<root>_tcp`, `tcp`라는 이름의 Xform link prim에서 옴)이 둘 다 생긴다.
`ee_link_name="tcp"`처럼 접미사만 주면 어느 쪽인지 모호(ambiguous)해서
`_resolve_name_from_candidates`가 예외를 던진다. `Indy7IK.__init__`은 실제
`ee_link`(RigidPrim)의 world 경로에서 로봇 root 이름을 뽑아
`f"{robot_root_name}_{ee_link_name}"`로 정확한 이름을 만들어 넘긴다 —
"tcp"만 넘기지 말 것.

## Interface

```python
from isaac_indy7.indy7 import Indy7IK

ik = Indy7IK(indy7)  # indy7: isaacsim.core.prims.SingleArticulation
ik.go_to(position, orientation)   # orientation: wxyz (Isaac 규약). 매 스텝 호출
ik.ee_pose()                      # 현재 EE (position, orientation wxyz)
ik.ee_path                        # 현재 EE prim path. wrist camera parent로 사용
ik.reset()                        # 타깃/필터 상태 초기화
```

- `go_to()`는 항상 wxyz(Isaac 규약)를 받아 내부에서 xyzw(pinocchio 규약)로
  변환한다.
- `num_arm_dofs`(기본 6)로 팔 관절만 제어하고, 뒤에 이어지는 그리퍼 DOF는
  건드리지 않는다. 그리퍼 제어는 같은 모듈의 `Indy7Gripper`가 별도로 담당한다.

## Lula와의 결정적 차이

Lula(`KinematicsSolver.compute_inverse_kinematics`)는 **한 번의 호출로
IK를 완전히 푼다**(analytic/numeric one-shot). PINK는 QP 기반 **리액티브
(미분) IK**라 한 스텝에 목표로 완전히 수렴하지 않는다 — `go_to()`를 매
물리 스텝마다 반복 호출해서 목표로 서서히 수렴시키는 용도다. 호출
반환하는 "성공"의 의미가 다르다: Lula는 "이 포즈가 IK적으로 풀렸다",
PINK(`Indy7IK`)는
"이 스텝의 목표가 reachable 범위(거리 < 0.5m) 안에 있었다"이다.

## Verified

2026-07-12, 헤드리스 스모크 테스트: `indy7_v2.usd` 단독 스폰 후 목표
`[0.3, 0.2, 0.6]`으로 60스텝 `go_to()` 반복 호출 → EE가
`[0.0, -0.20, 1.21]`에서 `[0.70, -0.32, 0.64]`로 계속 이동(수렴 진행 중)
확인. 예외 없이 동작.
