# isaac_indy7 — Indy7 Isaac Sim workspace

Isaac Sim standalone 6.0.1에서 Indy7 + Robotiq 2F-140, YCB object spawn,
PINK IK, wrist camera capture를 실행하는 최소 워크스페이스다.

Repo: https://github.com/CheolMin-Yoon/isaac_indy7

> 새 세션에서 이어받을 때는 [`docs/handoff.md`](docs/handoff.md)를 먼저 확인한다.
> 자산/IK 계약은 [`docs/design/`](docs/design/)가 정본이다.

## Environment

- **필요한 건 Isaac Sim standalone 6.0.1(`~/isaacsim`)뿐이다. Isaac Lab은 필요 없다** —
  이 워크스페이스는 IsaacLab의 manager-based task/gym 등록 없이 `isaacsim.core`
  API를 직접 써서 스폰/IK/카메라를 조립한다.
- 반드시 Isaac Sim 번들 python으로 실행한다. 시스템 python에는 `isaacsim`/`omni`/`pxr`
  모듈이 없다.

```bash
cd ~/isaac_indy7
```

## Structure

```
~/isaac_indy7/
├── README.md
├── docs/
│   ├── handoff.md
│   └── design/
│       ├── indy7.md
│       └── indy7-ik.md
├── indy7.py
├── source/
│   ├── assets/
│   └── isaac_indy7/
│       ├── camera.py
│       ├── indy7.py
│       └── ycb.py
└── output/camera/
```

## Run

`indy7.py`가 로봇, YCB 물체, TCP 하위 wrist camera를 스폰한다. 목표 pose를 주면
TCP가 해당 월드 pose를 PINK differential IK로 추종하고, 목표를 주지 않으면
스폰 상태로 둔다.

```bash
~/isaacsim/python.sh indy7.py
~/isaacsim/python.sh indy7.py --target-position 0.45 0.0 0.35
~/isaacsim/python.sh indy7.py --target-position 0.45 0.5 0.35 --target-orientation 0 0 1 0
~/isaacsim/python.sh indy7.py --gripper close
~/isaacsim/python.sh indy7.py --headless --max-steps 240
```

카메라는 기본으로 `link6/d455` 아래 RealSense D455 prim을 사용하고, RGB/depth를 60스텝마다
`output/camera/`에 저장한다.

주요 옵션:

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--headless` | off | GUI 없이 실행 |
| `--max-steps` | `0` | N 스텝 후 자동 종료, 0은 무한 |
| `--target-position X Y Z` | 없음 | 지정하면 TCP가 해당 월드 좌표로 이동 |
| `--target-orientation W X Y Z` | `0 0 1 0` | 목표 TCP orientation, Isaac wxyz quaternion |
| `--gripper` | `none` | `open` 또는 `close`로 시작 시 그리퍼 명령 |

USD 자산 구조/articulation 계약: [`docs/design/indy7.md`](docs/design/indy7.md).
IK 설계 계약: [`docs/design/indy7-ik.md`](docs/design/indy7-ik.md).

## Notes

- 실행 시 `fabric::IStageReaderWriter` 버전 불일치 경고는 무시 가능.
- 시스템 `rclpy` 없음 경고는 내부 ROS2(jazzy) 자동 로드 경로라 보통 무시 가능.
- YCB가 안 불러와지면 인터넷 연결 또는 Nucleus 접근을 확인한다.
- GUI가 안 뜨면 `echo $DISPLAY` 값을 확인한다.
