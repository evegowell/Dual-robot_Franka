"""
Isaac Sim에서 Franka 로봇을 한 번 움직이는 스크립트 (관절 목표치로 제어).

사용법:
  - Isaac Sim에서 씬을 연 상태에서 Script Editor (Window > Script Editor) 열기
  - 이 파일 내용을 붙여넣거나, 아래 경로로 스크립트 로드 후 Run
  - 시뮬레이션은 반드시 Play 상태로 두기 (재생 후 실행)

Franka prim 경로가 다르면 아래 FRANKA_PRIM_PATH 를 씬에 맞게 수정하세요.
"""

import numpy as np

# --------------- 설정 (씬에 맞게 수정) ---------------
# Franka articulation의 prim 경로. 씬에서 로봇 선택 후 Property 패널에서 확인 가능.
FRANKA_PRIM_PATH = "/World/Franka"
# 예: "/World/envs/env_0/panda", "/World/robots/franka" 등
# ----------------------------------------------------

# omni.isaac.core (Isaac Sim 5.0)
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Usd
import omni.usd


def find_franka_prim_path():
    """Stage에서 Franka articulation 경로를 자동으로 찾습니다."""
    stage = get_current_stage()
    if not stage:
        return None
    found = []
    for prim in stage.Traverse():
        if prim.GetTypeName() == "Xform" and "franka" in prim.GetPath().pathString.lower():
            path = prim.GetPath().pathString
            if path not in found:
                found.append(path)
    return found[0] if found else None


def main():
    stage = get_current_stage()
    if not stage:
        print("Stage가 없습니다. 씬을 연 뒤 다시 실행하세요.")
        return

    prim_path = FRANKA_PRIM_PATH
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        auto = find_franka_prim_path()
        if auto:
            prim_path = auto
            print(f"Franka 자동 감지: {prim_path}")
        else:
            print(f"Franka를 찾을 수 없습니다: {prim_path}. FRANKA_PRIM_PATH를 수정하세요.")
            return

    world = World.instance()
    if world is None:
        world = World()
    art = Articulation(prim_path)
    art.initialize()

    # Franka 기본 pose (rad). 7 arm + 2 finger
    default_pos = np.array([0.0, -0.8, 0.0, -2.2, 0.0, 2.0, 0.8, 0.04, 0.04], dtype=np.float64)
    # 살짝 움직인 pose (관절 1, 2만 변경해서 팔이 보이게 움직임)
    moved_pos = np.array([0.4, -0.6, 0.2, -2.0, 0.0, 2.0, 0.8, 0.04, 0.04], dtype=np.float64)

    controller = art.get_articulation_controller()
    if controller is None:
        print("ArticulationController를 가져올 수 없습니다.")
        return

    # 1) 기본 자세로
    controller.apply_action(ArticulationAction(joint_positions=default_pos))
    for _ in range(10):
        world.step(render=True)

    # 2) moved_pos로 이동
    controller.apply_action(ArticulationAction(joint_positions=moved_pos))
    for _ in range(120):
        world.step(render=True)

    # 3) 다시 기본 자세로
    controller.apply_action(ArticulationAction(joint_positions=default_pos))
    for _ in range(120):
        world.step(render=True)

    print("Franka 동작 1사이클 완료 (기본 -> 이동 -> 기본).")


if __name__ == "__main__":
    main()
