"""
Isaac Sim에서 Franka 로봇을 3가지 동작으로 3초마다 반복 움직이는 스크립트.

사용법:
  - Isaac Sim에서 씬을 연 상태에서 Script Editor (Window > Script Editor) 열기
  - 반드시 Play 버튼을 먼저 누른 뒤, 이 스크립트 Run
  - 로봇이 안 움직이면: Play 먼저 켜고 → 스크립트 다시 Run

Franka prim 경로가 다르면 아래 FRANKA_PRIM_PATH 를 씬에 맞게 수정하세요.
"""

import numpy as np

# --------------- 설정 (씬에 맞게 수정) ---------------
FRANKA_PRIM_PATH = "/World/Franka"
# 각 동작 유지 시간 (초). 기본 -> 이동 -> 기본 순서.
MOTION_DURATION_SEC = 2.0
# 3동작 후 대기 시간 (초).
REPEAT_INTERVAL_SEC = 3.0
# ----------------------------------------------------

from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Usd

# 물리 스텝 콜백 사용 (apply_action이 물리 스텝 시점에 호출되어야 로봇이 움직임)
try:
    from isaacsim.core.api import SimulationContext
    _HAS_SIM_CTX = True
except Exception:
    _HAS_SIM_CTX = False


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


# 콜백에서 쓸 상태 (구독이 살아 있는 동안 유지)
_state = None

# 물리 dt (콜백 인자 step_size 로 누적 시간 계산)
_PHYSICS_DT = 1.0 / 60.0


def _on_physics_step(step_size):
    """매 물리 스텝 호출. 3동작 + 대기 후 반복."""
    global _state
    if not _state or "controller" not in _state:
        return
    st = _state
    dt = step_size if step_size and step_size > 0 else _PHYSICS_DT
    st["elapsed"] += dt

    if st["phase"] == 0:  # 기본 자세
        target = st["default_pos"]
        if st["elapsed"] >= MOTION_DURATION_SEC:
            st["phase"] = 1
            st["elapsed"] = 0.0
    elif st["phase"] == 1:  # 이동 자세
        target = st["moved_pos"]
        if st["elapsed"] >= MOTION_DURATION_SEC:
            st["phase"] = 2
            st["elapsed"] = 0.0
    elif st["phase"] == 2:  # 다시 기본
        target = st["default_pos"]
        if st["elapsed"] >= MOTION_DURATION_SEC:
            st["phase"] = 3
            st["elapsed"] = 0.0
    else:  # phase 3: 대기
        target = st["default_pos"]
        if st["elapsed"] >= REPEAT_INTERVAL_SEC:
            st["cycle"] += 1
            print(f"사이클 {st['cycle']} 완료 (기본 -> 이동 -> 기본), {REPEAT_INTERVAL_SEC}초 대기 후 반복.")
            st["phase"] = 0
            st["elapsed"] = 0.0

    st["controller"].apply_action(ArticulationAction(joint_positions=target))


def main():
    global _state
    _state = {}
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

    art = Articulation(prim_path)
    art.initialize()
    controller = art.get_articulation_controller()
    if controller is None:
        print("ArticulationController를 가져올 수 없습니다.")
        return

    default_pos = np.array([0.0, -0.8, 0.0, -2.2, 0.0, 2.0, 0.8, 0.04, 0.04], dtype=np.float64)
    moved_pos = np.array([0.4, -0.6, 0.2, -2.0, 0.0, 2.0, 0.8, 0.04, 0.04], dtype=np.float64)

    _state["controller"] = controller
    _state["default_pos"] = default_pos
    _state["moved_pos"] = moved_pos
    _state["phase"] = 0
    _state["elapsed"] = 0.0
    _state["cycle"] = 0

    sim_ctx = None
    if _HAS_SIM_CTX:
        sim_ctx = SimulationContext.instance()
        if sim_ctx is None:
            try:
                sim_ctx = SimulationContext()
            except Exception:
                pass
    if sim_ctx is not None:
        if sim_ctx.physics_callback_exists("move_franka_3motion"):
            sim_ctx.remove_physics_callback("move_franka_3motion")
        sim_ctx.add_physics_callback("move_franka_3motion", _on_physics_step)
        print("Franka 3동작 반복 시작 (물리 콜백). Play 켜둔 상태에서 동작. 중지: Play 중지.")
    else:
        # SimulationContext 없으면 앱 업데이트 이벤트로 폴백 (동작 안 할 수 있음)
        import omni.kit.app
        def _on_update(e):
            dt = e.payload.get("dt", _PHYSICS_DT)
            _on_physics_step(dt)
        update_stream = omni.kit.app.get_app().get_update_event_stream()
        _state["subscription"] = update_stream.create_subscription_to_pop(_on_update, name="move_franka_3motion")
        print("Franka 3동작 (앱 업데이트 폴백). Play 먼저 켜고 실행해 보세요.")
