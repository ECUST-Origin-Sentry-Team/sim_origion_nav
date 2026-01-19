
"""
Behaviour nodes used by the navigation decision tree.

This module is intentionally "boring":
- all blackboard keys are registered in __init__
- update() functions are short and structured as early-returns
- state flags are named after *why* we store them (instead of how they are implemented)

Blackboard keys (main)
----------------------
Global blackboard:
- Referee: referee_msg/Referee
- ChaseCall: rm_interfaces/IsAbleToChaseCall
- goal: dict-like {'x': float, 'y': float}
- goal_without_chase: same as goal but not overwritten by chase
- running: std_msgs/Bool  (True while nav2 task is running)
- reach_goal: bool        (True only for the tick when a goal is reached)
- dec_now: str            (current decision name)
- is_chase: bool          (True when current goal was produced by chase)

YAML namespace blackboard:
- our_color: str
- activate_chase: bool
- activate_chase_point: dict[str, float]
- blood_limit: int/float
- <points_name>: list[dict], e.g. 'home', 'mid', 'outpost', ...
"""
from __future__ import annotations

from enum import IntEnum
import random
import time
from typing import Any, Callable, Dict, List, Optional

import py_trees
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from py_trees.common import Status
from rclpy.node import Node
from rm_interfaces.msg import IsAbleToChaseSend
from std_msgs.msg import Bool, Float32
import yaml


class PatrolMode(IntEnum):
    """How Patrol chooses the next point."""
    SEQUENTIAL = 0
    RANDOM = 1
    FIXED_BY_ENEMY_HERO_POS = 2


# -----------------------------------------------------------------------------
# GetDataFromYaml
# -----------------------------------------------------------------------------
class GetDataFromYaml(py_trees.behaviour.Behaviour):
    """
    Load config from a yaml file under package `dec_tree/config/`.

    - Data are stored under blackboard namespace "yaml".
    - Runs only once; subsequent ticks return SUCCESS.
    """

    def __init__(self, name: str, yaml_name: str, node: Node):
        super().__init__(name)
        self.yaml_name = yaml_name
        self._loaded = False

        self.node = node
        self.blackboard = self.attach_blackboard_client(namespace="yaml")

        # our_color is provided by ROS2 param self_color
        self.node.declare_parameter("self_color", "red")
        self.blackboard.register_key("our_color", py_trees.common.Access.WRITE)
        self.blackboard.our_color = (
            self.node.get_parameter("self_color").get_parameter_value().string_value
        )
        self.node.get_logger().info(f"[tree] our_color: {self.blackboard.our_color}")

    def update(self) -> Status:
        if self._loaded:
            return Status.SUCCESS

        path = f"{get_package_share_directory('dec_tree')}/config/{self.yaml_name}.yaml"
        try:
            with open(path, "r", encoding="utf-8") as f:
                yaml_file = yaml.safe_load(f) or {}
        except (FileNotFoundError, yaml.YAMLError, OSError) as e:
            self.node.get_logger().error(f"[tree] Failed to read yaml: {path} ({e})")
            return Status.FAILURE

        for key, value in yaml_file.items():
            self.blackboard.register_key(str(key), py_trees.common.Access.WRITE)
            setattr(self.blackboard, str(key), value)

        self._loaded = True
        self.node.get_logger().info(f"[tree] YAML loaded: {self.yaml_name} ({path})")
        return Status.SUCCESS


# -----------------------------------------------------------------------------
# PubGoal
# -----------------------------------------------------------------------------
class PubGoal(py_trees.behaviour.Behaviour):
    """Send the current blackboard goal to nav2 (BasicNavigator.goToPose)."""

    def __init__(self, name: str, nav: BasicNavigator):
        super().__init__(name)

        self.nav = nav
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("goal", py_trees.common.Access.READ)
        self.blackboard.register_key("running", py_trees.common.Access.READ)
        self.blackboard.register_key("running", py_trees.common.Access.WRITE)

    def update(self) -> Status:
        # Original behavior: if already running, return FAILURE (so the sequence fails)
        # Keeping it to avoid behavior changes.
        if self.blackboard.running.data is True:
            return Status.FAILURE

        goal = self.blackboard.goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.orientation.w = 1.0
        goal_msg.pose.position.x = float(goal["x"])
        goal_msg.pose.position.y = float(goal["y"])

        if self.nav.goToPose(goal_msg):
            self.blackboard.running = Bool()
            self.blackboard.running.data = True

        return Status.SUCCESS


# -----------------------------------------------------------------------------
# UnpackReferee
# -----------------------------------------------------------------------------
class UnpackReferee(py_trees.behaviour.Behaviour):
    """Extract commonly used bits from Referee message into separate blackboard keys."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("Referee", py_trees.common.Access.READ)
        self.blackboard.register_key("home_occupy", py_trees.common.Access.WRITE)

    def update(self) -> Status:
        # Original logic: bit 19 represents home occupy state
        self.blackboard.home_occupy = (self.blackboard.Referee.rfid_status >> 19) & 1
        return Status.SUCCESS


# -----------------------------------------------------------------------------
# Patrol
# -----------------------------------------------------------------------------
class Patrol(py_trees.behaviour.Behaviour):
    """
    Patrol among a set of points defined in yaml, writing `goal` to blackboard.

    Parameters
    ----------
    points_name : str
        The yaml key containing a list of patrol points.
    condition_func : Callable[[Patrol], bool]
        Gate function. Return False -> behaviour FAILURE -> selector tries next decision.
    mode : PatrolMode
        How next patrol point is chosen.
    """

    # Waiting reasons (for readability, matches strings used in root condition)
    WAIT_NORMAL = "normal"
    WAIT_HOME_PHASE_12S = "home_phase_12s"

    def __init__(
        self,
        name: str,
        points_name: str,
        node: Node,
        nav: BasicNavigator,
        condition_func: Callable[["Patrol"], bool],
        random: int = 1,  # keep kw-compat: existing code passes random=2
    ):
        super().__init__(name)

        self.node = node
        self.nav = nav
        self.points_name = points_name
        self.condition_func = condition_func

        self.mode: PatrolMode = PatrolMode(random)

        # Blackboard (yaml namespace)
        self.yaml = self.attach_blackboard_client(namespace="yaml")
        self.yaml.register_key(points_name, py_trees.common.Access.READ)
        self.yaml.register_key("our_color", py_trees.common.Access.READ)
        self.yaml.register_key("blood_limit", py_trees.common.Access.WRITE)

        # Blackboard (global)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("Referee", py_trees.common.Access.READ)
        self.blackboard.register_key("home_occupy", py_trees.common.Access.READ)
        self.blackboard.register_key("reach_goal", py_trees.common.Access.READ)
        self.blackboard.register_key("running", py_trees.common.Access.READ)
        self.blackboard.register_key("goal", py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal_without_chase", py_trees.common.Access.WRITE)
        self.blackboard.register_key("dec_now", py_trees.common.Access.WRITE)
        self.blackboard.register_key("is_chase", py_trees.common.Access.WRITE)

        # Stateful fields
        self.points: List[Dict[str, Any]] = []
        self.num_points: int = 0
        self.point_now: Dict[str, Any] | None = None

        self.blackboard.dec_now = None

        # ammo tracking (used by should_go_home condition)
        self.bullet_remain_last: int = 0
        self.got_bullet: bool = False
        self.got_bullet_in_final_minute: bool = False

        # wait handling
        self.waiting_for: Optional[str] = None
        self.wait_until: float = 0.0
        self.is_in_12s_home_wait: bool = False

        # misc
        self.their_color = "red"
        self.is_game_start = False

    # ------------------------- helpers ------------------------- #
    def condition(self) -> bool:
        return bool(self.condition_func(self))

    def _cancel_nav_task(self) -> None:
        while not self.nav.isTaskComplete():
            self.nav.cancelTask()

    def _select_initial_point(self) -> Dict[str, Any]:
        if self.mode == PatrolMode.FIXED_BY_ENEMY_HERO_POS:
            idx = max(self.blackboard.Referee.enemy_hero_pos - 1, 0)
            return self.points[idx]
        return self.points[0]

    def _select_next_point(self) -> None:
        if self.num_points <= 1:
            return

        if self.mode == PatrolMode.RANDOM:
            nxt = random.choice(self.points)
            while nxt == self.point_now:
                nxt = random.choice(self.points)
            self.point_now = nxt

        elif self.mode == PatrolMode.SEQUENTIAL:
            assert self.point_now is not None
            idx = (self.points.index(self.point_now) + 1) % self.num_points
            self.point_now = self.points[idx]

        elif self.mode == PatrolMode.FIXED_BY_ENEMY_HERO_POS:
            # point is driven by enemy_hero_pos; do nothing here
            return

    def _publish_current_point(self) -> None:
        assert self.point_now is not None
        self.blackboard.goal = self.point_now
        self.blackboard.goal_without_chase = self.point_now
        self.node.get_logger().info(
            f"{self.name}: send goal x:{float(self.point_now['x']):.3f} y:{float(self.point_now['y']):.3f}"
        )

    def _start_decision(self) -> None:
        """Called when switching into this Patrol decision."""
        self.blackboard.dec_now = self.name
        self.blackboard.is_chase = False

        self.point_now = self._select_initial_point()

        # reset wait state
        self.waiting_for = None
        self.wait_until = 0.0
        self.is_in_12s_home_wait = False

        random.seed(time.time())
        self._cancel_nav_task()

    def _update_blood_limit(self) -> None:
        # Preserve original rule: outpost/mid -> 50, otherwise 150
        if self.blackboard.dec_now in ("goto_outpost", "goto_mid"):
            self.yaml.blood_limit = 50
        else:
            self.yaml.blood_limit = 150

    def _maybe_clear_costmap_on_game_start(self) -> None:
        ref = self.blackboard.Referee
        if (ref.game_progress == 4 and not self.is_game_start) or ref.stage_remain_time in (415, 410):
            self.is_game_start = True
            self.nav.clearGlobalCostmap()

    def _tick_waiting(self) -> bool:
        """
        Handle waiting state. Returns True if handled and update() should return.
        """
        if self.waiting_for is None:
            return False

        if time.time() <= self.wait_until:
            self.node.get_logger().debug(f"{self.name}: waiting for {self.waiting_for}...")
            return True

        # waiting finished
        if self.waiting_for == self.WAIT_HOME_PHASE_12S:
            self.is_in_12s_home_wait = False

        self.waiting_for = None
        self._select_next_point()
        self._publish_current_point()
        return True

    def _maybe_enter_home_phase_wait(self) -> bool:
        """
        Special case:
        - If currently in home occupy area AND next ammo wave is within 10 seconds,
          force a 12s wait to get ammo.
        """
        ref = self.blackboard.Referee
        if (ref.stage_remain_time % 60) <= 10 and (self.blackboard.home_occupy != 0) and (not self.is_in_12s_home_wait):
            self.is_in_12s_home_wait = True
            self.waiting_for = self.WAIT_HOME_PHASE_12S
            self.wait_until = time.time() + 12.0
            return True
        return False

    # ------------------------- main tick ------------------------- #
    def update(self) -> Status:
        self._maybe_clear_costmap_on_game_start()

        # Evaluate decision condition using *last tick* bullet count.
        # Then update bullet_remain_last for the next tick.
        cond_ok = self.condition()
        self.bullet_remain_last = self.blackboard.Referee.bullet_remaining_num_17mm
        if not cond_ok:
            return Status.FAILURE

        self.points = getattr(self.yaml, self.points_name)
        self.num_points = len(self.points)
        if self.num_points == 0:
            self.node.get_logger().error(f"{self.name}: yaml.{self.points_name} is empty")
            return Status.FAILURE

        # Enter / re-enter this decision
        switching_decision = (self.blackboard.dec_now != self.name) or bool(self.blackboard.is_chase)
        hero_pos_changed = (
            self.mode == PatrolMode.FIXED_BY_ENEMY_HERO_POS
            and self.point_now is not None
            and self.point_now != self.points[max(self.blackboard.Referee.enemy_hero_pos - 1, 0)]
        )
        if switching_decision or hero_pos_changed:
            self._start_decision()
            self._publish_current_point()
            self._update_blood_limit()
            return Status.SUCCESS

        # If a nav task is still running, do nothing (goal already sent)
        if self.blackboard.running.data is True:
            return Status.SUCCESS

        # Waiting state has priority once we enter it
        if self._tick_waiting():
            return Status.SUCCESS

        # Special 12s wait near ammo wave in home
        if self._maybe_enter_home_phase_wait():
            return Status.SUCCESS

        # Normal: reached goal -> start a short wait
        if self.blackboard.reach_goal:
            self.waiting_for = self.WAIT_NORMAL
            self.wait_until = time.time() + 7.0
            return Status.SUCCESS

        # Default: keep publishing current point
        self._publish_current_point()
        return Status.SUCCESS


# -----------------------------------------------------------------------------
# CheckNavState
# -----------------------------------------------------------------------------
class CheckNavState(py_trees.behaviour.Behaviour):
    """Update blackboard.running and blackboard.reach_goal according to nav2 task status."""

    def __init__(self, name: str, nav: BasicNavigator, node: Node):
        super().__init__(name)
        self.nav = nav
        self.node = node

        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("running", py_trees.common.Access.READ)
        self.blackboard.register_key("running", py_trees.common.Access.WRITE)
        self.blackboard.register_key("reach_goal", py_trees.common.Access.WRITE)

        self.blackboard.running = Bool()
        self.blackboard.running.data = False
        self.blackboard.reach_goal = False

    def update(self) -> Status:
        if not self.blackboard.running.data:
            return Status.SUCCESS

        if not self.nav.isTaskComplete():
            return Status.SUCCESS

        self.blackboard.running.data = False
        if self.nav.getResult() == TaskResult.SUCCEEDED:
            self.node.get_logger().info("nav2 success")
            self.blackboard.reach_goal = True
        else:
            self.node.get_logger().info("nav2 failed")
            self.nav.clearAllCostmaps()
            self.blackboard.reach_goal = False
        return Status.SUCCESS


# -----------------------------------------------------------------------------
# Yaw/Pitch decisions (kept behavior; only formatting changed)
# -----------------------------------------------------------------------------
class YawDec(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("yaw", py_trees.common.Access.WRITE)

        self.blackboard.yaw = Float32()
        self.blackboard.yaw.data = 0.0

    def update(self) -> Status:
        self.blackboard.yaw.data = 1.5
        return Status.SUCCESS


class PitchDec(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("dec_now", py_trees.common.Access.READ)
        self.blackboard.register_key("reach_goal", py_trees.common.Access.READ)
        self.blackboard.register_key("pitch", py_trees.common.Access.WRITE)

        self.blackboard.pitch = Bool()
        self.blackboard.pitch.data = False

    def update(self) -> Status:
        self.blackboard.pitch.data = bool(
            self.blackboard.dec_now == "goto_outpost" and self.blackboard.reach_goal
        )
        return Status.SUCCESS


class OutpostAttackDec(py_trees.behaviour.Behaviour):
    """Publish whether outpost should be attacked (currently: allow after stage_remain_time <= 360)."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("Referee", py_trees.common.Access.READ)
        self.blackboard.register_key("outpost_attack", py_trees.common.Access.WRITE)

        self.blackboard.outpost_attack = Bool()

    def update(self) -> Status:
        self.blackboard.outpost_attack.data = self.blackboard.Referee.stage_remain_time <= 360
        return Status.SUCCESS


class ReachEnemyHeroPos(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("dec_now", py_trees.common.Access.READ)
        self.blackboard.register_key("reach_goal", py_trees.common.Access.READ)
        self.blackboard.register_key("reach_enemy_hero_pos", py_trees.common.Access.WRITE)

        self.blackboard.reach_enemy_hero_pos = Bool()
        self.blackboard.reach_enemy_hero_pos.data = False

    def update(self) -> Status:
        self.blackboard.reach_enemy_hero_pos.data = bool(
            self.blackboard.dec_now == "goto_catch_hero" and self.blackboard.reach_goal
        )
        return Status.SUCCESS


# -----------------------------------------------------------------------------
# Chase integration
# -----------------------------------------------------------------------------
class CallIsAbleToChase(py_trees.behaviour.Behaviour):
    """
    If chase is activated and ready, overwrite goal with chase goal.

    - If chase isn't active or not ready -> FAILURE (selector will try next decision)
    - When active -> SUCCESS and updates goal once per second.
    """

    def __init__(self, name: str, node: Node, nav: BasicNavigator):
        super().__init__(name)
        self.node = node
        self.nav = nav

        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("ChaseCall", py_trees.common.Access.READ)
        self.blackboard.register_key("dec_now", py_trees.common.Access.READ)
        self.blackboard.register_key("goal", py_trees.common.Access.WRITE)
        self.blackboard.register_key("is_chase", py_trees.common.Access.WRITE)

        self.yaml = self.attach_blackboard_client(namespace="yaml")
        self.yaml.register_key("activate_chase", py_trees.common.Access.WRITE)
        self.yaml.register_key("activate_chase_point", py_trees.common.Access.READ)

        self.blackboard.is_chase = False
        self._next_update_time = 0.0  # 1 Hz update

    def update(self) -> Status:
        able_decisions = set(getattr(self.yaml, "activate_chase_point", {}).keys())

        if (
            (not getattr(self.yaml, "activate_chase", False))
            or (self.blackboard.dec_now not in able_decisions)
            or (self.blackboard.ChaseCall.chase_ready is False)
        ):
            self._next_update_time = 0.0
            return Status.FAILURE

        if time.time() >= self._next_update_time:
            self._next_update_time = time.time() + 1.0
            self.blackboard.goal = {
                "x": self.blackboard.ChaseCall.chase_x,
                "y": self.blackboard.ChaseCall.chase_y,
            }
            self.blackboard.is_chase = True
            while not self.nav.isTaskComplete():
                self.nav.cancelTask()

        return Status.SUCCESS


class PublishChaseGoal(py_trees.behaviour.Behaviour):
    """Publish `IsAbleToChaseSend` based on current goal_without_chase and decision."""

    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self.node = node
        self.publisher = node.create_publisher(IsAbleToChaseSend, "tree/able_chase", 10)

        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("goal_without_chase", py_trees.common.Access.READ)
        self.blackboard.register_key("dec_now", py_trees.common.Access.READ)

        self.yaml = self.attach_blackboard_client(namespace="yaml")
        self.yaml.register_key("activate_chase_point", py_trees.common.Access.READ)

    def update(self) -> Status:
        msg = IsAbleToChaseSend()
        msg.x = float(self.blackboard.goal_without_chase["x"])
        msg.y = float(self.blackboard.goal_without_chase["y"])
        msg.max_distance_from_goal = getattr(self.yaml, "activate_chase_point", {}).get(
            self.blackboard.dec_now, 0.0
        )
        self.publisher.publish(msg)
        return Status.SUCCESS