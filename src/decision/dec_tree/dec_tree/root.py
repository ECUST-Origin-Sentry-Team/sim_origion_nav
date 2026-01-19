
"""
Behaviour Tree construction for navigation decision-making.

Tree Layout (high level)
------------------------
root (Sequence)
├── get_data (Parallel: SuccessOnAll)
│   ├── get_data_from_yaml (one-shot)
│   ├── referee_list (Sequence)
│   │   ├── save_referee (ToBlackboard: /Referee -> blackboard.Referee)
│   │   └── unpack_referee (extract home_occupy bit)
│   ├── check_nav_state (updates blackboard.running / reach_goal)
│   └── save_chase_call (ToBlackboard: tree/chase_goal -> blackboard.ChaseCall)
└── dec (Sequence)
    ├── dec_selector (Selector, priority order)
    │   ├── goto_home
    │   ├── goto_chase
    │   ├── goto_outpost
    │   ├── goto_last_stand
    │   ├── goto_catch_hero
    │   └── goto_mid
    ├── pitch (decide + publish /serial/nav_pitch)
    ├── enemy_hero (decide + publish reach_hero)
    ├── outpost_attack_list (decide + publish outpost_attack)
    ├── pub_chase (publish ability-to-chase message)
    └── pub_goal (send nav goal if not already running)

Notes
-----
- This file focuses on readability: all conditions are extracted as top-level functions,
  thresholds are constants, and repetitive publisher/subscriber setup is factored out.
"""
from __future__ import annotations

from typing import Any, Callable

import py_trees
import py_trees_ros
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool

from referee_msg.msg import Referee
from rm_interfaces.msg import IsAbleToChaseCall

from .tree_node import (
    CallIsAbleToChase,
    CheckNavState,
    GetDataFromYaml,
    OutpostAttackDec,
    Patrol,
    PitchDec,
    PublishChaseGoal,
    PubGoal,
    ReachEnemyHeroPos,
    UnpackReferee,
)

# -----------------------------------------------------------------------------
# Tunable thresholds (kept identical to the original logic)
# -----------------------------------------------------------------------------
HP_FULL_THRESHOLD = 400
BULLET_LOW_THRESHOLD = 75
BULLET_GAIN_THRESHOLD = 50
FINAL_MINUTE_SECONDS = 62


# -----------------------------------------------------------------------------
# Helpers: reduce boilerplate for ToBlackboard / FromBlackboard nodes
# -----------------------------------------------------------------------------
def _to_blackboard(
    *,
    name: str,
    node: Node,
    qos_profile: QoSProfile,
    topic_name: str,
    topic_type: Any,
    blackboard_variables: str,
    initialise_variables: Any,
) -> py_trees.behaviour.Behaviour:
    behaviour = py_trees_ros.subscribers.ToBlackboard(
        name=name,
        topic_name=topic_name,
        topic_type=topic_type,
        blackboard_variables=blackboard_variables,
        initialise_variables=initialise_variables,
        qos_profile=qos_profile,
    )
    behaviour.setup(node=node)
    return behaviour


def _from_blackboard(
    *,
    name: str,
    node: Node,
    qos_profile: QoSProfile,
    topic_name: str,
    topic_type: Any,
    blackboard_variable: str,
) -> py_trees.behaviour.Behaviour:
    behaviour = py_trees_ros.publishers.FromBlackboard(
        name=name,
        topic_name=topic_name,
        topic_type=topic_type,
        qos_profile=qos_profile,
        blackboard_variable=blackboard_variable,
    )
    behaviour.setup(node=node)
    return behaviour


# -----------------------------------------------------------------------------
# Decision conditions (extracted out of create_dec for readability)
# -----------------------------------------------------------------------------
def should_go_home(patrol: Patrol) -> bool:
    """
    Decide whether to stay/go to "home" (supply zone).

    Original behavior preserved:
    - Before final minute:
        - Go home if HP low OR bullets empty.
        - If already at home decision: stay until HP full AND bullets not low.
        - Special: if Patrol is in 'home_phase_12s' waiting, keep returning True.
    - Final minute:
        - Track if we have received bullets during the final minute.
        - If NOT received bullets yet:
            - Go home on HP low OR bullets empty.
            - If already at home decision: stay until HP full AND bullets not low.
        - If received bullets during final minute:
            - Go home only based on HP (low / not full when already home).
    """
    ref = patrol.blackboard.Referee
    bullets = ref.bullet_remaining_num_17mm

    hp_full = ref.remain_hp >= HP_FULL_THRESHOLD
    hp_low = ref.remain_hp < patrol.yaml.blood_limit
    bullet_low = bullets < BULLET_LOW_THRESHOLD
    bullet_empty = bullets <= 0
    in_final_minute = ref.stage_remain_time <= FINAL_MINUTE_SECONDS

    # Detect "got bullets this tick while in home occupy area"
    got_bullet_now = (bullets - patrol.bullet_remain_last > BULLET_GAIN_THRESHOLD) and (
        patrol.blackboard.home_occupy != 0
    )
    patrol.got_bullet = got_bullet_now
    if in_final_minute and got_bullet_now:
        patrol.got_bullet_in_final_minute = True

    already_decided_home = patrol.blackboard.dec_now == "goto_home"

    if in_final_minute:
        if not patrol.got_bullet_in_final_minute:
            # Still trying to get bullets before the match ends
            if hp_low or bullet_empty:
                return True
            if already_decided_home and ((not hp_full) or bullet_low):
                return True
            return False

        # Got bullets in final minute: only care about HP
        if hp_low:
            return True
        if already_decided_home and (not hp_full):
            return True
        return False

    # Pre-final-minute (normal phase)
    if hp_low or bullet_empty:
        return True
    if patrol.waiting_for == "home_phase_12s":
        return True
    if already_decided_home and ((not hp_full) or bullet_low):
        return True
    return False


def should_go_mid(_: Patrol) -> bool:
    return True


def should_go_last_stand(patrol: Patrol) -> bool:
    our_color = patrol.yaml.our_color
    return getattr(patrol.blackboard.Referee, f"{our_color}_base_hp") <= 2000


def should_go_return_fortress(patrol: Patrol) -> bool:
    return patrol.blackboard.Referee.return_fortress > 0


def should_catch_enemy_hero(patrol: Patrol) -> bool:
    return patrol.blackboard.Referee.enemy_hero_pos > 0


def should_go_outpost(patrol: Patrol) -> bool:
    # Keep original "their_color" mutation behavior.
    if patrol.yaml.our_color == "red":
        patrol.their_color = "blue"
    return (patrol.blackboard.Referee.stage_remain_time < 400) and (
        getattr(patrol.blackboard.Referee, f"{patrol.their_color}_outpost_hp") > 0
    )


# -----------------------------------------------------------------------------
# Subtree builders
# -----------------------------------------------------------------------------
def create_get_data(node: Node, qos_profile: QoSProfile, nav: BasicNavigator) -> py_trees.behaviour.Behaviour:
    """Data acquisition subtree (Parallel, SuccessOnAll)."""
    get_data = py_trees.composites.Parallel(
        name="get_data",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    get_data_from_yaml = GetDataFromYaml(
        name="get_data_from_yaml",
        yaml_name="rmuc",
        node=node,
    )

    referee_list = py_trees.composites.Sequence(name="referee_list", memory=False)
    save_referee = _to_blackboard(
        name="save_Referee",
        node=node,
        qos_profile=qos_profile,
        topic_name="/Referee",
        topic_type=Referee,
        blackboard_variables="Referee",
        initialise_variables=Referee(),
    )
    unpack_referee = UnpackReferee(name="unpack_referee")
    referee_list.add_children([save_referee, unpack_referee])

    save_chase_call = _to_blackboard(
        name="save_Chase_data",
        node=node,
        qos_profile=qos_profile,
        topic_name="tree/chase_goal",
        topic_type=IsAbleToChaseCall,
        blackboard_variables="ChaseCall",
        initialise_variables=IsAbleToChaseCall(),
    )

    check_nav_state = CheckNavState(
        name="check_nav_state",
        nav=nav,
        node=node,
    )

    get_data.add_children([get_data_from_yaml, referee_list, check_nav_state, save_chase_call])
    return get_data


def create_dec(node: Node, nav: BasicNavigator, qos_profile: QoSProfile) -> py_trees.behaviour.Behaviour:
    """Decision subtree (Sequence)."""
    dec = py_trees.composites.Sequence(name="dec", memory=False)

    # Priority decision selector
    dec_selector = py_trees.composites.Selector(name="dec_selector", memory=False)

    goto_home = Patrol(
        name="goto_home",
        points_name="home",
        node=node,
        nav=nav,
        condition_func=should_go_home,
    )
    goto_chase = CallIsAbleToChase(name="goto_chase", node=node, nav=nav)
    goto_outpost = Patrol(
        name="goto_outpost",
        points_name="outpost",
        node=node,
        nav=nav,
        condition_func=should_go_outpost,
    )
    goto_last_stand = Patrol(
        name="goto_last",
        points_name="the_last_stand",
        node=node,
        nav=nav,
        condition_func=should_go_last_stand,
    )
    goto_return_fortress = Patrol(
        name="goto_fortress",
        points_name="return_fortress",
        node=node,
        nav=nav,
        condition_func=should_go_return_fortress,
    )
    goto_catch_hero = Patrol(
        name="goto_catch_hero",
        points_name="enemy_hero",
        node=node,
        nav=nav,
        condition_func=should_catch_enemy_hero,
        random=2,
    )
    goto_mid = Patrol(
        name="goto_mid",
        points_name="mid",
        node=node,
        nav=nav,
        condition_func=should_go_mid,
    )

    # NOTE: original code defines goto_return_fortress but does not add it to selector.
    # We preserve behavior by *not* adding it by default.
    dec_selector.add_children([goto_home, goto_chase, goto_outpost, goto_last_stand, goto_catch_hero, goto_mid])

    # Pitch subtree
    pitch = py_trees.composites.Sequence(name="pitch", memory=False)
    pitch.add_children(
        [
            PitchDec(name="pitch_dec"),
            _from_blackboard(
                name="send_pitch",
                node=node,
                qos_profile=qos_profile,
                topic_name="/serial/nav_pitch",
                topic_type=Bool,
                blackboard_variable="pitch",
            ),
        ]
    )

    # Enemy hero reach subtree
    enemy_hero = py_trees.composites.Sequence(name="enemy_hero", memory=False)
    enemy_hero.add_children(
        [
            ReachEnemyHeroPos(name="reach_enemy_hero_dec"),
            _from_blackboard(
                name="send_reach_hero",
                node=node,
                qos_profile=qos_profile,
                topic_name="reach_hero",
                topic_type=Bool,
                blackboard_variable="reach_enemy_hero_pos",
            ),
        ]
    )

    # Outpost attack subtree
    outpost_attack_list = py_trees.composites.Sequence(name="outpost_attack_list", memory=False)
    outpost_attack_list.add_children(
        [
            OutpostAttackDec(name="outpost_attack_dec"),
            _from_blackboard(
                name="send_outpost_attack",
                node=node,
                qos_profile=qos_profile,
                topic_name="outpost_attack",
                topic_type=Bool,
                blackboard_variable="outpost_attack",
            ),
        ]
    )

    pub_chase = PublishChaseGoal(name="pub_chase", node=node)
    pub_goal = PubGoal(name="pub_goal", nav=nav)

    dec.add_children([dec_selector, pitch, enemy_hero, outpost_attack_list, pub_chase, pub_goal])
    return dec


def create_tree(node: Node) -> py_trees.behaviour.Behaviour:
    qos_profile = QoSProfile(depth=10)
    nav = BasicNavigator()

    root = py_trees.composites.Sequence(name="root", memory=False)
    root.add_children([create_get_data(node, qos_profile, nav), create_dec(node, nav, qos_profile)])
    return root


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("tree_node")

    period_ms = 100
    root = create_tree(node)
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node=node)
    tree.tick_tock(period_ms=period_ms)

    rclpy.spin(node)
    rclpy.shutdown()