import py_trees
from py_trees.common import Status
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from .parameter import *
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from rclpy.node import Node

class PitchDec(py_trees.behaviour.Behaviour):
    def __init__(self, name: str,node:Node):
        super().__init__(name)
        self.node = node
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("reach_now",py_trees.common.Access.WRITE)
        self.blackboard.register_key("nav_status",py_trees.common.Access.READ)
        self.pitch_pub = self.node.create_publisher(Bool, '/nav_pitch', 10)



    def update(self):       
        msg = Bool()
        print(self.blackboard.reach_now,self.blackboard.nav_status)
        msg.data = False
        try:
            if self.blackboard.reach_now == 'attack_outpost':
                msg.data = True
            else:
                msg.data = False
        except Exception as e:
            self.node.get_logger().error(f"Error occurred while updating pitch decision: {e}")

        self.pitch_pub.publish(msg)

        return Status.SUCCESS