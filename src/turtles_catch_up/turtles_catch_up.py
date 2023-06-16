#!/usr/bin/python3
import math
from typing import Optional

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn


class TurtlesCatchUPNode:
    chasing_turtle_velocity_param_name = "~chasing_turtle_vel"

    default_chasing_turtle_velocity = 1.0
    chasing_turtle_rate_hz = 10

    def __init__(
        self,
        ros_node_name: str = "turtles_catchup_node",
        ros_service_name: str = "/spawn",
        escaping_turtle_name: str = "escaping_turtle",
        chasing_turtle_name: str = "chasing_turtle",
    ) -> None:
        self.ros_node_name = ros_node_name
        self.ros_service_name = ros_service_name

        rospy.init_node(self.ros_node_name)
        rospy.wait_for_service(self.ros_service_name)

        self.escaping_turtle_name = escaping_turtle_name
        self.chasing_turtle_name = chasing_turtle_name

        self.escaping_turtle_pose: Optional[Pose] = None
        self.chasing_turtle_pose: Optional[Pose] = None

        self.chasing_turtle_velocity_publisher: Optional[rospy.Publisher] = None
        self.chasing_turtle_rate = rospy.Rate(self.chasing_turtle_rate_hz)

        # default velocity: 1
        self.chasing_turtle_velocity: float = rospy.get_param(
            self.chasing_turtle_velocity_param_name,
            self.default_chasing_turtle_velocity,
        )
        self.chasing_turtle_spawn_args = (10.0, 10.0, 0.0, self.chasing_turtle_name)

        self.__init_ros_node()

    def __init_ros_node(self) -> None:
        rospy.ServiceProxy(self.ros_service_name, Spawn)(*self.chasing_turtle_spawn_args)

        rospy.Subscriber(f"/{self.escaping_turtle_name}/pose", Pose, self.__escaping_turtle_pose_callback)
        rospy.Subscriber(f"/{self.chasing_turtle_name}/pose", Pose, self.__chasing_turtle_pose_callback)

        self.chasing_turtle_velocity_publisher = rospy.Publisher(
            f"/{self.chasing_turtle_name}/cmd_vel",
            Twist,
            queue_size=10,
        )

    def __escaping_turtle_pose_callback(self, data: Pose) -> None:
        self.escaping_turtle_pose = data

    def __chasing_turtle_pose_callback(self, data: Pose) -> None:
        self.chasing_turtle_pose = data

    def recalculate_velocity_and_get_new_chasing_turtle_twist(self) -> Twist:
        twist = Twist()

        if not (self.escaping_turtle_pose and self.chasing_turtle_pose):
            return twist

        twist.linear.x = self.chasing_turtle_velocity

        dx = self.escaping_turtle_pose.x - self.chasing_turtle_pose.x
        dy = self.escaping_turtle_pose.y - self.chasing_turtle_pose.y

        theta = math.atan2(dy, dx) - self.chasing_turtle_pose.theta

        # range: [-pi, pi]
        if theta > math.pi:
            theta -= 2 * math.pi
        elif theta < -math.pi:
            theta += 2 * math.pi

        twist.angular.z = 2.0 * theta

        return twist

    def run(self) -> None:
        while not rospy.is_shutdown():
            self.chasing_turtle_velocity_publisher.publish(self.recalculate_velocity_and_get_new_chasing_turtle_twist())
            self.chasing_turtle_rate.sleep()


def main() -> None:
    try:
        turtles_catch_up_node = TurtlesCatchUPNode()
    except Exception as exc:
        rospy.logerr("[INIT ERROR] Error while initializing TurtlesCatchUPNode: %s", exc)
        return None

    try:
        turtles_catch_up_node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
