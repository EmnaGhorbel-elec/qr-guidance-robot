"""Single-writer guard: stop the robot when two nodes command (or feed) the same topic.

Typical ways this happens on the robot, all of which make it drive "randomly":
  - robot.service already runs the launch at boot and a second launch is started by hand
    (two mission_node / command_node: two goals and two velocity streams alternate),
  - a PC on the same ROS_DOMAIN_ID runs the Gazebo simulation (a second /odom and /cmd_vel),
  - an old node (guidance_node, a teleop) publishes on /cmd_vel next to motor_node.

ROS 2 accepts all of this silently, so the nodes check the graph themselves.
"""


class WriterGuard:
    """Checks that at most `max_writers` publishers exist on `topic`.

    check() is cheap enough for a 1 Hz timer. `conflict` stays True while the graph is wrong.
    """

    def __init__(self, node, topic, max_writers=1):
        self.node = node
        self.topic = topic
        self.max_writers = max_writers
        self.conflict = False

    def check(self, consequence='the robot is stopped until only one is left', stop=True):
        infos = self.node.get_publishers_info_by_topic(self.topic)
        bad = len(infos) > self.max_writers
        if bad:
            names = ', '.join(sorted(f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                                     for i in infos))
            log = self.node.get_logger().error if stop else self.node.get_logger().warn
            log(f'{len(infos)} publishers on {self.topic} ({names}): {consequence}. '
                'Is robot.service running next to a manual launch, or a simulation on the '
                'same ROS_DOMAIN_ID?', throttle_duration_sec=5.0)
        elif self.conflict:
            self.node.get_logger().info(f'{self.topic}: single publisher again')
        self.conflict = bad and stop
        return self.conflict
