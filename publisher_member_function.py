# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
'''
QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE

QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST
QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL



'''
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        qos_at_start = QoSProfile()
        print("Configuring", end = ',')
        qos_at_start.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        qos_at_start.depth = 10
        qos_at_start.history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL
        qos_at_start.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        print("Configured")        
        #self.get_logger().info('Publishing: "%s"' % qos_profile_at_start)
        #print("  ", qos_at_start)
        self.publisher_ = self.create_publisher(String, 'topic', qos_profile=qos_at_start)
        print(self.publisher_)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
