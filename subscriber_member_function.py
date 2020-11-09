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
from rclpy.callback_groups import CallbackGroup
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
import threading

count = 0
class ThrottledCallbackGroup(CallbackGroup):
    """
    Throttle callbacks using a token bucket.
    Callback groups are responsible for controlling when callbacks are allowed to be executed.
    rclpy provides two groups: one which always allows a callback to be executed, and another which
    allows only one callback to be executed at a time. If neither of these are sufficient then a
    custom callback group should be used instead.
    """

    def __init__(self, node):
        super().__init__()
        self.timer = node.create_timer(0.5, self.timer_callback)
        self.bucket = 10
        self.test = 0
        self.bucket_max = 10
        self.lock = threading.Lock()


    def can_execute(self, entity):
        """
        Ask group if this entity could be executed.
        :param entity: A timer, subscriber, client, or service instance
        :rtype bool: true if a callback can be executed
        """
        global count
        #print(self.test )
        if (self.test < 1 and count > 4):
            print("Subscribtion not complete .. Configurations of subscriber and publisher may not be compatible")
        count += 1
        return self.bucket > 0

    def beginning_execution(self, entity):
        """
        Get permission from the group to execute a callback for an entity.
        :param entity: A timer, subscriber, client, or service instance
        :rtype bool: true if the executor has permission to execute it
        """
        self.test += 1
        with self.lock:
            if self.bucket > 0:
                # Take a token
                self.bucket -= 1
                return True
            # The bucket has no tokens
            return False

    def ending_execution(self, entity):
        """
        Notify group that a callback finished executing.
        :param entity: A timer, subscriber, client, or service instance
        """
        self.test += 1
        pass

    def timer_callback(self):
        """Replenish the tokens in the bucket at a steady rate."""
        with self.lock:
            # If there is room in the bucket, add a token to it.
            if self.bucket < self.bucket_max:
                self.bucket += 1

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        qos_at_start = QoSProfile()

        qos_at_start.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        #qos_at_start.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE

        qos_at_start.depth = 10
        qos_at_start.history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL
        qos_at_start.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        self.group = ThrottledCallbackGroup(self)
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            qos_profile=qos_at_start, callback_group = self.group)
        print(self.subscription)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
