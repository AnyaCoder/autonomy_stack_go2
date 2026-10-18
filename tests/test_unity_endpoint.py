"""Check that Unity reconnects reuse existing ROS topic endpoints."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

try:
    from std_msgs.msg import String
    from ros_tcp_endpoint.server import SysCommands
except ImportError:
    SysCommands = None


@unittest.skipIf(SysCommands is None, "ROS 2 workspace is not sourced")
class UnityEndpointTest(unittest.TestCase):
    def test_duplicate_registrations_keep_live_nodes(self):
        server = SimpleNamespace(
            subscribers_table={}, publishers_table={}, executor=None,
            unregister_node=Mock(), loginfo=Mock(), send_unity_error=Mock())
        commands = SysCommands(server)
        with patch.object(commands, "resolve_message_name", return_value=String), \
                patch("ros_tcp_endpoint.server.RosSubscriber") as subscriber, \
                patch("ros_tcp_endpoint.server.RosPublisher") as publisher:
            subscriber.return_value.msg = String
            publisher.return_value.msg = String()
            commands.subscribe("/test_topic", "std_msgs/String")
            commands.subscribe("/test_topic", "std_msgs/String")
            commands.publish("/test_output", "std_msgs/String")
            commands.publish("/test_output", "std_msgs/String")

        subscriber.assert_called_once()
        publisher.assert_called_once()
        server.unregister_node.assert_not_called()
