"""Shared ROS lifecycle with cleanup protected from repeated interrupt signals."""
import signal

import rclpy
from rclpy.executors import ExternalShutdownException


def run(factory,args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=factory()
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can race subscription conversion with SIGINT context shutdown.
        # Keep genuine runtime failures visible while the context is still alive.
        if rclpy.ok():
            raise
    finally:
        # Terminal/process-group SIGINT plus launch forwarding can arrive twice.
        # Finish worker/entity cleanup before accepting another interrupt.
        previous=signal.signal(signal.SIGINT,signal.SIG_IGN)
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
            signal.signal(signal.SIGINT,previous)
