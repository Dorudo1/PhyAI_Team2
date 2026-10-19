"""Read-only master subscriber for camera-driven discovery results."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..handoff import Master


class MasterNode(Node):
    def __init__(self):
        super().__init__('junh_master')
        self.declare_parameter('input_timeout', 3.)
        self.timeout = float(self.get_parameter('input_timeout').value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('input_timeout must be positive and finite.')
        self.master = Master()
        self.received = None
        self.create_subscription(String,'junh/discovery',self.on_discovery,1)
        self.pub = self.create_publisher(String,'junh/status',1)
        self.create_timer(.2,self.tick)

    def on_discovery(self,msg):
        try:
            self.master.consume(json.loads(msg.data))
            self.received = time.monotonic()
        except (ValueError,TypeError,KeyError,OverflowError) as error:
            self.master.wait(f'Invalid discovery message: {error}')
            self.received = None
            self.get_logger().error(self.master.reason)

    def tick(self):
        if self.received is None or time.monotonic()-self.received > self.timeout:
            self.master.wait('Waiting for a live discovery publisher; saved map is not current.')
        self.pub.publish(String(data=json.dumps(self.master.status(),allow_nan=False)))


def main(args=None):
    from .runtime import run
    run(MasterNode,args)
