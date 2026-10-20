"""Compatibility entry point for the shared discovery ROS adapter."""
from phyai_discovery_ros.node import DiscoveryNode as SharedDiscoveryNode
from phyai_discovery_ros.runtime import run


class DiscoveryNode(SharedDiscoveryNode):
    def __init__(self, **kwargs):
        super().__init__(node_name='junh_discovery', config_package='junh_master',
                         interface_prefix='junh', inventory_name='discovery', **kwargs)


def main(args=None):
    run(DiscoveryNode, args)
