"""RViz MarkerArray subscriber; never reconstructs a separate workspace map."""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from ..viewer.visualization import primitives


class ViewerNode(Node):
    def __init__(self):
        super().__init__('junh_workspace_viewer')
        self.declare_parameter('stale_seconds',10.)
        self.declare_parameter('http_port',0)
        self.declare_parameter('simulator_port',0)
        self.web=None
        port=int(self.get_parameter('http_port').value)
        if port:
            from ..viewer.web import WorkspaceWeb
            self.web=WorkspaceWeb(port,int(self.get_parameter('simulator_port').value))
            self.get_logger().info(f'App workspace map: http://localhost:{port}')
        self.latest = None
        self.received = None
        self.create_subscription(String,'junh/discovery',self.on_workspace,1)
        self.pub = self.create_publisher(MarkerArray,'junh/workspace_markers',1)
        self.create_timer(.5,self.tick)

    def on_workspace(self,msg):
        try:
            data=json.loads(msg.data)
            primitives(data)  # validate shape before replacing the last display
            self.latest,self.received=data,time.monotonic()
            if self.web is not None:self.web.update(data)
        except (ValueError,KeyError,TypeError,ZeroDivisionError) as error:
            self.get_logger().error(f'Invalid viewer snapshot: {error}')

    def tick(self):
        if self.latest is None or self.pub.get_subscription_count()==0:
            return
        # Recorded acquisition time is separate from publisher liveness.
        now = self.get_clock().now().nanoseconds/1e9 if self.latest.get('input_mode') == 'live' else self.latest['workspace']['stamp']
        items=primitives(self.latest,now,float(self.get_parameter('stale_seconds').value))
        if time.monotonic()-self.received > 3.:
            for item in items:
                item['color']=(.6,.25,.6,.4)
            items[-1]['text']='STALE DISCOVERY PUBLISHER\n'+items[-1]['text']
        clear=Marker()
        clear.action=Marker.DELETEALL
        messages=[clear]
        for index,item in enumerate(items):
            marker=Marker()
            marker.header.frame_id=item['frame']
            marker.header.stamp=self.get_clock().now().to_msg()
            marker.ns='workspace'
            marker.id=index
            marker.type=Marker.TEXT_VIEW_FACING if item['kind']=='label' else Marker.CUBE
            marker.action=Marker.ADD
            marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=map(float,item['position'])
            w,x,y,z=item['quaternion']
            marker.pose.orientation.w,marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z=map(float,(w,x,y,z))
            marker.scale.x,marker.scale.y,marker.scale.z=map(float,item['scale'])
            marker.color.r,marker.color.g,marker.color.b,marker.color.a=map(float,item['color'])
            marker.text=item.get('text','')
            messages.append(marker)
        self.pub.publish(MarkerArray(markers=messages))


    def destroy_node(self):
        if self.web is not None:self.web.close()
        return super().destroy_node()


def main(args=None):
    from .runtime import run
    run(ViewerNode,args)
