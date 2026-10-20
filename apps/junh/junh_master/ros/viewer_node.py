"""Live workspace viewer with a completion-triggered GetBlockInfo client."""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from phyai_discovery_interfaces.srv import GetBlockInfo
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
        self.block_client=self.create_client(GetBlockInfo,'junh/get_block_info')
        self.block_key=None
        self.block_query=None
        self.block_query_started=None
        self.next_block_query=0.
        self.block_info={'status':'waiting','message':'Waiting for search to finish.'}
        self.create_subscription(String,'junh/discovery',self.on_workspace,1)
        self.pub = self.create_publisher(MarkerArray,'junh/workspace_markers',1)
        self.create_timer(.5,self.tick)

    def on_workspace(self,msg):
        try:
            data=json.loads(msg.data)
            primitives(data)  # validate shape before replacing the last display
            key=(data['workspace']['session_id'],data['workspace']['revision']) if data['search']['state']=='search_finished' else None
            if key!=self.block_key:
                if self.block_query is not None:self.block_query.cancel()
                self.block_key=key;self.block_query=None;self.next_block_query=0.
                self.block_info={'status':'pending' if key else 'waiting',
                                 'message':'Querying final block information.' if key else 'Waiting for search to finish.'}
            self.latest,self.received=data,time.monotonic()
            self.update_display()
        except (ValueError,KeyError,TypeError,ZeroDivisionError) as error:
            self.get_logger().error(f'Invalid viewer snapshot: {error}')

    def update_display(self):
        if self.latest is None:return
        self.latest['block_info']=self.block_info
        if self.block_info['status']=='ready':
            self.latest['workspace']={**self.latest['workspace'],
                                      'tracks':self.block_info['blocks']}
        if self.web is not None:self.web.update(self.latest)

    def query_block_info(self):
        if self.block_key is None:return
        now=time.monotonic()
        if self.block_query is not None:
            if not self.block_query.done() and now-self.block_query_started<5.:return
            try:
                if not self.block_query.done():
                    self.block_query.cancel()
                    raise ValueError('Block information query timed out.')
                response=self.block_query.result()
                if not response.success:raise ValueError(response.message)
                if (response.session_id,response.revision)!=self.block_key or not response.finished:
                    raise ValueError('Block information does not match the finished scan.')
                snapshot=json.loads(response.snapshot_json)
                primitives(snapshot)
                if ((snapshot['workspace']['session_id'],snapshot['workspace']['revision'])!=self.block_key
                        or snapshot['search']['state']!='search_finished'):
                    raise ValueError('Returned snapshot does not match the finished scan.')
                self.block_info={'status':'ready','message':'Final inventory retrieved with GetBlockInfo.',
                    'discovery_state':response.discovery_state,'search_status':response.search_status,
                    'session_id':response.session_id,'revision':response.revision,
                    'frame_id':response.frame_id,'blocks':snapshot['workspace']['tracks']}
            except Exception as error:
                self.block_info={'status':'error','message':str(error)}
                self.next_block_query=now+2.
            self.block_query=None
            self.update_display()
        if self.block_info['status']=='ready' or now<self.next_block_query:return
        if not self.block_client.service_is_ready():return
        request=GetBlockInfo.Request(session_id=self.block_key[0])
        self.block_query=self.block_client.call_async(request)
        self.block_query_started=now

    def tick(self):
        self.query_block_info()
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
        if self.block_query is not None:self.block_query.cancel()
        if self.web is not None:self.web.close()
        return super().destroy_node()


def main(args=None):
    from .runtime import run
    run(ViewerNode,args)
