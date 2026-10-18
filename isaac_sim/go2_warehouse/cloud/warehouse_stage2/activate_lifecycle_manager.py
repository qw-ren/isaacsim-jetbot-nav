"""Delay native Nav2 startup until lifecycle service discovery has settled."""
import argparse,time,json
import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
p=argparse.ArgumentParser();p.add_argument('--manager',required=True);p.add_argument('--nodes',nargs='+',required=True);a=p.parse_args();rclpy.init();n=rclpy.create_node('go2_delayed_lifecycle_startup')
def call(c,req,seconds):
 if not c.wait_for_service(timeout_sec=15):raise RuntimeError('Lifecycle service unavailable')
 f=c.call_async(req);end=time.monotonic()+seconds
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 if not f.done():
  f.cancel()
  raise RuntimeError('Lifecycle service response timeout')
 return f.result()
def get_state(name):
 for attempt in range(3):
  c=n.create_client(GetState,'/'+name+'/get_state')
  try:return call(c,GetState.Request(),8).current_state.label
  except RuntimeError:
   if attempt==2:raise
   print(json.dumps({'read_only_state_retry':name,'attempt':attempt+1}),flush=True)
  finally:n.destroy_client(c)
  end=time.monotonic()+1
  while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
for name in a.nodes:get_state(name)
manager=n.create_client(ManageLifecycleNodes,'/'+a.manager+'/manage_nodes')
if not manager.wait_for_service(timeout_sec=15):raise RuntimeError('Native lifecycle manager unavailable')
end=time.monotonic()+2
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
req=ManageLifecycleNodes.Request();req.command=0;response=call(manager,req,60)
if not response.success:raise RuntimeError('Native lifecycle startup rejected')
states={x:get_state(x) for x in a.nodes};print(json.dumps({'manager':a.manager,'states':states}));n.destroy_node();rclpy.shutdown()
if not all(x=='active' for x in states.values()):raise SystemExit('Managed nodes not active')
