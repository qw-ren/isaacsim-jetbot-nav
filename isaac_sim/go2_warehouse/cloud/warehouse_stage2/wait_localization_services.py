"""Wait for lifecycle replies, optionally until all requested nodes are active."""
import time,json,argparse
import rclpy
from lifecycle_msgs.srv import GetState
p=argparse.ArgumentParser();p.add_argument('--nodes',nargs='+',default=['map_server','amcl']);p.add_argument('--require-active',action='store_true');a=p.parse_args();rclpy.init();n=rclpy.create_node('go2_localization_service_readiness');clients={x:n.create_client(GetState,'/'+x+'/get_state') for x in a.nodes};out={};deadline=time.monotonic()+(120 if a.require_active else 40);rounds=0
while time.monotonic()<deadline:
 for name,c in clients.items():
  if not c.wait_for_service(timeout_sec=max(.1,min(5,deadline-time.monotonic()))):continue
  f=c.call_async(GetState.Request());end=min(deadline,time.monotonic()+8)
  while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
  if f.done():out[name]=f.result().current_state.label
 rounds+=1
 if len(out)==len(clients) and (all(v=='active' for v in out.values()) if a.require_active else rounds>=2):break
 until=time.monotonic()+.5
 while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.05)
ready=len(out)==len(clients) and (not a.require_active or all(v=='active' for v in out.values()));print(json.dumps({'lifecycle_services_ready':out,'ready':ready}));n.destroy_node();rclpy.shutdown()
if not ready:raise SystemExit('Lifecycle readiness timeout')
