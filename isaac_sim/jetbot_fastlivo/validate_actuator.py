"""Execute the actual actuator watchdog method against controlled clock/loss cases.

No simulator, robot or ROS connection. The tested source SHA binds the proof.
"""
import argparse,ast,hashlib,io,json,types
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
tree=ast.parse(a.source.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='SensorBridge');method=next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name=='command');scope={'time':types.SimpleNamespace(monotonic=lambda:10.),'json':json};exec(compile(ast.Module(body=[method],type_ignores=[]),str(a.source),'exec'),scope);command=scope['command'];facts=[]
def state(last=None,wall=None,sim=None,fault=None):return types.SimpleNamespace(last_command=last,last_wall=wall,last_sim=sim,command_fault=fault,world=types.SimpleNamespace(current_time=5.),controls=io.StringIO())
def check(name,obj,expected,fault):
 actual=command(obj);assert actual==expected,(name,actual);assert obj.command_fault==fault,(name,obj.command_fault);facts.append(dict(case=name,output=actual,fault=obj.command_fault));return obj
check('no publisher yet',state(),[0.,0.],None)
check('zero-only startup gap',state([0.,0.],9.,4.),[0.,0.],None)
obj=check('fresh motion',state([.15,.2],9.9,4.9),[.15,.2],None)
obj.last_wall=9.;check('motion wall loss',obj,[0.,0.],'COMMAND_TIMEOUT')
obj.last_wall=9.99;obj.last_sim=4.99;check('new message does not clear motion-loss latch',obj,[0.,0.],'COMMAND_TIMEOUT')
check('motion simulation age loss',state([.15,0.],9.99,4.),[0.,0.],'COMMAND_TIMEOUT')
check('nonfinite message fault stays latched',state([.1,0.],9.99,4.99,'NONFINITE_COMMAND'),[0.,0.],'NONFINITE_COMMAND')
result=dict(passed=True,source_sha256=hashlib.sha256(a.source.read_bytes()).hexdigest(),scope='actual command method; controlled source clocks; no ROS/robot connection; not a wheel dynamics test',facts=facts)
if a.output.exists():raise RuntimeError('Refuse overwriting validation')
a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
