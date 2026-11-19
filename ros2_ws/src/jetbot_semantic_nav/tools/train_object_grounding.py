"""Reproducible synthetic prototype, disjoint scene seeds and phrasing families.
Not real language/vision or navigation acceptance. No weights installed into sim.
"""
import argparse,json,random,hashlib
from pathlib import Path
import torch
from jetbot_semantic_nav.object_grounding import Listener,batch

CLASSES={'box':'纸箱','pallet':'托盘','cart':'推车','sign':'标牌','shelf':'货架'}

def scene(seed):
    r=random.Random(seed);objects=[]
    def obj(key,kind,x,y,dims,row=None):
        o=dict(object_id=key,class_name=kind,position=[x,y,.5],dimensions=dims,orientation_xyzw=[0.,0.,0.,1.])
        if row:o['row_index']=row
        objects.append(o)
    for i in range(1,5):obj('s'+str(i),'shelf',i*5,10,[1.,18.,3.],i)
    for i in range(r.randint(3,9)):
        row=r.randint(1,4);obj('o'+str(i),r.choice(['box','pallet','cart','sign']),row*5+r.choice([-1,1])*r.uniform(1.,2.5),r.uniform(2,18),[.7,.5,1.])
    r.shuffle(objects)
    return r,objects,[r.uniform(0,25),r.uniform(0,20)]


def generate(start,count,heldout=False):
    data=[]
    for seed in range(start,start+count):
        r,objects,robot=scene(seed)
        kind=r.choice(list(CLASSES));selection=r.choice(['nearest','farthest','unique']);row=r.choice([None,None,1,2,3,4,5]);relation='' if row is None else '第'+str(row)+'排货架旁边的'
        choice={'nearest':'最近的','farthest':'最远的','unique':''}[selection]
        prefix=r.choice(['请去','走到','找到']) if not heldout else r.choice(['麻烦带我去','我想前往'])
        text=prefix+relation+choice+CLASSES[kind]+('那里' if heldout else '')
        selected=[o for o in objects if o['class_name']==kind]
        if row is not None:
            shelf=next((o for o in objects if o.get('row_index')==row),None)
            selected=[] if shelf is None else [o for o in selected if max(abs(o['position'][0]-shelf['position'][0])-(o['dimensions'][0]+1)/2,0)<=1.5]
        if not selected:target='NOT_FOUND'
        elif selection=='unique' and len(selected)>1:target='AMBIGUOUS'
        else:
            selected.sort(key=lambda o:(o['position'][0]-robot[0])**2+(o['position'][1]-robot[1])**2,reverse=selection=='farthest');target=selected[0]['object_id']
        data.append(dict(text=text,objects=objects,robot_xy=robot,target=target,scene_seed=seed,split='heldout' if heldout else 'train'))
    return data


def evaluate(model,rows,vocab):
    correct=accepted=accepted_correct=0;groups={}
    model.eval()
    with torch.inference_mode():
        for offset in range(0,len(rows),64):
            part=rows[offset:offset+64];args=batch(part,vocab);probs=model(*args[:4]).softmax(-1);values,indices=probs.topk(2,dim=1)
            for i,r in enumerate(part):
                prediction=int(indices[i,0]);truth=int(args[4][i]);ok=prediction==truth;correct+=ok
                category=r['target'] if r['target'] in ('NOT_FOUND','AMBIGUOUS') else 'OBJECT';g=groups.setdefault(category,[0,0]);g[0]+=ok;g[1]+=1
                if float(values[i,0])>=.9 and float(values[i,0]-values[i,1])>=.2:accepted+=1;accepted_correct+=ok
    return dict(samples=len(rows),accuracy=correct/len(rows),accepted=accepted,coverage=accepted/len(rows),accepted_accuracy=accepted_correct/accepted if accepted else None,groups={k:dict(correct=v[0],total=v[1]) for k,v in groups.items()})


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--epochs',type=int,default=80);p.add_argument('--samples',type=int,default=4000);a=p.parse_args();a.output.mkdir(exist_ok=False,parents=True)
    torch.set_num_threads(4);torch.manual_seed(19);random.seed(19)
    train=generate(0,a.samples);dev=generate(100000,400);test=generate(200000,500,True)
    # Vocabulary covers character inventory, not scene labels. Heldout phrase composition remains unseen.
    chars=sorted(set(''.join(r['text'] for r in train)+ '麻烦带我去我想前往那里'))
    vocab={'<PAD>':0,'<UNK>':1};vocab.update({c:i+2 for i,c in enumerate(chars)})
    model=Listener(len(vocab));optimizer=torch.optim.AdamW(model.parameters(),lr=.002);best=-1
    for epoch in range(a.epochs):
        model.train();random.shuffle(train);total=0.
        for offset in range(0,len(train),64):
            args=batch(train[offset:offset+64],vocab);optimizer.zero_grad();loss=torch.nn.functional.cross_entropy(model(*args[:4]),args[4]);loss.backward();torch.nn.utils.clip_grad_norm_(model.parameters(),1.);optimizer.step();total+=float(loss)
        if (epoch+1)%5==0:
            metrics=evaluate(model,dev,vocab);print(json.dumps(dict(epoch=epoch+1,loss=total,dev=metrics)),flush=True)
            if metrics['accuracy']>best:
                best=metrics['accuracy'];torch.save(dict(format_version=1,vocab=vocab,state_dict=model.state_dict(),training='synthetic from scratch; no natural language acceptance',selection_metric='Euclidean robot-to-object center; not Nav2 path distance'),a.output/'listener.pt')
    artifact=torch.load(a.output/'listener.pt',weights_only=True);model.load_state_dict(artifact['state_dict'])
    report=dict(prototype_only=True,trained_from_scratch=True,parameters=sum(p.numel() for p in model.parameters()),train_samples=len(train),dev=evaluate(model,dev,vocab),heldout_scene_and_phrase=evaluate(model,test,vocab),checkpoint_sha256=hashlib.sha256((a.output/'listener.pt').read_bytes()).hexdigest(),not_navigation_acceptance=True)
    (a.output/'EVALUATION.json').write_text(json.dumps(report,indent=2))
    for name,rows in [('train',train),('dev',dev),('test',test)]:
        (a.output/(name+'.jsonl')).write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows))
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
