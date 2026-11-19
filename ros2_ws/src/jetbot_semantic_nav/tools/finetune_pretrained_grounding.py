"""Two supervised neural stages, fixed disjoint splits, full heldout reporting."""
import argparse,hashlib,json,random,time
from pathlib import Path
import torch
from transformers import BertModel,BertTokenizerFast
from jetbot_semantic_nav.grounding_dataset import generate
from jetbot_semantic_nav.pretrained_grounding import Parser,ObjectSelector,object_batch

def evaluate(parser,selector,tokenizer,rows,device,oracle_slots=False):
    parser.eval();selector.eval();correct=accepted=accepted_correct=slot_ok=0;groups={};predictions=[];field_correct=None
    with torch.inference_mode():
        for start in range(0,len(rows),64):
            part=rows[start:start+64];encoded=tokenizer([r['text'] for r in part],padding=True,truncation=True,max_length=96,return_tensors='pt').to(device)
            heads=parser(**encoded);slot=torch.stack([h.argmax(-1) for h in heads],-1);gold=torch.tensor([r['slots'] for r in part],device=device);slot_ok+=int((slot==gold).all(-1).sum());field_correct=(slot==gold).sum(0) if field_correct is None else field_correct+(slot==gold).sum(0);slot_c=torch.stack([h.softmax(-1).max(-1).values for h in heads],-1).min(-1).values
            args=object_batch(part,gold if oracle_slots else slot,device);logits,_=selector(*args[:3]);prob=logits.softmax(-1);values,indices=prob.topk(2,-1)
            for i,r in enumerate(part):
                index=int(indices[i,0]);truth=int(args[3][i]);ok=index==truth;correct+=ok;conf=min(float(values[i,0]),float(slot_c[i]));margin=float(values[i,0]-values[i,1]);take=conf>=.9 and margin>=.2
                accepted+=take;accepted_correct+=take and ok
                category=r['target'] if r['target'] in ('NOT_FOUND','AMBIGUOUS') else 'OBJECT';group=groups.setdefault(category,[0,0]);group[0]+=ok;group[1]+=1
                n=args[0].shape[1];prediction=r['objects'][index]['object_id'] if index<len(r['objects']) else 'NOT_FOUND' if index==n else 'AMBIGUOUS'
                predictions.append(dict(text=r['text'],scene_seed=r['scene_seed'],expected=r['target'],prediction=prediction,slots=slot[i].tolist(),expected_slots=r['slots'],confidence=conf,margin=margin,accepted=take,correct=ok))
    return dict(samples=len(rows),accuracy=correct/len(rows),slot_exact_accuracy=slot_ok/len(rows),slot_field_accuracy={name:float(field_correct[i])/len(rows) for i,name in enumerate(['object_class','reference_row','selection','navigation_intent'][:len(field_correct)])},coverage=accepted/len(rows),accepted_accuracy=accepted_correct/accepted if accepted else None,groups={k:dict(correct=v[0],total=v[1]) for k,v in groups.items()}),predictions

def main():
    p=argparse.ArgumentParser();p.add_argument('--pretrained',required=True,type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--samples',type=int,default=8000);p.add_argument('--language-epochs',type=int,default=8);p.add_argument('--object-epochs',type=int,default=100);a=p.parse_args();a.output.mkdir(exist_ok=False,parents=True)
    torch.set_num_threads(4);torch.manual_seed(119);random.seed(119);device='cuda' if torch.cuda.is_available() else 'cpu'
    train=generate('train',a.samples,300000);dev=generate('dev',800,500000);test=generate('test',1000,700000)
    heldout_texts={r['text'] for r in dev+test}
    train=[r for r in train if r['text'] not in heldout_texts]
    for name,rows in [('train',train),('dev',dev),('test',test)]:
        (a.output/(name+'.jsonl')).write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows))
    manifest={name:dict(samples=len(rows),sha256=hashlib.sha256((a.output/(name+'.jsonl')).read_bytes()).hexdigest(),scene_seed_range=[min(r['scene_seed'] for r in rows),max(r['scene_seed'] for r in rows)]) for name,rows in [('train',train),('dev',dev),('test',test)]}
    train_texts={r['text'] for r in train};dev_texts={r['text'] for r in dev};test_texts={r['text'] for r in test}
    if train_texts & (dev_texts|test_texts):raise ValueError('Exact text leakage across train/heldout')
    tokenizer=BertTokenizerFast.from_pretrained(a.pretrained,local_files_only=True);parser=Parser(BertModel.from_pretrained(a.pretrained,local_files_only=True),include_intent=True,attention_pool=True).to(device);selector=ObjectSelector().to(device)
    optimizer=torch.optim.AdamW(parser.parameters(),lr=7e-5);best=0.;encoded=tokenizer([r['text'] for r in train],padding=True,truncation=True,max_length=96,return_tensors='pt');gold=torch.tensor([r['slots'] for r in train]);started=time.time()
    for epoch in range(a.language_epochs):
        parser.train();indices=torch.randperm(len(train));losses=0.
        for off in range(0,len(train),64):
            ix=indices[off:off+64];x={k:v[ix].to(device) for k,v in encoded.items()};target=gold[ix].to(device);optimizer.zero_grad();heads=parser(**x);loss=sum(torch.nn.functional.cross_entropy(h,target[:,j]) for j,h in enumerate(heads));loss.backward();torch.nn.utils.clip_grad_norm_(parser.parameters(),1.);optimizer.step();losses+=float(loss)
        parser.eval();count=0
        with torch.inference_mode():
            for off in range(0,len(dev),64):
                part=dev[off:off+64];heads=parser(**tokenizer([r['text'] for r in part],padding=True,return_tensors='pt').to(device));pred=torch.stack([h.argmax(-1) for h in heads],-1);count+=int((pred==torch.tensor([r['slots'] for r in part],device=device)).all(-1).sum())
        accuracy=count/len(dev);print(json.dumps(dict(stage='language',epoch=epoch+1,loss=losses,dev_slot_exact=accuracy,elapsed_s=time.time()-started)),flush=True)
        if accuracy>best:best=accuracy;torch.save(parser.state_dict(),a.output/'best_parser.pt')
    parser.load_state_dict(torch.load(a.output/'best_parser.pt',map_location=device,weights_only=True));args=object_batch(train,[r['slots'] for r in train],device);optimizer=torch.optim.AdamW(selector.parameters(),lr=.003);best=-1
    for epoch in range(a.object_epochs):
        selector.train();indices=torch.randperm(len(train),device=device);losses=0.
        for off in range(0,len(train),128):
            ix=indices[off:off+128];optimizer.zero_grad();logits,valid=selector(args[0][ix],args[1][ix],args[2][ix]);loss=torch.nn.functional.cross_entropy(logits,args[3][ix])+torch.nn.functional.binary_cross_entropy_with_logits(valid[args[1][ix]],args[4][ix][args[1][ix]]);loss.backward();optimizer.step();losses+=float(loss)
        if (epoch+1)%10==0:
            metric,_=evaluate(parser,selector,tokenizer,dev,device);print(json.dumps(dict(stage='objects',epoch=epoch+1,loss=losses,dev=metric)),flush=True)
            if metric['accuracy']>best:best=metric['accuracy'];torch.save(selector.state_dict(),a.output/'best_selector.pt')
    selector.load_state_dict(torch.load(a.output/'best_selector.pt',map_location=device,weights_only=True));tokenizer.save_pretrained(a.output);parser.encoder.config.to_json_file(a.output/'config.json')
    torch.save(dict(format_version=4,parser={k:v.cpu() for k,v in parser.state_dict().items()},selector={k:v.cpu() for k,v in selector.state_dict().items()},selection_metric='Euclidean object-center distance; not Nav2 path length',qualified=False),a.output/'grounding.pt')
    report=dict(prototype_only=True,pretrained_model=json.loads((a.pretrained/'DOWNLOAD_PROVENANCE.json').read_text()),dataset=manifest,parameters=sum(p.numel() for p in parser.parameters())+sum(p.numel() for p in selector.parameters()),device=device,elapsed_s=time.time()-started,checkpoint_sha256=hashlib.sha256((a.output/'grounding.pt').read_bytes()).hexdigest(),live_navigation_qualified=False)
    for name,rows in [('dev',dev),('test',test)]:
        metric,predictions=evaluate(parser,selector,tokenizer,rows,device);report[name]=metric;(a.output/(name+'_predictions.jsonl')).write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in predictions))
    report['test_with_oracle_language_slots']=evaluate(parser,selector,tokenizer,test,device,True)[0]
    (a.output/'EVALUATION.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
if __name__=='__main__':main()
