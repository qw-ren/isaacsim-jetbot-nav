"""Chinese pretrained intent heads plus learned object selection. Preview only."""
import torch,math
from torch import nn
from transformers import BertModel,BertConfig,BertTokenizerFast
from .grounding_dataset import CLASSES,surface_gap
from .grounding_contract import check_choice

class Parser(nn.Module):
    def __init__(self,encoder,include_intent=False,attention_pool=False):
        super().__init__();self.encoder=encoder;h=encoder.config.hidden_size
        self.attentions=nn.ModuleList(nn.Linear(h,1) for _ in range(4 if include_intent else 3)) if attention_pool else None
        self.heads=nn.ModuleList(nn.Linear(h,n) for n in ((6,6,3,2) if include_intent else (6,6,3)))
    def forward(self,**encoded):
        hidden=self.encoder(**encoded).last_hidden_state
        if self.attentions is None:return [head(hidden[:,0]) for head in self.heads]
        outputs=[]
        for attention,head in zip(self.attentions,self.heads):
            weights=attention(hidden).squeeze(-1).masked_fill(~encoded['attention_mask'].bool(),-1e9).softmax(-1)
            outputs.append(head((hidden*weights[:,:,None]).sum(1)))
        return outputs

class ObjectSelector(nn.Module):
    def __init__(self):
        super().__init__()
        self.eligibility=nn.Sequential(nn.Linear(3,32),nn.Tanh(),nn.Linear(32,1))
        self.preference=nn.Sequential(nn.Linear(4,32),nn.Tanh(),nn.Linear(32,1))
        self.reject=nn.Sequential(nn.Linear(4,32),nn.Tanh(),nn.Linear(32,2))
    def forward(self,f,mask,selection):
        valid=self.eligibility(f[:,:,:3]).squeeze(-1)
        sel=torch.nn.functional.one_hot(selection,3).float();expand=sel[:,None,:].expand(-1,f.shape[1],-1)
        preference=self.preference(torch.cat([f[:,:,3:4],expand],-1)).squeeze(-1)
        scores=(valid+preference).masked_fill(~mask,-1e9)
        count=(valid.sigmoid()*mask).sum(-1,keepdim=True)
        rejects=self.reject(torch.cat([count,sel],-1))
        return torch.cat([scores,rejects],-1),valid

def object_batch(rows,slots,device='cpu'):
    maximum=max(1,max(len(r['objects']) for r in rows));f=torch.zeros((len(rows),maximum,4));mask=torch.zeros((len(rows),maximum),dtype=torch.bool);eligible=torch.zeros_like(mask,dtype=torch.float);targets=[]
    for i,(r,slot) in enumerate(zip(rows,slots)):
        kind,row,selection=[int(x) for x in slot[:3]]
        if len(slot)>3 and int(slot[3])==0:kind=5
        objects=r['objects'];shelf=next((o for o in objects if o.get('row_index')==row),None)
        for j,o in enumerate(objects):
            gap=surface_gap(o,shelf) if shelf else 30.
            f[i,j]=torch.tensor([float(kind<5 and o['class_name']==CLASSES[kind]),0. if row==0 else min(gap,30.)/3.,float(row>0 and shelf is None),math.dist(o['position'][:2],r['robot_xy'])/30.]);mask[i,j]=True
        if 'eligible' in r:eligible[i,:len(objects)]=torch.tensor(r['eligible'])
        target=r.get('target','NOT_FOUND');targets.append(maximum if target=='NOT_FOUND' else maximum+1 if target=='AMBIGUOUS' else next(j for j,o in enumerate(objects) if o['object_id']==target))
    return f.to(device),mask.to(device),torch.tensor([int(s[2]) for s in slots],device=device),torch.tensor(targets,device=device),eligible.to(device)

class PretrainedGrounder:
    def __init__(self,checkpoint,min_confidence=.9,min_margin=.2):
        from pathlib import Path
        if not (.5<=min_confidence<=1 and 0<=min_margin<=1):raise ValueError('Invalid thresholds')
        base=Path(checkpoint);self.tokenizer=BertTokenizerFast.from_pretrained(base,local_files_only=True)
        state=torch.load(base/'grounding.pt',map_location='cpu',weights_only=True)
        if state['format_version']not in (2,3,4):raise ValueError('Invalid pretrained grounding version')
        self.parser=Parser(BertModel(BertConfig.from_json_file(str(base/'config.json'))),include_intent=state['format_version']>=3,attention_pool=state['format_version']==4);self.selector=ObjectSelector()
        self.parser.load_state_dict(state['parser']);self.selector.load_state_dict(state['selector']);self.parser.eval();self.selector.eval();self.confidence=min_confidence;self.margin=min_margin
    def predict(self,text,objects,robot_xy):
        if not isinstance(text,str) or not 0<len(text)<=128:raise ValueError('Invalid text')
        if len({o['object_id'] for o in objects})!=len(objects):raise ValueError('Duplicate IDs')
        if len(robot_xy)!=2 or not all(math.isfinite(v) for v in robot_xy):raise ValueError('Invalid position')
        if not all(len(o['position'])==3 and len(o['dimensions'])==3 and all(math.isfinite(v) for v in o['position']+o['dimensions']) and min(o['dimensions'])>0 for o in objects):raise ValueError('Invalid geometry')
        with torch.inference_mode():
            heads=self.parser(**self.tokenizer(text,return_tensors='pt',truncation=True,max_length=128));slots=[int(h.argmax(-1)) for h in heads];slot_confidence=min(float(h.softmax(-1).max()) for h in heads)
            r=dict(text=text,objects=objects,robot_xy=robot_xy);args=object_batch([r],[slots]);logits,_=self.selector(*args[:3]);prob=logits.softmax(-1)[0];v,ix=prob.topk(2);index=int(ix[0]);confidence=min(float(v[0]),slot_confidence);margin=float(v[0]-v[1]);n=args[0].shape[1]
        state='SELECTED' if index<len(objects) else 'NOT_FOUND' if index==n else 'AMBIGUOUS'
        if confidence<self.confidence or margin<self.margin:state='LOW_CONFIDENCE'
        violation=check_choice(objects,robot_xy,slots,objects[index]['object_id']) if state=='SELECTED' else None
        if violation:state='GROUNDING_INCONSISTENT'
        return dict(state=state,contract_violation=violation,proposed_object_id=objects[index]['object_id'] if index<len(objects) else None,object_id=objects[index]['object_id'] if state=='SELECTED' else None,confidence=confidence,margin=margin,slots=dict(object_class=CLASSES[slots[0]] if slots[0]<5 else 'unsupported',reference_row=slots[1],selection=['unique','nearest_euclidean','farthest_euclidean'][slots[2]],intent=('navigate' if slots[3] else 'no_navigation') if len(slots)>3 else 'not_separately_predicted'),backend='pretrained_chinese_encoder')
