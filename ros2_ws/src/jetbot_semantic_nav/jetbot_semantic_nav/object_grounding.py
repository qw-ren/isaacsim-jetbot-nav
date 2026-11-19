"""Small trained language/object listener. No ROS, coordinates regressor, or rule parser.

Prototype vocabulary and model are task-specific, trained from scratch. Not a
pretrained Chinese language model. Candidate order and IDs are not model inputs.
"""
import math
import torch
from torch import nn

CLASSES = ('box', 'pallet', 'shelf', 'sign', 'cart')
FEATURE_DIM = len(CLASSES) + 4 + 4


def features(objects, robot_xy):
    shelves = [o for o in objects if o['class_name'] == 'shelf']
    output = []
    for o in objects:
        x,y = o['position'][:2]
        distances = []
        for row in range(1,5):
            options = [s for s in shelves if s.get('row_index') == row]
            distances.append(min([math.hypot(max(abs(x-s['position'][0])-(o['dimensions'][0]+s['dimensions'][0])/2,0), max(abs(y-s['position'][1])-(o['dimensions'][1]+s['dimensions'][1])/2,0)) for s in options] or [30.])/30.)
        output.append([float(o['class_name']==c) for c in CLASSES] + [(x-robot_xy[0])/30.,(y-robot_xy[1])/30., math.dist([x,y],robot_xy)/30.,o.get('row_index',0)/4.] + distances)
    return output


class Listener(nn.Module):
    def __init__(self, vocabulary_size, width=64):
        super().__init__()
        self.embedding = nn.Embedding(vocabulary_size,32,padding_idx=0)
        self.language = nn.GRU(32,width,batch_first=True)
        self.objects = nn.Sequential(nn.Linear(FEATURE_DIM,width),nn.ReLU(),nn.Linear(width,width))
        self.score = nn.Sequential(nn.Linear(width*3,width),nn.ReLU(),nn.Linear(width,1))
        self.reject = nn.Sequential(nn.Linear(width*2,width),nn.ReLU(),nn.Linear(width,2))

    def forward(self,tokens,lengths,nodes,mask):
        embedded=self.embedding(tokens)
        sequence=nn.utils.rnn.pack_padded_sequence(embedded,lengths.cpu(),batch_first=True,enforce_sorted=False)
        _,h=self.language(sequence);query=h[-1]
        objects=self.objects(nodes);q=query[:,None,:].expand_as(objects)
        scores=self.score(torch.cat([q,objects,q*objects],dim=-1)).squeeze(-1).masked_fill(~mask,-1e9)
        pooled=(objects*mask[:,:,None]).sum(1)/mask.sum(1,keepdim=True).clamp_min(1)
        return torch.cat([scores,self.reject(torch.cat([query,pooled],dim=-1))],dim=1)


def batch(rows,vocab):
    maximum=max(1,max(len(r['objects']) for r in rows));size=max(len(r['text']) for r in rows)
    tokens=torch.zeros((len(rows),size),dtype=torch.long);nodes=torch.zeros((len(rows),maximum,FEATURE_DIM));mask=torch.zeros((len(rows),maximum),dtype=torch.bool)
    labels=[];lengths=[]
    for i,r in enumerate(rows):
        text=r['text'];lengths.append(len(text));tokens[i,:len(text)]=torch.tensor([vocab.get(c,1) for c in text])
        if r['objects']:
            nodes[i,:len(r['objects'])]=torch.tensor(features(r['objects'],r['robot_xy']));mask[i,:len(r['objects'])]=True
        target=r.get('target')
        labels.append(maximum if target=='NOT_FOUND' else maximum+1 if target=='AMBIGUOUS' else next(j for j,o in enumerate(r['objects']) if o['object_id']==target) if target is not None else maximum)
    return tokens,torch.tensor(lengths),nodes,mask,torch.tensor(labels)


class Grounder:
    def __init__(self,checkpoint,min_confidence=.9,min_margin=.2):
        if not (.5<=min_confidence<=1 and 0<=min_margin<=1):raise ValueError('Invalid confidence thresholds')
        artifact=torch.load(checkpoint,map_location='cpu',weights_only=True)
        if artifact['format_version']!=1:raise ValueError('Unsupported model format')
        self.vocab=artifact['vocab'];self.model=Listener(len(self.vocab));self.model.load_state_dict(artifact['state_dict']);self.model.eval()
        self.min_confidence=min_confidence;self.min_margin=min_margin

    def predict(self,text,objects,robot_xy):
        if not isinstance(text,str) or not 0<len(text)<=128:raise ValueError('Invalid query length')
        if len({o['object_id'] for o in objects})!=len(objects):raise ValueError('Duplicate candidate IDs')
        if not all(len(o['position'])==3 and len(o['dimensions'])==3 and all(math.isfinite(v) for v in o['position']+o['dimensions']) for o in objects):raise ValueError('Invalid object geometry')
        if len(robot_xy)!=2 or not all(math.isfinite(v) for v in robot_xy):raise ValueError('Invalid robot position')
        unknown=sum(c not in self.vocab for c in text)/len(text)
        if unknown>.15:return dict(state='LOW_CONFIDENCE',object_id=None,unknown_character_fraction=unknown)
        args=batch([dict(text=text,objects=objects,robot_xy=robot_xy)],self.vocab)
        with torch.inference_mode():probabilities=self.model(*args[:4]).softmax(-1)[0]
        values,indices=probabilities.topk(2);confidence=float(values[0]);margin=float(values[0]-values[1]);index=int(indices[0]);maximum=args[2].shape[1]
        state='SELECTED' if index< len(objects) else 'NOT_FOUND' if index==maximum else 'AMBIGUOUS'
        if confidence<self.min_confidence or margin<self.min_margin:state='LOW_CONFIDENCE'
        return dict(state=state,object_id=objects[index]['object_id'] if state=='SELECTED' else None,confidence=confidence,margin=margin,unknown_character_fraction=unknown)
