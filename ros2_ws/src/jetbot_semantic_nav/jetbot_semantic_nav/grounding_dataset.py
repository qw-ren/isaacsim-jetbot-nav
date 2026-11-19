"""Versioned synthetic warehouse language/object pairs. Geometry labels only offline.

Test wording families never occur in training. Seeds are disjoint. This is an
agent-authored synthetic dataset, not human-validated language acceptance.
"""
import math,random
CLASSES=('box','pallet','shelf','sign','cart')
ALIASES={'box':['纸箱','箱子'],'pallet':['托盘','货盘'],'shelf':['货架'],'sign':['标牌','标识牌'],'cart':['推车','手推车']}
NUMBERS={1:'一',2:'二',3:'三',4:'四',5:'五'}
PREFIXES={'train':['请去','走到','前往','帮我找到','带我去','请导航到','麻烦去','找到','可不可以去','能不能带我前往','不知能否前往','能否到'], 'dev':['我想去','请带我前往'], 'test':['麻烦你带我到','我需要前往','帮我导航到','能不能去一下']}
SELECT={'train':[['','那个','其中一个'],['最近的','离我最近的','距离我最近的','最近的一个','最近那个','距离最小的'],['最远的','离我最远的','距离我最远的','最远的一个','最远那个','距离最大的']], 'dev':[['那个'],['最近的那一个'],['最远的那一个']], 'test':[['那个'],['离我距离最短的','离我最近的那个'],['离我距离最大的','离我最远的那个']]}
REL={'train':['第{n}排货架旁边的','第{n}排货架附近的','靠近第{n}排货架的','在第{n}排货架附近的','第{n}排货架那边的','与第{n}排货架相邻的'], 'dev':['在第{n}排货架旁的'], 'test':['位于第{n}排货架边上的','紧挨着第{n}排货架的']}
NEG={'train':['不要移动','停止导航','告诉我电池电量','去找不存在的叉车','把纸箱搬过来','别找纸箱','先不要去货架','不用移动','去第二近的纸箱那里','找倒数第二远的托盘','找到一个人','我想找叉车','帮我把箱子拿来','告诉我现在的时间','今天星期几','电池还剩多少','介绍一下这台机器人','识别一下箱子的颜色','报告定位状态','帮我画个纸箱','原地等我一会儿'], 'dev':['请保持停车','现在不用前往纸箱'], 'test':['先别去找纸箱','不要前往最近的托盘','请问现在几点了','把那个箱子拿给我']}

def surface_gap(a,b):
    return math.hypot(*[max(abs(a['position'][j]-b['position'][j])-(a['dimensions'][j]+b['dimensions'][j])/2,0.) for j in range(2)])

def make_scene(seed):
    r=random.Random(seed);objects=[]
    for row in range(1,5):
        objects.append(dict(object_id='s_'+str(row),class_name='shelf',row_index=row,position=[row*5+r.uniform(-.3,.3),10.,1.5],dimensions=[1.,18.,3.],orientation_xyzw=[0.,0.,0.,1.]))
    for i in range(r.randint(4,12)):
        shelf=r.choice(objects[:4]);objects.append(dict(object_id='instance_'+str(i),class_name=r.choice(['box','pallet','sign','cart']),position=[shelf['position'][0]+r.choice([-1,1])*r.uniform(1.,3.),r.uniform(1,19),.5],dimensions=[r.uniform(.5,1.),r.uniform(.4,.9),1.],orientation_xyzw=[0.,0.,0.,1.]))
    r.shuffle(objects)
    return r,objects,[r.uniform(0,25),r.uniform(0,20)]

def label(objects,robot,slots):
    kind,row,selection=slots[:3]
    if len(slots)>3 and slots[3]==0:return 'NOT_FOUND',[False]*len(objects)
    shelf=next((o for o in objects if o.get('row_index')==row),None)
    eligible=[kind<5 and o['class_name']==CLASSES[kind] and (row==0 or shelf is not None and surface_gap(o,shelf)<=1.5) for o in objects]
    chosen=[o for o,ok in zip(objects,eligible) if ok]
    if not chosen:target='NOT_FOUND'
    elif selection==0 and len(chosen)>1:target='AMBIGUOUS'
    else:target=sorted(chosen,key=lambda o:math.dist(o['position'][:2],robot),reverse=selection==2)[0]['object_id']
    return target,eligible

def generate(split,count,offset):
    rows=[]
    for index in range(count):
        seed=offset+index;r,objects,robot=make_scene(seed)
        kind=r.randrange(5);row=r.choice([0,0,0,1,2,3,4,5]) if kind!=2 else 0;selection=r.randrange(3)
        slots=[kind,row,selection,1]
        name=r.choice(ALIASES[CLASSES[kind]]);n=r.choice([str(row),NUMBERS.get(row,'')]);relation=r.choice(REL[split]).format(n=n) if row else ''
        prefix=r.choice(PREFIXES[split]);qualifier=r.choice(SELECT[split][selection])
        text=prefix+relation+qualifier+name
        if split=='train':
            form=r.randrange(5)
            if form==1:text=prefix+qualifier+relation+name
            elif form==2:text=relation+name+'，'+prefix+qualifier+'那一个'
            elif form==3:text=prefix+name+'，选'+qualifier+'那个，'+relation
            elif form==4:text='目标是'+relation+qualifier+name+'，请带我过去'
            text+=r.choice(['','。','那里','旁边','，谢谢'])
        if split=='test':text+='那里。'
        if index%10==0:
            text=r.choice(NEG[split])
            # Annotation of known generated templates; never used as a runtime parser.
            named=[i for i,c in enumerate(CLASSES) if any(a in text for a in ALIASES[c])]
            slots=[named[0] if len(named)==1 else 5,0,1 if '最近' in text else 2 if '最远' in text else 0,0]
        elif split=='train' and index%5==1:
            # Counterfactual hard negatives: the same objects may exist, but no motion is requested.
            slots=[kind,row,selection,0]
            text=r.choice(['不要','先别','暂时不要','目前不用','不必'])+r.choice(['去','前往','导航到','寻找'])+relation+qualifier+name
        target,eligible=label(objects,robot,slots)
        rows.append(dict(text=text,objects=objects,robot_xy=robot,slots=slots,target=target,eligible=eligible,scene_seed=seed,split=split))
    return rows
