import numpy as np
from evaluate_range import rotation, iou

assert np.allclose(rotation([0,0,0,1]),np.eye(3))
assert np.allclose(rotation([0,0,1,1])@np.array([1,0,0]),[0,1,0])
assert iou([0,0,10,10],[0,0,10,10])==1
assert iou([0,0,1,1],[2,2,3,3])==0
print('Projection helper checks passed')
