import numpy as np

def skewSym(v):
    S = np.array([[0,-v[2],v[1]],
        [v[2], 0,-v[0]],
        [-v[1], v[0],0]])
    return S