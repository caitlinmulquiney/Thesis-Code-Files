import numpy as np

from Rbn import Rbn
from Tbn import Tbn

def Jbn(eta):
    J = np.block([
        [Rbn(eta), np.zeros((3, 3))],
        [np.zeros((3, 3)), Tbn(eta)]
    ])
    return J
