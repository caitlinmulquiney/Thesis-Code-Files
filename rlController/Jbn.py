import numpy as np
def Jbn(eta):
    J = np.array([[Rbn(eta), zeros(3,3)],
        [zeros(3,3), Tbn(eta)]])
    return J
