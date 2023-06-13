# Everything's from Mr. P Solver: https://www.youtube.com/watch?v=MM3cBamj1Ms

# packages

import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
from scipy.integrate import odeint      # classic. lsoda solver.
from scipy.integrate import solve_ivp   # customizable. list of sovlers

## First order ODEs
# defining the function
def dvdt(t, v):
    return 3*v**2 - 5
v0 = 0

# time range
t = np.linspace(0, 1, 100)
sol_m1 = odeint(dvdt, y0=v0, t=t, tfirst=True)  # solving with odeint
sol_m2 = solve_ivp(dvdt, t_span=(0,max(t)), y0=[v0], t_eval=t)

print(sol_m1)