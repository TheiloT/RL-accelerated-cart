# Physical problem
M_REAL = 5      # Mass of the cart
LAMBDA_P = 100  # Penalization for not reaching the target at time T
LAMBDA_V = 200   # Penalization for not reaching the target at time T
T = 1           # Time horizon of the problem

# Time discretization
N = 100         # Number of time steps, common for the simulator (by default),
            # the data collection for model identification, the control obtained
            # by traditional methods and the RL control.
DT = T/N        # Length of a time step