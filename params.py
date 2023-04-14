# Physical problem
M_REAL = 0.5      # Mass of the cart
LAMBDA_P = 1  # Penalization for not reaching the target at time T
LAMBDA_V = 100   # Penalization for not reaching the target at time T
T = 1           # Time horizon of the problem

# Time discretization
N = 5         # Number of time steps, common for: the simulator (by default);
            # the data collection for model identification; the control obtained
            # by traditional methods; the DP formulation; the RL formulation.
DT = T/N        # Length of a time step

# Dynamic programming
U_L, U_R = -1, 1  # Bounds for the control
N_U = 20  # Resolution of the control
DU = 1/N_U  # Step between two possible values of the control
V_L, V_R = U_L/M_REAL*T, U_R/M_REAL*T  # Bounds for the velocity
DV = DU/M_REAL*DT  # Step between two possible values of the velocity
N_V = round(1/DV)  # Resolution of the velocity
X_L, X_R = -1+U_L/(2*M_REAL)*T**2, U_R/(2*M_REAL)*T**2  # Bounds for the position
DX = (DT**2)/(2*M_REAL)*DU  # Step between two possible values of the position
N_X = round(1/DX)  # Resolution of the position
