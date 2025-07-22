import numpy as np
import matplotlib.pyplot as plt

# Parameters
N = 4                # Number of oscillators
T = 20.0             # Total simulation time (s)
dt = 0.01            # Time step (s)
time = np.arange(0, T, dt)

# Natural frequencies (rad/s)
omega = np.array([1.0, 1.0, 1.0, 1.0])

# Coupling weight matrix
W = np.full((N, N), 0.5)
np.fill_diagonal(W, 0.5)

# Coupling phases φ_i
phi = np.array([0, np.pi/2, np.pi, 3*np.pi/2])

# 1. Simulate phase θ evolution
theta = np.zeros((len(time), N))
theta[0] = np.random.rand(N) * 2 * np.pi

for k in range(len(time) - 1):
    th = theta[k]
    dth = omega.copy()
    for i in range(N):
        dth[i] += np.sum(W[i] * np.sin(th - th[i] - phi[i]))
    theta[k + 1] = th + dth * dt

theta_mod = np.mod(theta, 2*np.pi)

# 2. Generate xi based on θ (1 for swing, -0.1 for stance)
xi = np.where(theta_mod < np.pi/2, 1.0, -0.1)

# 3. Accelerate p2 response by increasing gain a
a = 10.0  

p2 = np.zeros((len(time), N))
p2_dot = np.zeros((len(time), N))

for k in range(len(time) - 1):
    for i in range(N):
        ddot = a * (a/4 * (xi[k, i] - p2[k, i]) - p2_dot[k, i])
        p2_dot[k+1, i] = p2_dot[k, i] + ddot * dt
        p2[k+1, i] = p2[k, i] + p2_dot[k, i] * dt

# 4. Compute b(θ)
def b_of_s(s, p0, p2_val, p3):
    return ((1-s)**4 + 4*(1-s)**3 * s) * p0 + \
           6*(1-s)**2 * s**2 * p2_val + \
           (4*(1-s) * s**3 + s**4) * p3

p0_sw, p3_sw = 0.0, 0.0
p0_st, p3_st = 0.0, 0.0

b = np.zeros_like(theta_mod)
for i in range(N):
    mask_sw = theta_mod[:, i] < np.pi/2
    s_sw = (2/np.pi) * theta_mod[:, i]
    s_st = (2/(3*np.pi)) * theta_mod[:, i] - 1/3

    b[mask_sw, i] = b_of_s(s_sw[mask_sw], p0_sw, p2[mask_sw, i], p3_sw)
    b[~mask_sw, i] = b_of_s(s_st[~mask_sw], p0_st, p2[~mask_sw, i], p3_st)

# 5. Plot four subplots in one figure
fig, axs = plt.subplots(4, 1, figsize=(8, 12), sharex=True)

# Subplot 1: θ evolution
for i in range(N):
    axs[0].plot(time, theta_mod[:, i], label=f'θ_{i+1}')
axs[0].set_ylabel('θ (rad)')
axs[0].set_title('Oscillator Phase Evolution θ')
axs[0].legend()

# Subplot 2: xi signal
for i in range(N):
    axs[1].plot(time, xi[:, i], label=f'ξ_{i+1}')
axs[1].set_ylabel('ξ value')
axs[1].set_title('xi Signal (1: Swing, -0.1: Stance)')
axs[1].legend()

# Subplot 3: p2 response
for i in range(N):
    axs[2].plot(time, p2[:, i], label=f'p2_{i+1}')
axs[2].set_ylabel('p2 value')
axs[2].set_title('Accelerated p2 Response')
axs[2].legend()

# Subplot 4: b output
for i in range(N):
    axs[3].plot(time, b[:, i], label=f'b_{i+1}')
axs[3].set_xlabel('Time (s)')
axs[3].set_ylabel('b value')
axs[3].set_title('Bézier Output b(θ)')
axs[3].legend()

plt.tight_layout()
plt.show()
