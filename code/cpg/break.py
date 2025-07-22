import numpy as np
import matplotlib.pyplot as plt

# Parameters
N = 4                # Number of oscillators
T = 10.0             # Total simulation time (s)
dt = 0.01            # Time step (s)
time = np.arange(0, T, dt)

# Target oscillator and break time
i_target = 0         # index of oscillator to interrupt (0-based)
i_next = (i_target + 1) % N  # next oscillator
t_break = 5.0        # break time in seconds
k_break = int(t_break / dt)

# Natural frequencies (rad/s)
omega = np.array([1.0, 1.0, 1.0, 1.0])

# Coupling weight matrix: all weights = 0.5
W = np.full((N, N), 0.5)

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

# wrap to [0,2π)
theta_mod = np.mod(theta, 2*np.pi)

# 2. Manual phase jump at break: target -> stance (π/2), next -> swing (0)
theta_mod2 = theta_mod.copy()
theta_mod2[k_break:, i_target] = np.pi/2
theta_mod2[k_break:, i_next] = 0.0

# 3. Generate xi based on modified θ (1 for swing, -0.1 for stance)
xi_mod = np.where(theta_mod2 < np.pi/2, 1.0, -0.1)

# 4. Accelerated p2 response based on xi_mod
a = 20.0  
p2 = np.zeros((len(time), N))
p2_dot = np.zeros((len(time), N))
for k in range(len(time) - 1):
    for i in range(N):
        ddot = a * (a/4 * (xi_mod[k, i] - p2[k, i]) - p2_dot[k, i])
        p2_dot[k+1, i] = p2_dot[k, i] + ddot * dt
        p2[k+1, i] = p2[k, i] + p2_dot[k, i] * dt

# 5. Compute full b to get break height on modified phase
def b_of_s(s, p0, p2_val, p3):
    return ((1-s)**4 + 4*(1-s)**3 * s) * p0 + \
           6*(1-s)**2 * s**2 * p2_val + \
           (4*(1-s) * s**3 + s**4) * p3

p0_sw, p3_sw = 0.0, 0.0
p0_st_default, p3_st = 0.0, 0.0

b_full = np.zeros_like(theta_mod2)
for i in range(N):
    mask_sw = theta_mod2[:, i] < np.pi/2
    s_sw = (2/np.pi) * theta_mod2[:, i]
    s_st = (2/(3*np.pi)) * theta_mod2[:, i] - 1/3
    b_full[mask_sw, i] = b_of_s(s_sw[mask_sw], p0_sw, p2[mask_sw, i], p3_sw)
    b_full[~mask_sw, i] = b_of_s(s_st[~mask_sw], p0_st_default, p2[~mask_sw, i], p3_st)

# break height
b_break = b_full[k_break, i_target]

# per-oscillator stance p0 array
p0_st_array = np.full(N, p0_st_default)
p0_st_array[i_target] = b_break

# 6. Compute interrupted b (using modified phase & p0_st_array)
b_mod = np.zeros_like(theta_mod2)
for i in range(N):
    mask_sw = theta_mod2[:, i] < np.pi/2
    if i == i_target:
        mask_sw[k_break:] = False  # force stance after break
    s_sw = (2/np.pi) * theta_mod2[:, i]
    s_st = (2/(3*np.pi)) * theta_mod2[:, i] - 1/3
    b_mod[mask_sw, i] = b_of_s(s_sw[mask_sw], p0_sw, p2[mask_sw, i], p3_sw)
    b_mod[~mask_sw, i] = b_of_s(s_st[~mask_sw], p0_st_array[i], p2[~mask_sw, i], p3_st)

# 7. Plot four subplots
fig, axs = plt.subplots(4, 1, figsize=(8, 12), sharex=True)

# θ evolution
for i in range(N):
    axs[0].plot(time, theta_mod2[:, i], label=f'θ_{i+1}')
axs[0].axvline(t_break, color='k', linestyle='--', linewidth=1)
axs[0].set_ylabel('θ (rad)')
axs[0].set_title('Oscillator Phase Evolution θ (Modified)')
axs[0].legend()

# xi signal
for i in range(N):
    axs[1].plot(time, xi_mod[:, i], label=f'ξ_{i+1}')
axs[1].axvline(t_break, color='k', linestyle='--', linewidth=1)
axs[1].set_ylabel('ξ value')
axs[1].set_title('xi Signal (Modified Phase)')
axs[1].legend()

# p2 response
for i in range(N):
    axs[2].plot(time, p2[:, i], label=f'p2_{i+1}')
axs[2].axvline(t_break, color='k', linestyle='--', linewidth=1)
axs[2].set_ylabel('p2 value')
axs[2].set_title('Accelerated p2 Response')
axs[2].legend()

# interrupted b output
for i in range(N):
    axs[3].plot(time, b_mod[:, i], label=f'b_{i+1}')
axs[3].axvline(t_break, color='k', linestyle='--', linewidth=1)
axs[3].set_xlabel('Time (s)')
axs[3].set_ylabel('b value')
axs[3].set_title('Interrupted Bézier Output b(θ)')
axs[3].legend()

plt.tight_layout()
plt.show()
