# Adding code to plot both x (swing phase) and y (stance phase) together, as per the previous explanation.

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# Constants
alpha = 1.0
beta = 1.0
mu = 0.1  # Control parameter
omega_stance = 2.0
omega_swing = 2.5
b = 1.0
feedback_strength = 0.5  # Feedback strength (used in feedback function)
F = 1.0  # Force or stimulus for fast transitions

# Define the system of equations (modified Hopf oscillator with feedback ignoring y_j)
def hopf_system_with_feedback(state, t, feedback_period):
    x, y = state
    r = np.sqrt(x**2 + y**2)
    
    # Calculate phase-dependent frequency for stance and swing phases
    omega = (omega_stance / (np.exp(-b * y) + 1)) + (omega_swing / (np.exp(b * y) + 1))
    
    # Calculate feedback u_i based on the formula
    if (t % feedback_period) < 1:  # Fast transition phase (1 second on)
        u = -np.sign(y) * F  # Feedback for fast transitions
    elif (t % feedback_period) < 2:  # Stop transition phase (following fast transition)
        u = -omega * x  # Feedback for stop transition (ignoring y_j)
    else:
        u = 0  # No feedback otherwise
    
    # System of equations with feedback
    dxdt = alpha * (mu - r**2) * x - omega * y 
    dydt = beta * (mu - r**2) * y + omega * x + u
    
    return [dxdt, dydt]

# Set the time range to 50 seconds
t_50s = np.linspace(0, 50, 1000)

# Varying feedback period between 5 and 10 seconds (linearly interpolated)
feedback_periods = np.linspace(5, 10, len(t_50s))

# Solve the system of ODEs with varying square wave feedback periods
solution_with_feedback = np.array([odeint(hopf_system_with_feedback, [0.5, 0.5], [time, time+1], args=(feedback_periods[i],))[1] for i, time in enumerate(t_50s)])

# Plot the results
plt.figure(figsize=(12, 6))

# Plot x (phase component) and y (stance phase) together
plt.plot(t_50s, solution_with_feedback[:, 0], label='x (Phase Component 1 - Swing Phase)', color='blue')
plt.plot(t_50s, solution_with_feedback[:, 1], label='y (Phase Component 2 - Stance Phase)', color='green')

# Add labels and title
plt.title('Hopf Oscillator with Feedback Based on Phase (Ignoring y_j)')
plt.xlabel('Time (s)')
plt.ylabel('Phase Components (x - Swing, y - Stance)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
