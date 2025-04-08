import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
dt = 0.01  # Time step (s)
T = 10     # Total time (s)
N = int(T / dt)

# PID controller parameters
Kp = 2.0
Ki = 0.5
Kd = 1.0

# Desired target position
target_position = 10.0

# Initialization
position = 0.0
velocity = 0.0
integral = 0.0
prev_error = target_position - position

# Data for plotting
positions = []
times = []

# PID control loop
for i in range(N):
    t = i * dt
    error = target_position - position
    integral += error * dt
    derivative = (error - prev_error) / dt
    prev_error = error

    # PID output (acceleration)
    control = Kp * error + Ki * integral + Kd * derivative

    # Update dynamics (simple physics)
    acceleration = control
    velocity += acceleration * dt
    position += velocity * dt

    # Store for plotting
    positions.append(position)
    times.append(t)

# Plotting the result
plt.figure(figsize=(10, 5))
plt.plot(times, positions, label="Robot Position")
plt.axhline(y=target_position, color='r', linestyle='--', label="Target Position")
plt.title("1D Robot PID Position Control")
plt.xlabel("Time (s)")
plt.ylabel("Position")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("pid_sim_plot.png")
plt.show()
