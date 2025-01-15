# Satellite Attitude Control and Simulation

## Project Overview

This project consists of two primary objectives:

1. **PD Satellite Attitude Control System**: Implementing a Proportional-Derivative (PD) control system to stabilize the attitude of a satellite in orbit using reaction wheels.
2. **Satellite Simulation**: Simulating the satellite's behavior in orbit under the influence of external perturbations and disturbances, visualizing its orientation and stability over time.

The satellite's attitude control system is simulated with random perturbations occurring at intervals, which the system compensates for using reaction wheels. The entire simulation is visualized in 3D, including the satellite's cube structure, axes, and Earth.

## Features

- **Satellite Dynamics Simulation**: Simulates the attitude of a satellite over time using quaternion-based orientation representation.
- **Perturbations and Stability**: Random perturbations are introduced to simulate real-world disturbances, and the PD control system stabilizes the satellite.
- **3D Visualization**: Visualizes the satellite’s orientation, position, and axes in 3D, along with Earth’s surface.
- **Animation**: Animates the satellite in orbit with control dynamics applied to stabilize it.

## Requirements

- **Python 3.x**
- **NumPy**: For numerical operations.
- **SciPy**: For solving differential equations (ODE solver).
- **PyQuaternion**: For handling quaternion operations.
- **Matplotlib**: For 2D and 3D plotting, and animation.
- **mpl_toolkits.mplot3d**: For 3D plotting and animation.
  
You can install the necessary dependencies using pip:

```bash
pip install numpy scipy pyquaternion matplotlib

Explanation of Key Components
1. PD Satellite Attitude Control System
The PD controller works by generating control torques to counteract angular velocity errors and help stabilize the satellite. The gains for proportional (Kp) and derivative (Kd) control are adjustable parameters.

2. Satellite Dynamics
The satellite’s dynamics are described using quaternions for orientation and angular velocity. The satellite's orientation is updated over time using the quaternion derivative, and the control system adjusts the torque applied by the reaction wheels to maintain stability.

3. Perturbations
To simulate real-world conditions, random perturbations (disturbances) are added to the satellite’s system at regular intervals. The control system adjusts the satellite's attitude to stabilize it despite these disturbances.

4. 3D Visualization and Animation
The satellite's position and orientation are visualized in 3D, with axes indicating the satellite's orientation in space. The satellite orbits Earth, and its attitude is animated over time. The perturbations and stabilization process can be observed through the animation.

Project Extensions
Advanced Control: Implement more advanced control algorithms such as LQR or Kalman Filtering for satellite attitude stabilization.
Realistic Environmental Model: Introduce more realistic environmental models, including gravitational influences, drag, and external forces.
Orbit Mechanics: Extend the simulation to include orbital mechanics, modeling the satellite's movement in a realistic orbit around Earth.
