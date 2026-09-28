# Proportional-Navigation-Projeto-Final-de-Curso

# Proportional Navigation PFC

## Overview

This project implements a 3D Proportional Navigation guidance algorithm for interceptor-target engagement simulation.

The model is developed in C++, using numerical integration to simulate the relative dynamics and calculate the guidance acceleration command. Simulation results are exported for post-processing and visualization using MATLAB.

---

## Features

- 3D Proportional Navigation guidance law
- Interceptor-target engagement simulation
- Runge-Kutta 4th order numerical integration
- Line-of-sight (LOS) and relative motion analysis
- CSV data export
- MATLAB trajectory visualization

---

## Project Structure

```
ProportionalNavigationPFC/
│
├── C++/
│   ├── Source files
│   └── Visual Studio project
│
├── MATLAB/
│   └── Plotting and analysis scripts
│
└── README.md
```

---

## Requirements

### C++
- Visual Studio 2022
- C++17 compatible compiler

### MATLAB
- MATLAB R2020 or newer

---

## Usage

1. Build and run the C++ project.
2. The simulation generates CSV files containing the engagement data.
3. Use the MATLAB scripts to visualize trajectories and analyze results.

---

## Applications

This project provides a simulation framework for studying:

- Guidance and navigation algorithms
- Aerospace control systems
- Autonomous interception problems
- Missile guidance concepts

---

## Observation

The MATLAB script comparison.m contains an independent implementation of the same guidance algorithm developed in C++, allowing comparison and verification between both implementations.

---

## Author

Aerospace Engineering Project
Gustavo Rickli
