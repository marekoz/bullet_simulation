# Bullet Trajectory Simulation

This project is a simulation of a bullet trajectory towards a target. 
Its main purpose is to **search for the correct vertical angle** that allows the bullet to hit the target. 
The simulation calculates this angle based on the **muzzle position**, **target position**, **initial speed**, **bullet mass**, and **simulation step length**. 
It uses **Euler** and **RK4** methods and  **binary search** to determine the required angle.  
The project was developed on and built for Linux Mint 22.

## Running This Application

The angle search simulation can be run using the executables in the `builds` directory. 
There are two ways to run the program:

### 1. Command Line

The simulation can be run from the **command line**. The program can optionally **print the progress of the angle search** and **display the full bullet trajectory** corresponding to the optimal vertical angle.

```bash
./simulation [options] <shooter_x> <shooter_y> <shooter_z> <target_x> <target_y> <target_z> <initial_speed m/s> <bullet_mass g> <step_length s>

Options:
  -p, --print-progress     Print progress of the angle search algorithm
  -t, --print-trajectory   Print full bullet trajectory (x,y,z)
  -e, --euler              Use faster Euler method instead of RK4
  -h, --help               Show this help message
```

### 2. 3D Application
Alternatively, you can run a separate 3D application built with Godot Engine, which also visualizes the bullet trajectory. The executable for this application is located at: `builds/ballistics_visualisation.x86_64`

---

## Source Files Overview
The **Bullet Simulation** project is composed of three main modules:

1. **Simulation**
- Located in: `src/main_simulation`
   - Files:
     - `include/simulation.hpp`
     - `src/simulation.cpp`
     - `src/main.cpp`
   - Handles all **physics calculations**, **trajectory prediction**, and **simulation logic**.
   - Built with **EnTT**.

2. **Unit Tests**
    - Located in: `src/main_simulation`
        -  File: `tests/tests.cpp`
   - Built with **Catch2**.

3. **Trajectory Visualisation (UI)**
   - Located in: `src/ballistics_visualisation`
   - Used for **3D visualisation** of bullet trajectories and simulation results.
   - Compiled outputs are located in the `builds/` directory:
     - `ballistics_visualisation.pck`
     - `ballistics_visualisation.x86_64`
    - Implemented using **Godot Engine 4.5**.


## Build Instructions
### Requirements
- **CMake** (>= 3.24)  
- **C++** compiler 
- **Catch2** (for testing)  

---
### Build Steps

```bash
# create and enter the build directory
cd build

# generate build files
cmake ..

# compile all targets
cmake --build .
```
### 3D Application 

The 3D application project can be opened, edited, and exported using **Godot 4.5**.  
To access the project, open the `project.godot` file located in the `src/ballistics_visualisation/` directory.


## Directory Structure

- **builds/**
    - `ballistics_visualisation.pck` — Packed Godot resources for the visualisation.
    - `ballistics_visualisation.x86_64` — Linux executable for the visualisation build.
    - `simulation` — Runs the main angle search simulation
    - `tests` — Runs all of the unit tests.
- **src/**
    - **ballistics_visualisation/** — Godot 4.5 project for visualising bullet trajectories.
      - *(Contains `project.godot` and related Godot scene, script, and asset files.)*
    - **main_simulation/**
      - **build/** — Local build folder for compiled binaries or CMake cache.
      - **include/**
        - **catch2/** — Catch2 unit testing framework headers.
        - **entt/** — EnTT ECS (Entity Component System) library headers.
        - `simulation.hpp` — Header file containing `constexpr` definitions, function declarations, structs, and other related components.
      - **src/**
        - `main.cpp` — Entry point for the main simulation executable.
        - `simulation.cpp` — Implements simulation logic (physics, trajectory, etc.).
      - **tests/**
        - `tests.cpp` — Unit tests for validating the simulation using Catch2.
      - `CMakeLists.txt` — CMake configuration for building the main simulation project.
---