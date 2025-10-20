# Simulation Project

## Overview
This project is composed of three main parts:

1. **Simulation Core** – implemented in `include/simulation.hpp`, `src/simulation.cpp`, `src/main.cpp`
2. **Unit Tests** – implemented in `tests/tests.cpp` using the **Catch2** framework  
3. **Trajectory Visualisation / UI** – a **Godot Engine**-based executable for visualising simulation results

---

## Build Instructions

### Requirements
- **CMake** (>= 3.16)  
- **C++17** (or later) compiler (GCC, Clang, MSVC)  
- **Catch2** (for testing)  
- **Godot Engine** (for the visualisation component)

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