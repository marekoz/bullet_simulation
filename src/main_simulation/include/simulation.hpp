#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <cmath>
#include <iostream>
#include <vector>
#include <limits>
#include <entt/entt.hpp>

constexpr double ALMOST_ZERO = 1e-9; // For division by zero checks
constexpr double GRAVITY = 9.80665; // m/s^2
constexpr double AIR_DENSITY = 1.225; // kg/m^3
constexpr double DRAG_COEFFICIENT = 0.3; 

constexpr double PI = 3.14159265358979323846;
constexpr double BULLET_DIAMETER = 0.00762;
constexpr double CROSS_SECTIONAL_AREA = PI * (BULLET_DIAMETER * BULLET_DIAMETER) / 4.0;

constexpr double HIT_TOLERANCE = 0.01; // meters
constexpr size_t MAX_SIM_STEPS = 100000;
constexpr double RAD_90_DEGREES = M_PI / 2.0;
constexpr size_t MAX_ANGLE_SEARCH_ITERATIONS = 32;


enum sim_result
{
    HIT,
    TOO_HIGH,
    TOO_LOW,
    ERROR
};




// Entity components
struct Position
{
    double x, y, z; // meters
};

struct Velocity
{
    double dx, dy, dz; // meters/second
};

struct Mass
{
    double mass; // g
};


// Application cmd line parameters
struct Parameters
{
    Position muzzle_pos;
    Position target_pos;
    double initial_speed;
    double bullet_mass;
    double step_length;
    bool print_progress = false;
    bool print_trajectory = false;
    bool method_rk4 = true;
};


// Function return types

struct TrajectoryResult {
    sim_result result;
    double closest_distance;
    std::vector<Position> trajectory;
};

struct AngleSearchResult
{
    double best_angle_rad;
    bool hit_found;  
    double closest_distance;
    size_t iterations;  
};


// Function declarations

// Read command-line inputs, check for invalid parameters, and set up positions and parameters
bool handle_arguments(int argc, char *argv[], Parameters &parameters);

// Overload operator<< for sim_result prints
std::ostream &operator<<(std::ostream &os, sim_result result);

// Calculate distance between two positions
double calculate_distance(const Position &pos1, const Position &pos2);

// Calculate horizontal distance between two positions
double calculate_horizontal_distance(const Position &pos1, const Position &pos2);

// Calculate acceleration for given velocity and mass
Velocity calculate_acceleration(const Velocity &vel, const Mass &mass);

// Perform a simulation step using Euler's method
void sim_step_system_euler(entt::registry &registry, double delta_time);

// Perform a simulation step using the RK4 method
void sim_step_system(entt::registry &registry, double delta_time);


// Create and initialize a bullet entity
entt::entity create_bullet(entt::registry &registry, double horizontal_angle, double elevation_angle,
                           double initial_speed, const Position &muzzle_pos, double bullet_mass);

// Check if the bullet hit the target
bool check_target_hit(const Position &bullet_pos, const Position &target_pos);

// Check if the bullet hit the ground
bool check_ground_hit(const Position &bullet_pos);

// Check if the bullet passed the target horizontally
bool check_passed_target(const Position &bullet_pos, const Position &muzzle_pos, const Position &target_pos);

// Interpolation of the last simulation step to end up exactly at the target horizontal distance
TrajectoryResult last_step_interpolation(entt::registry &registry, entt::entity &bullet,
                                        const Position &prev_pos, const Velocity &prev_vel, 
                                        std::vector<Position> &traj, const Parameters &params);

// Simulation loop without creating bullet
TrajectoryResult simulate_trajectory_loop(entt::registry &registry, entt::entity &bullet, const Parameters &params);

// Simulate bullet trajectory of specific angle and return result, distance to target and vector of positions
TrajectoryResult simulate_bullet_trajectory(double horizontal_angle, double elevation_angle, const Parameters& params);

// Find the optimal elevation angle using binary search
AngleSearchResult find_optimal_angle(const Parameters &parameters);

#endif // SIMULATION_HPP;