#include "simulation.hpp"
#include <algorithm>
#include <fstream>
#include <cfloat>
#include <iomanip>

std::ostream &operator<<(std::ostream &os, sim_result result)
{
    switch (result)
    {
    case HIT:
        return os << "HIT";
    case TOO_HIGH:
        return os << "TOO HIGH";
    case TOO_LOW:
        return os << "TOO LOW";
    default:
        return os << "ERROR";
    }
    return os;
}

// Function to calculate distance between two positions
double calculate_distance(const Position &pos1, const Position &pos2)
{
    double dx = pos1.x - pos2.x;
    double dy = pos1.y - pos2.y;
    double dz = pos1.z - pos2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double calculate_horizontal_distance(const Position &pos1, const Position &pos2)
{
    double dx = pos1.x - pos2.x;
    double dy = pos1.y - pos2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Calculate acceleration for given velocity and mass
// Simplified (no wind, constant gravity,)
Velocity calculate_acceleration(const Velocity &vel, const Mass &mass)
{
    double speed = std::hypot(vel.dx, vel.dy, vel.dz); // sqrt(vel.dx * vel.dx + vel.dy * vel.dy + vel.dz * vel.dz);

    Velocity acc = {0.0, 0.0, -GRAVITY};

    if (speed > ALMOST_ZERO)
    {
        double drag_force = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * CROSS_SECTIONAL_AREA * speed * speed;
        const double drag_acc_mag = drag_force / mass.mass;
        acc.dx -= (vel.dx / speed) * drag_acc_mag;
        acc.dy -= (vel.dy / speed) * drag_acc_mag;
        acc.dz -= (vel.dz / speed) * drag_acc_mag;
    }
    return acc;
}

void sim_step_system_euler(entt::registry &registry, double delta_time)
{
    auto view = registry.view<Position, Velocity, Mass>();

    // Only 1 entity (the bullet)
    for (auto entity : view)
    {
        auto &pos = view.get<Position>(entity);
        auto &vel = view.get<Velocity>(entity);
        auto &mass = view.get<Mass>(entity);

        // Calculate acceleration
        Velocity acc = calculate_acceleration(vel, mass);

        // Update velocity
        vel.dx += acc.dx * delta_time;
        vel.dy += acc.dy * delta_time;
        vel.dz += acc.dz * delta_time;

        // Update position
        pos.x += vel.dx * delta_time;
        pos.y += vel.dy * delta_time;
        pos.z += vel.dz * delta_time;
    }
}

// Physics simulation system using RK4 method
void sim_step_system(entt::registry &registry, double delta_time)
{
    auto view = registry.view<Position, Velocity, Mass>();

    // Only 1 entity (the bullet)
    for (auto entity : view)
    {
        auto &pos = view.get<Position>(entity);
        auto &vel = view.get<Velocity>(entity);
        auto &mass = view.get<Mass>(entity);

        // Store current state
        Position pos0 = pos;
        Velocity vel0 = vel;

        // RK4 integration
        // k1 = f(t, y)
        Velocity k1_acc = calculate_acceleration(vel0, mass);
        Velocity k1_vel = vel0;

        // k2 = f(t + dt/2, y + k1*dt/2)
        Velocity vel_k2 = {
            vel0.dx + k1_acc.dx * delta_time / 2,
            vel0.dy + k1_acc.dy * delta_time / 2,
            vel0.dz + k1_acc.dz * delta_time / 2};
        Velocity k2_acc = calculate_acceleration(vel_k2, mass);
        Velocity k2_vel = vel_k2;

        // k3 = f(t + dt/2, y + k2*dt/2)
        Velocity vel_k3 = {
            vel0.dx + k2_acc.dx * delta_time / 2,
            vel0.dy + k2_acc.dy * delta_time / 2,
            vel0.dz + k2_acc.dz * delta_time / 2};
        Velocity k3_acc = calculate_acceleration(vel_k3, mass);
        Velocity k3_vel = vel_k3;

        // k4 = f(t + dt, y + k3*dt)
        Velocity vel_k4 = {
            vel0.dx + k3_acc.dx * delta_time,
            vel0.dy + k3_acc.dy * delta_time,
            vel0.dz + k3_acc.dz * delta_time};
        Velocity k4_acc = calculate_acceleration(vel_k4, mass);
        Velocity k4_vel = vel_k4;

        // RK4 weighted average
        vel.dx += (delta_time / 6.0) * (k1_acc.dx + 2 * k2_acc.dx + 2 * k3_acc.dx + k4_acc.dx);
        vel.dy += (delta_time / 6.0) * (k1_acc.dy + 2 * k2_acc.dy + 2 * k3_acc.dy + k4_acc.dy);
        vel.dz += (delta_time / 6.0) * (k1_acc.dz + 2 * k2_acc.dz + 2 * k3_acc.dz + k4_acc.dz);

        pos.x += (delta_time / 6.0) * (k1_vel.dx + 2 * k2_vel.dx + 2 * k3_vel.dx + k4_vel.dx);
        pos.y += (delta_time / 6.0) * (k1_vel.dy + 2 * k2_vel.dy + 2 * k3_vel.dy + k4_vel.dy);
        pos.z += (delta_time / 6.0) * (k1_vel.dz + 2 * k2_vel.dz + 2 * k3_vel.dz + k4_vel.dz);
    }
}

// Create and initialize bullet entity
entt::entity create_bullet(entt::registry &registry, double horizontal_angle, double elevation_angle,
                           double initial_speed, const Position &muzzle_pos, double bullet_mass)
{
    auto bullet = registry.create();

    // Calculate initial velocity components using spherical coordinates
    double initial_vel_x = initial_speed * cos(elevation_angle) * cos(horizontal_angle);
    double initial_vel_y = initial_speed * cos(elevation_angle) * sin(horizontal_angle);
    double initial_vel_z = initial_speed * sin(elevation_angle);

    // Add components to bullet 
    registry.emplace<Position>(bullet, muzzle_pos.x, muzzle_pos.y, muzzle_pos.z);
    registry.emplace<Velocity>(bullet, initial_vel_x, initial_vel_y, initial_vel_z);
    registry.emplace<Mass>(bullet, bullet_mass / 1000.0); // grams to kg
    return bullet;
}

// Check if bullet hit the target within tolerance
bool check_target_hit(const Position &bullet_pos, const Position &target_pos)
{
    double distance = calculate_distance(bullet_pos, target_pos);
    return distance <= HIT_TOLERANCE;
}

// Check if bullet passed target horizontally
bool check_passed_target(const Position &bullet_pos, const Position &muzzle_pos, const Position &target_pos)
{
    double dx_bullet = bullet_pos.x - muzzle_pos.x;
    double dy_bullet = bullet_pos.y - muzzle_pos.y;
    double dx_target = target_pos.x - muzzle_pos.x;
    double dy_target = target_pos.y - muzzle_pos.y;

    double bullet_distance_sq = dx_bullet * dx_bullet + dy_bullet * dy_bullet;
    double target_distance_sq = dx_target * dx_target + dy_target * dy_target;

    return bullet_distance_sq > target_distance_sq;
}

// Interpolation of the last simulation step to end up exactly at the target horizontal distance
TrajectoryResult last_step_interpolation(entt::registry &registry, entt::entity &bullet,
                                         const Position &prev_pos, const Velocity &prev_vel,
                                         std::vector<Position> &traj, const Parameters &params)
{

    // Calculate the exact time when bullet crosses the target's horizontal distance
    // Using linear interpolation: t = (target_distance - prev_distance) / (curr_distance - prev_distance) * step_length
    double prev_h_distance = calculate_horizontal_distance(prev_pos, params.muzzle_pos);
    Position &bullet_pos = registry.get<Position>(bullet);
    double curr_h_distance = calculate_horizontal_distance(bullet_pos, params.muzzle_pos);
    double target_h_distance = calculate_horizontal_distance(params.target_pos, params.muzzle_pos);
    double interp_factor = (target_h_distance - prev_h_distance) / (curr_h_distance - prev_h_distance);
    double interp_step_length = interp_factor * params.step_length;

    if (params.print_progress)
        std::cout << "  Bullet overshot target horizontal position. Stepping back and interpolating to reach the exact position using step_length = "
                  << interp_step_length << std::endl;

    // Restore previous state
    registry.replace<Position>(bullet, prev_pos);
    registry.replace<Velocity>(bullet, prev_vel);

    if (!params.method_rk4)
        sim_step_system_euler(registry, interp_step_length);
    else
        sim_step_system(registry, interp_step_length);

    auto &interpolated_bullet_pos = registry.get<Position>(bullet);
    double distance = calculate_distance(interpolated_bullet_pos, params.target_pos);
    traj.push_back(interpolated_bullet_pos);
    if (params.print_progress)
        std::cout << "    Interpolated height: " << interpolated_bullet_pos.z << " (target: " << params.target_pos.z << ")" << std::endl;

    if (abs(interpolated_bullet_pos.z - params.target_pos.z) < HIT_TOLERANCE)
    {
        if (params.print_progress)
            std::cout << "    Bullet hit target at interpolated crossing with tolerance of " << HIT_TOLERANCE << " meters" << std::endl;
        return {HIT, distance, traj};
    }
    else if (interpolated_bullet_pos.z < params.target_pos.z)
    {
        if (params.print_progress)
            std::cout << "    Bullet is too low at interpolated crossing " << std::endl;
        return {TOO_LOW, distance, traj};
    }
    else
    {
        if (params.print_progress)
            std::cout << "  Bullet is too high at interpolated crossing" << std::endl;
        return {TOO_HIGH, distance, traj};
    }
}

// Simulation loop without creating bullet
TrajectoryResult simulate_trajectory_loop(entt::registry &registry, entt::entity &bullet, const Parameters &params)
{
    double time = 0;
    Position prev_pos = params.muzzle_pos;
    std::vector<Position> trajectory;
    trajectory.push_back(params.muzzle_pos);
    size_t i = 0;
    while (i < MAX_SIM_STEPS)
    {
        i++;
        prev_pos = registry.get<Position>(bullet);
        auto prev_vel = registry.get<Velocity>(bullet);

        if (!params.method_rk4)
            sim_step_system_euler(registry, params.step_length);
        else
            sim_step_system(registry, params.step_length);

        auto &bullet_pos = registry.get<Position>(bullet);
        if (check_target_hit(bullet_pos, params.target_pos))
        {
            double distance = calculate_distance(bullet_pos, params.target_pos);
            return {HIT, distance, trajectory};
        }
        else if (check_passed_target(bullet_pos, params.muzzle_pos, params.target_pos)) // Check if bullet passed target horizontally
        {
            // Determine if bullet was too high or too low when passing target
            if (params.print_progress)
                std::cout << "  Bullet passed target horizontal position at time: " << time << " (step = " << time / params.step_length << ")" << std::endl;
            return last_step_interpolation(registry, bullet, prev_pos, prev_vel, trajectory, params);
        }
        else if (bullet_pos.z < 0.0 - HIT_TOLERANCE) // Check if bullet hit ground
        {
            double distance = calculate_distance(bullet_pos, params.target_pos);
            if (params.print_progress)
            {
                double horizontal_progress = calculate_horizontal_distance(bullet_pos, params.muzzle_pos);
                double target_horizontal_distance = calculate_horizontal_distance(params.target_pos, params.muzzle_pos);
                std::cout << "  Bullet hit ground at horizontal distance " << horizontal_progress
                          << " (target distance: " << target_horizontal_distance << ")" << std::endl;
            }
            return {TOO_LOW, distance, trajectory};
        }
        trajectory.push_back(bullet_pos);
        time += params.step_length;
    }
    return {ERROR, -1, trajectory};
}

// Simulate bullet trajectory of specific angle and return result, distance to target and vector of positions
TrajectoryResult simulate_bullet_trajectory(double horizontal_angle, double elevation_angle, const Parameters &params)
{
    entt::registry registry;
    entt::entity bullet = create_bullet(registry, horizontal_angle, elevation_angle, params.initial_speed, params.muzzle_pos, params.bullet_mass);
    TrajectoryResult result = simulate_trajectory_loop(registry, bullet, params);
    return result;
}

// Function to find the optimal elevation angle using binary search
AngleSearchResult find_optimal_angle(const Parameters &parameters)
{
    const Position &muzzle_pos = parameters.muzzle_pos;
    const Position &target_pos = parameters.target_pos;

    // Edge case when target is directly above or below muzzle
    if (muzzle_pos.x == target_pos.x && muzzle_pos.y == target_pos.y)
    {

        if (muzzle_pos.z < target_pos.z)
        {
            TrajectoryResult result = simulate_bullet_trajectory(0.0, RAD_90_DEGREES, parameters);
            return {RAD_90_DEGREES, true}; // The angle is 90 but hard to confirm the hit due to simulation logic with horizontal distance check
        }
        else if (muzzle_pos.z > target_pos.z)
        {
            TrajectoryResult result = simulate_bullet_trajectory(0.0, -RAD_90_DEGREES, parameters);
            return {-RAD_90_DEGREES, true};
        }
        std::cerr << "Error: Muzzle position and target position cannot be the same." << std::endl;
        return {0.0, false};
    }
    

    // Binary search for optimal elevation angle
    double min_angle = -RAD_90_DEGREES; // -90 degrees
    double max_angle = RAD_90_DEGREES;  // 90 degrees
    double h_angle = atan2(target_pos.y - muzzle_pos.y, target_pos.x - muzzle_pos.x); // (0 to 2pi)
    size_t iteration = 0;
    double best_angle = -1.0;
    double closest_distance = DBL_MAX;
    bool found_solution = false;
    while (iteration < MAX_ANGLE_SEARCH_ITERATIONS)
    {

        iteration++;
        double mid_elevation = (min_angle + max_angle) / 2.0;
        if (parameters.print_progress)
            std::cout << "Iteration " << iteration << ": Testing elevation " << (mid_elevation * 180.0 / M_PI)
                      << " degrees," << " (range: " << (min_angle * 180.0 / M_PI)
                      << "° to " << (max_angle * 180.0 / M_PI) << "°)" << std::endl;

        // Test current elevation angle
        TrajectoryResult result = simulate_bullet_trajectory(h_angle, mid_elevation, parameters);

        if (parameters.print_progress)
            std::cout << " result: " << result.result << std::endl;

        if (result.closest_distance < closest_distance)
        {
            closest_distance = result.closest_distance;
            best_angle = mid_elevation; // 45 degrees mostly likely if too far
        }

        if (result.result == HIT)
        {
            found_solution = true;
            best_angle = mid_elevation;
            break;
        }
        else if (result.result == TOO_HIGH)
        {
            max_angle = mid_elevation;
        }
        else if (result.result == TOO_LOW)
        {
            min_angle = mid_elevation;
        }
    }

    return {best_angle, found_solution, closest_distance, iteration};
}

// Read command-line inputs, check for invalid parameters, and set up positions and parameters
bool handle_arguments(int argc, char *argv[],
                      Parameters &params)
{
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        // Optional flags
        if (arg == "-p" || arg == "--print-progress")
        {
            params.print_progress = true;
            continue;
        }
        if (arg == "-t" || arg == "--print-trajectory")
        {
            params.print_trajectory = true;
            continue;
        }
        if (arg == "-e" || arg == "--euler")
        {
            params.method_rk4 = false;
            continue;
        }
        if (arg == "-h" || arg == "--help")
        {
            std::cout << "Usage: " << argv[0]
                      << " [options] <shooter_x> <shooter_y> <shooter_z> "
                      << "<target_x> <target_y> <target_z> "
                      << "<initial_speed m/s> <bullet_mass g> <step_length s>\n\n"
                      << "Options:\n"
                      << "  -p, --print-progress     Print progress of the angle search algorithm\n"
                      << "  -t, --print-trajectory   Print full bullet trajectory (x,y,z)\n"
                      << "  -e, --euler              Use faster Euler method instead of RK4\n"
                      << "  -h, --help               Show this help message"
                      << std::endl;
            return false;
        }
        positional.push_back(arg);
    }

    // Expect exactly 9 positional arguments
    if (positional.size() < 9)
    {
        std::cerr << "Error: not enough arguments.\n";
        std::cerr << "Try '" << argv[0] << " --help' for usage information." << std::endl;
        return false;
    }

    if (positional.size() > 9)
    {
        std::cerr << "Warning: too many arguments, ignoring extra values." << std::endl;
    }

    bool all_numbers = true;
    for (const std::string &s : positional)
    {
        try
        {
            std::stod(s);
        }
        catch (const std::invalid_argument &)
        {
            all_numbers = false;
            break;
        }
    }
    if (!all_numbers)
    {
        std::cerr << "Error: all positional arguments must be valid numbers." << std::endl;
        return false;
    }

    // Parse positional values

    params.muzzle_pos = {
        std::stod(positional[0]),
        std::stod(positional[1]),
        std::stod(positional[2])};
    params.target_pos = {
        std::stod(positional[3]),
        std::stod(positional[4]),
        std::stod(positional[5])};
    params.initial_speed = stod(positional[6]);
    params.bullet_mass = stod(positional[7]);
    params.step_length = stod(positional[8]);

    if (params.muzzle_pos.x == params.target_pos.x && params.muzzle_pos.y == params.target_pos.y && params.muzzle_pos.z == params.target_pos.z)
    {
        std::cerr << "Error: Muzzle position and target position cannot be the same." << std::endl;
        return false;
    }
    if (params.muzzle_pos.z < 0.0)
    {
        std::cerr << "Error: Muzzle position z cannot be below ground level (0.0)." << std::endl;
        return false;
    }
    if (params.target_pos.z < 0.0)
    {
        std::cerr << "Error: Target position z cannot be below ground level (0.0)." << std::endl;
        return false;
    }
    if (params.initial_speed <= 0.0)
    {
        std::cerr << "Error: Initial speed must be greater than 0." << std::endl;
        return false;
    }
    if (params.bullet_mass <= 0.0)
    {
        std::cerr << "Error: Bullet mass must be greater than 0." << std::endl;
        return false;
    }

    if (params.step_length <= 0.0)
    {
        std::cerr << "Error: Step length must be greater than 0." << std::endl;
        return false;
    }
    if (params.step_length > 1.0)
    {
        std::cerr << "Warning: Step length is very large, consider using a smaller value for better accuracy." << std::endl;
    }
    if (params.step_length < 0.0001)
    {
        std::cerr << "Warning: Step length is very small, simulation may be slow." << std::endl;
    }

    return true;
}