#include "simulation.hpp"
#include <algorithm>
#include <fstream>
#include <cfloat>

using namespace std;

int main(int argc, char *argv[])
{
    Parameters params;
    if (!handle_arguments(argc, argv, params))
    {
        return 1;
    }

    AngleSearchResult result = find_optimal_angle(params);

    cout << "Simulation result:" << endl;
    if (result.hit_found)
    {
        cout << "  Solution found in " << result.iterations << " iterations!" << endl;
        cout << "  Optimal elevation angle: " << (result.best_angle_rad * 180.0 / M_PI) << " degrees (" << result.best_angle_rad << " radians)" << endl;
        cout << "  Closest distance to center of target: " << result.closest_distance << " meters" << endl;
    }
    else
    {
        cout << "  No solution found in " << result.iterations << " iterations" << endl;
        cout << "  The target may be too far or the bullet too slow." << endl;
        cout << "  Best elevation angle: " << (result.best_angle_rad * 180.0 / M_PI) << " degrees (" << result.best_angle_rad << " radians)" << endl;
        cout << "  Closest distance to center of target: " << result.closest_distance << " meters" << endl;
    }

    // Trajectory print of the found solution (only if requested with -t flag)
    if (params.print_trajectory)
    {
        std::cout << "Trajectory: " << std::endl;
        double h_angle = atan2(params.target_pos.y - params.muzzle_pos.y, params.target_pos.x - params.muzzle_pos.x);
        params.print_progress = false;
        TrajectoryResult traj_result = simulate_bullet_trajectory(h_angle, result.best_angle_rad, params);
        for (Position& pos : traj_result.trajectory)
        {
            std::cout << pos.x << ", " << pos.y << ", " << pos.z << "\n";
        }
        std::cout << std::endl;
    }
    return 0;
}