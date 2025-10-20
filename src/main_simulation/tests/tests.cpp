#include "catch2/catch.hpp"  
#include "simulation.hpp"
#include "entt/entt.hpp"



// handle_arguments tests

TEST_CASE("handle_arguments - Valid arguments", "[handle_arguments]") {
    int argc = 10;
    char* argv[] = {
        (char*)"simulation",
        (char*)"0.0", (char*)"0.0", (char*)"0.0",  // Muzzle position
        (char*)"100.0", (char*)"0.0", (char*)"0.0", // Target position
        (char*)"300.0", // Initial speed
        (char*)"0.01",  // Bullet mass
        (char*)"0.1"    // Step length
    };

    Parameters params;
    bool result = handle_arguments(argc, argv, params);
    Position &muzzle_pos = params.muzzle_pos;
    Position &target_pos = params.target_pos;
    REQUIRE(result == true);
    REQUIRE(muzzle_pos.x == Approx(0.0));
    REQUIRE(muzzle_pos.y == Approx(0.0));
    REQUIRE(muzzle_pos.z == Approx(0.0));
    REQUIRE(target_pos.x == Approx(100.0));
    REQUIRE(target_pos.y == Approx(0.0));
    REQUIRE(target_pos.z == Approx(0.0));
    REQUIRE(params.initial_speed == Approx(300.0));
    REQUIRE(params.bullet_mass == Approx(0.01));
    REQUIRE(params.step_length == Approx(0.1));
}

TEST_CASE("handle_arguments - Invalid number of arguments", "[handle_arguments]") {
    int argc = 5; // Too few arguments
    char* argv[] = {
        (char*)"simulation",
        (char*)"0.0", (char*)"0.0", (char*)"0.0",  // Muzzle position
        (char*)"100.0" // Target position (incomplete)
    };

    Parameters params;

    bool result = handle_arguments(argc, argv, params);

    REQUIRE(result == false);
}


TEST_CASE("handle_arguments - Non-numeric argument", "[handle_arguments]") {
    int argc = 10;
    char* argv[] = {
        (char*)"simulation",
        (char*)"0.0", (char*)"0.0", (char*)"0.0",  // Muzzle position
        (char*)"100.0", (char*)"abc", (char*)"0.0", // Target position with non-numeric value
        (char*)"300.0", // Initial speed
        (char*)"0.01",  // Bullet mass
        (char*)"0.1"    // Step length
    };

    Parameters params;

    bool result = handle_arguments(argc, argv, params);

    REQUIRE(result == false);
}

TEST_CASE("handle_arguments - Muzzle and target positions are the same", "[handle_arguments]") {
    int argc = 10;
    char* argv[] = {
        (char*)"simulation",
        (char*)"0.0", (char*)"0.0", (char*)"0.0",  // Muzzle position
        (char*)"0.0", (char*)"0.0", (char*)"0.0", // Target position (same as muzzle)
        (char*)"300.0", // Initial speed
        (char*)"0.01",  // Bullet mass
        (char*)"0.1"    // Step length
    };

    Parameters params;

    bool result = handle_arguments(argc, argv, params);

    REQUIRE(result == false);
}

TEST_CASE("handle_arguments - Negative initial speed", "[handle_arguments]") {
    int argc = 10;
    char* argv[] = {
        (char*)"simulation",
        (char*)"0.0", (char*)"0.0", (char*)"0.0",  // Muzzle position
        (char*)"100.0", (char*)"0.0", (char*)"0.0", // Target position
        (char*)"-300.0", // Negative initial speed
        (char*)"0.01",  // Bullet mass
        (char*)"0.1"    // Step length
    };

    Parameters params;

    bool result = handle_arguments(argc, argv, params);

    REQUIRE(result == false);
}

TEST_CASE("handle_arguments - Zero bullet mass", "[handle_arguments]") {
    int argc = 10;
    char* argv[] = {
        (char*)"simulation",
        (char*)"0.0", (char*)"0.0", (char*)"0.0",  // Muzzle position
        (char*)"100.0", (char*)"0.0", (char*)"0.0", // Target position
        (char*)"300.0", // Initial speed
        (char*)"0.0",  // Zero bullet mass
        (char*)"0.1"    // Step length
    };

    Parameters params;

    bool result = handle_arguments(argc, argv, params);

    REQUIRE(result == false);
}

// simulate_trajectory_loop function tests

TEST_CASE("simulate_trajectory_loop - Bullet hits the target", "[simulate_trajectory_loop]") {
    Parameters params;
    params.muzzle_pos = {0.0, 0.0, 0.0};
    params.target_pos = {100.0, 0.0, 0.0};
    params.initial_speed = 300.0;
    params.bullet_mass = 7.5;
    params.step_length = 0.01;
    params.print_progress = false;
    params.print_trajectory = false;
    double horizontal_angle = 0.0;
    double elevation_angle = 45.0 / 180.0 * M_PI;

    entt::registry registry;
    auto bullet = create_bullet(registry, horizontal_angle, elevation_angle, params.initial_speed, params.muzzle_pos, params.bullet_mass);
    TrajectoryResult result = simulate_trajectory_loop(registry, bullet, params);

    auto &final_pos = registry.get<Position>(bullet);
    // Assert
    // Hard to test for hit directly due to precision and step size
    REQUIRE(final_pos.x == Approx(params.target_pos.x).margin(HIT_TOLERANCE));
    REQUIRE(final_pos.y == Approx(params.target_pos.y).margin(HIT_TOLERANCE));
    REQUIRE(final_pos.z == Approx(params.target_pos.z).margin(5));
}


TEST_CASE("simulate_trajectory_loop - Bullet misses the target (too high)", "[simulate_trajectory_loop]") {
    Parameters params;
    params.muzzle_pos = {0.0, 0.0, 0.0};
    params.target_pos = {10.0, 0.0, 0.0};
    params.initial_speed = 3000.0;
    params.bullet_mass = 7.6;
    params.step_length = 0.01;
    params.print_progress = false;
    params.print_trajectory = false;
    double horizontal_angle = 0.0;
    double elevation_angle = 45.0 / 180.0 * M_PI;

    TrajectoryResult result = simulate_bullet_trajectory(horizontal_angle, elevation_angle, params);

    REQUIRE(result.result == TOO_HIGH);
}


TEST_CASE("simulate_trajectory_loop - Bullet misses the target (too far)", "[simulate_trajectory_loop]") {
    Parameters params;
    params.muzzle_pos = {0.0, 0.0, 0.0};
    params.target_pos = {10000.0, 0.0, 0.0};
    params.initial_speed = 300.0;
    params.bullet_mass = 7.6;
    params.step_length = 0.1;
    params.print_progress = false;
    params.print_trajectory = false;
    double horizontal_angle = 0.0;
    double elevation_angle = 10.0 / 180.0 * M_PI;

    TrajectoryResult result = simulate_bullet_trajectory(horizontal_angle, elevation_angle, params);

    REQUIRE(result.result == TOO_LOW);
}


TEST_CASE("simulate_trajectory_loop - Angle too low to reach target", "[simulate_trajectory_loop]") {
    Parameters params;
    params.muzzle_pos = {0.0, 0.0, 0.0};
    params.target_pos = {100.0, 0.0, 0.0};
    params.initial_speed = 50.0;
    params.bullet_mass = 7.6;
    params.step_length = 0.1;
    params.print_progress = false;
    params.print_trajectory = false;
    double horizontal_angle = 0.0;
    double elevation_angle = 1.0 / 180.0 * M_PI;

    TrajectoryResult result = simulate_bullet_trajectory(horizontal_angle, elevation_angle, params);
    REQUIRE(result.result == TOO_LOW);
}


TEST_CASE("simulate_trajectory_loop - Bullet hits ground before target (angle too high)", "[simulate_trajectory_loop]") {
    Parameters params;
    params.muzzle_pos = {0.0, 0.0, 0.0};
    params.target_pos = {100.0, 0.0, 0.0};
    params.initial_speed = 50.0;
    params.bullet_mass = 7.6;
    params.step_length = 0.1;
    params.print_progress = false;
    params.print_trajectory = false;
    double horizontal_angle = 0.0;
    double elevation_angle = 89.0 / 180.0 * M_PI;

    TrajectoryResult result = simulate_bullet_trajectory(horizontal_angle, elevation_angle, params);        

    REQUIRE(result.result == TOO_LOW);
}

