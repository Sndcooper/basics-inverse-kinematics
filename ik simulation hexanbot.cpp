#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

struct IkResult {
    bool ok;
    double base_yaw;
    double angle1;
    double angle2;
    double angle3;
    double phi;
    double psi;
    double r_eff;
};

struct Point3 {
    double x;
    double y;
    double z;
};

static double deg(double rad) {
    return rad * 180.0 / M_PI;
}

static bool within_limits(double value, double min_value, double max_value) {
    return value >= min_value && value <= max_value;
}

static IkResult solve_ik_3d(double x, double y, double z, double l1, double l2, double l3, double foot_angle) {
    const double base_yaw = std::atan2(y, x);
    const double r = std::hypot(x, y);

    const double r_eff = r - l3 * std::cos(foot_angle);
    const double z_eff = z - l3 * std::sin(foot_angle);
    const double planar_dist = std::hypot(r_eff, z_eff);
    if (planar_dist > l1 + l2 || planar_dist < std::abs(l1 - l2)) {
        return {false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    double cos_a2 = (planar_dist * planar_dist - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    cos_a2 = std::max(-1.0, std::min(1.0, cos_a2));
    const double angle2 = -std::acos(cos_a2);

    const double phi = std::atan2(z_eff, r_eff);
    const double psi = std::atan2(l2 * std::sin(angle2), l1 + l2 * std::cos(angle2));
    const double angle1 = phi - psi;
    const double angle3 = foot_angle - (angle1 + angle2);

    return {true, base_yaw, angle1, angle2, angle3, phi, psi, r_eff};
}

static std::vector<Point3> forward_kinematics(
    double base_yaw,
    double angle1,
    double angle2,
    double angle3,
    double l1,
    double l2,
    double l3
) {
    const double dir_x = std::cos(base_yaw);
    const double dir_y = std::sin(base_yaw);

    const double r1 = l1 * std::cos(angle1);
    const double z1 = l1 * std::sin(angle1);
    const double r2 = r1 + l2 * std::cos(angle1 + angle2);
    const double z2 = z1 + l2 * std::sin(angle1 + angle2);
    const double r3 = r2 + l3 * std::cos(angle1 + angle2 + angle3);
    const double z3 = z2 + l3 * std::sin(angle1 + angle2 + angle3);

    Point3 p0{0.0, 0.0, 0.0};
    Point3 p1{0.0, 0.0, 0.0};
    Point3 p2{r1 * dir_x, r1 * dir_y, z1};
    Point3 p3{r2 * dir_x, r2 * dir_y, z2};
    Point3 p4{r3 * dir_x, r3 * dir_y, z3};

    return {p0, p1, p2, p3, p4};
}

static Point3 offset_point(const Point3& p, const Point3& offset) {
    return {p.x + offset.x, p.y + offset.y, p.z + offset.z};
}

int main(int argc, char** argv) {
    const double L1 = 26.0;
    const double L2 = 57.0;
    const double L3 = 122.0;

    const double HIP_MIN = -80.0 * M_PI / 180.0;
    const double HIP_MAX = 80.0 * M_PI / 180.0;
    const double KNEE_MIN = -30.0 * M_PI / 180.0;
    const double KNEE_MAX = 90.0 * M_PI / 180.0;
    const double ANKLE_MIN = -140.0 * M_PI / 180.0;
    const double ANKLE_MAX = 20.0 * M_PI / 180.0;

    const double HIP_RADIUS = 137.5;
    const int NUM_LEGS = 6;

    double target_x = 50.0;
    double target_y = 40.0;
    double target_z = -50.0;
    if (argc == 4) {
        target_x = std::stod(argv[1]);
        target_y = std::stod(argv[2]);
        target_z = std::stod(argv[3]);
    }

    std::vector<Point3> hip_positions;
    hip_positions.reserve(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; ++i) {
        const double angle = (M_PI / 180.0) * (i * 60.0);
        hip_positions.push_back({HIP_RADIUS * std::cos(angle), HIP_RADIUS * std::sin(angle), 0.0});
    }

    const double foot_angle = 0.0;
    int reachable_count = 0;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Target (world): (" << target_x << ", " << target_y << ", " << target_z << ")\n";
    std::cout << "Hip radius: " << HIP_RADIUS << " mm, spacing: 60 deg\n\n";

    for (int i = 0; i < NUM_LEGS; ++i) {
        const Point3 hip = hip_positions[i];
        const double local_x = target_x - hip.x;
        const double local_y = target_y - hip.y;
        const double local_z = target_z - hip.z;

        const IkResult ik = solve_ik_3d(local_x, local_y, local_z, L1, L2, L3, foot_angle);
        bool ok = ik.ok;
        if (ok) {
            ok = within_limits(ik.angle1, HIP_MIN, HIP_MAX)
                && within_limits(ik.angle2, KNEE_MIN, KNEE_MAX)
                && within_limits(ik.angle3, ANKLE_MIN, ANKLE_MAX);
        }

        std::cout << "Leg " << i << " hip=(" << hip.x << ", " << hip.y << ", " << hip.z << ")";
        if (!ok) {
            std::cout << " -> unreachable\n";
            continue;
        }

        ++reachable_count;
        const auto fk_points = forward_kinematics(ik.base_yaw, ik.angle1, ik.angle2, ik.angle3, L1, L2, L3);
        const Point3 eff_local = fk_points.back();
        const Point3 eff_world = offset_point(eff_local, hip);
        const double dx = target_x - eff_world.x;
        const double dy = target_y - eff_world.y;
        const double dz = target_z - eff_world.z;
        const double err = std::hypot(std::hypot(dx, dy), dz);

        std::cout << " -> base_yaw=" << ik.base_yaw << " rad (" << deg(ik.base_yaw) << " deg)"
                  << ", a1=" << ik.angle1 << " rad, a2=" << ik.angle2 << " rad, a3=" << ik.angle3 << " rad"
                  << ", err=" << err << "\n";
    }

    std::cout << "\nReachable legs: " << reachable_count << "/" << NUM_LEGS << "\n";
    return 0;
}
