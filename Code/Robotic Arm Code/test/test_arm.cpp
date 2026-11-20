// PC test for arm_kinematics.h. No Arduino needed.
// Build:  g++ -std=c++11 -Wall test_arm.cpp -o test_arm
// Run:    ./test_arm
#include <cstdio>
#include "kinematics.h"

int main() {
    Vec2 P  = {0, 106.934};    // pivot
    Vec2 b  = {0, 46.99};  // fixed base point of the actuator
    Vec2 y0 = {251.60, 184.76};   // y in the reference pose
    Vec2 x0 =  {237.86, 454.04};   // x in the reference pose

    ArmKinematics arm(P, b, y0, x0);
    std::printf("Reference length l0 = %.4f\n", arm.referenceLength());
    std::printf("Type a change in l (e.g. 0.05 or -0.02), 's' for a sweep, 'q' to quit.\n");

    char line[64];
    while (true) {
        std::printf("dl> ");
        if (!std::fgets(line, sizeof line, stdin)) break;
        if (line[0] == 'q') break;

        if (line[0] == 's') {  // sweep dl from 50 to -50 in steps of 5
            for (int i = -10; i <= 10; ++i) {
                float dl = i * 5.0f;
                Vec2 p;
                if (arm.xPosition(dl, p))
                    std::printf("  dl=%+.2f  x=(%.4f, %.4f)\n", dl, p.x, p.y);
                else
                    std::printf("  dl=%+.2f  unreachable\n", dl);
            }
            continue;
        }

        float dl;
        if (std::sscanf(line, "%f", &dl) != 1) {
            std::printf("  couldn't read a number\n");
            continue;
        }
        Vec2 p;
        if (arm.xPosition(dl, p))
            std::printf("  x = (%.4f, %.4f)\n", p.x, p.y);
        else
            std::printf("  unreachable: l = %.4f is outside what the geometry allows\n",
                        arm.referenceLength() + dl);
    }
    return 0;
}