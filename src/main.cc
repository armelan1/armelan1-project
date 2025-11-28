#include <iostream>
#include "ControlAllocation.hh"
#include <cmath>

constexpr double kLx = 0.5;
constexpr double kL  = 20.0;
constexpr double kS  = 0.70710678118; // sin(45deg) or cos(45deg)

void controlVectorBuilder(const double* mIn, double* f)
{
    f[0] = 0;
    f[1] = 0;
    f[2] = 0;
    f[3] = 0;
    // Roll
    if (mIn[0] < 0) {
        f[1] += abs(mIn[0]) / (kLx * 2);
        f[3] += abs(mIn[0]) / (kLx * 2);
    }else if (mIn[0] > 0) {
        f[0] += abs(mIn[0]) / (kLx * 2);
        f[2] += abs(mIn[0]) / (kLx * 2);
    }
    // Pitch
    if (mIn[1] < 0) {
        f[0] += (abs(mIn[1]) / kL) * kS;
        f[1] += (abs(mIn[1]) / kL) * kS;
    }else if (mIn[1] > 0) {
        f[2] += (abs(mIn[1]) / kL) * kS;
        f[3] += (abs(mIn[1]) / kL) * kS;
    }
    // Yaw
    if (mIn[2] < 0) {
        f[0] += (abs(mIn[2]) / kL) * kS;
        f[3] += (abs(mIn[2]) / kL) * kS;
    }else if (mIn[2] > 0) {
        f[1] += (abs(mIn[2]) / kL) * kS;
        f[2] += (abs(mIn[2]) / kL) * kS;
    }
}

int main() 
{

    ControlAllocation myCtrlAllocator(30.0, 300.0, 4);

    // double desiredControls[4] = {330.0, 30.0, 30.0, 30.0};
    // double desiredControls[4] = {330.0, 20.0, 30.0, 30.0};
    // double desiredControls[4] = {330.0, 40.0, 100.0, 100.0};
    // double desiredControls[4] = {270.0, -30.0, 100.0, 100.0};
    // double desiredControls[4] = {280.0, -30.0, 100.0, 100.0};
    // double desiredControls[4] = {280.0, 30.0, 100.0, 100.0};
    // double desiredControls[4] = {280.0, 30.0, 40.0, 100.0};
    // double desiredControls[4] = {280.0, 0.000000001, 40.0, 100.0};
    // double desiredControls[4] = {100.0, 200.0, 305.0, 300.0};
    // double desiredControls[4] = {100.0, 100.0, 100.0, 100.0};
    // double desiredControls[4] = {40.0, 40.0, 100.0, 330.0};

    // double desiredControls[4] = {25.0, 20.0, 15.0, 10.0};
    // double desiredControls[4] = {25.0, 30.0, 35.0, 40.0};
    // double desiredControls[4] = {25.0, 130.0, 135.0, 140.0};

    double desiredControls[4] = {300.0, 300.0, 900.0, 300.0};
    std::cout << "\nDesired Controls: ";
    for (int i = 0; i < 4; i++) {
        std::cout << desiredControls[i] << " ";
    }
    std::cout << std::endl;
    double allocatedControls[4];  // Stack allocation
    myCtrlAllocator.allocateControls(desiredControls, allocatedControls);
    std::cout << "Allocated Controls: ";
    for (int i = 0; i < 4; i++) {
        std::cout << allocatedControls[i] << " ";
    }
    std::cout << "\n\n";

    double maxRoll = kLx * 2 * 300.0; // max thrust per motor
    double maxPitch = 300.0 * kL * std::sqrt(2.0);
    double maxYaw = maxPitch;
    // std::cout << "Max Roll Moment: " << maxRoll << std::endl;
    // std::cout << "Max Pitch Moment: " << maxPitch << std::endl;
    double moments[3] = {maxRoll, maxPitch, maxYaw};
    controlVectorBuilder(moments, desiredControls);
    myCtrlAllocator.allocateControls(desiredControls, allocatedControls);
    std::cout << "Desired Controls for moments: ";
    for (int i = 0; i < 4; i++) {
        std::cout << desiredControls[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "Allocated Controls for moments: ";
    for (int i = 0; i < 4; i++) {
        std::cout << allocatedControls[i] << " ";
    }
    std::cout << std::endl;

    return 0;
}