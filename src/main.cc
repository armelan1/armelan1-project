#include <iostream>
#include "ControlAllocation.hh"
#include <cmath>

constexpr double kLx = 0.5;
constexpr double kL  = 20.0;
constexpr double kS  = 0.70710678118; // sin(45deg) or cos(45deg)

constexpr int kNumInputs = 4;
constexpr int kNumAxes = 3;
constexpr double kTolerance = FLOATING_POINT_TOLERANCE;

// Control effectiveness matrix B (3 x 4, row-major)
const double B[3 * 4] =
{
    //  u0      u1      u2      u3
        kLx,    -kLx,    kLx,    -kLx,   // roll axis
        -kL*kS, -kL*kS,  kL*kS,  kL*kS,   // pitch axis
        -kL*kS,  kL*kS,  kL*kS, -kL*kS    // yaw axis
};

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

// Matrix-vector multiply: M = B * u
void matVecMultiply(const double* A, const double* x, double* y, int rows, int cols)
{
    for (int r = 0; r < rows; ++r)
    {
        double sum = 0.0;
        for (int c = 0; c < cols; ++c)
        {
            sum += A[r * cols + c] * x[c];
        }
        y[r] = sum;
    }
}

void checkMomentsEqual(const double* uIn, const double* uOut)
{
    double mIn[3]  = {};
    double mOut[3] = {};

    matVecMultiply(B, uIn,  mIn,  3, 4);
    matVecMultiply(B, uOut, mOut, 3, 4);

    for (int i = 0; i < 3; ++i)
    {
        std::cout << "mIn[" << i << "] = " << mIn[i] << ", mOut[" << i << "] = " << mOut[i] << std::endl;
    }
}

// Check if two moment vectors have the same direction (parallel, including zero vectors)
void checkMomentDirectionEqual(const double* uIn, const double* uOut)
{
    double mIn[kNumAxes]  = {};
    double mOut[kNumAxes] = {};

    matVecMultiply(B, uIn,  mIn,  kNumAxes, kNumInputs);
    matVecMultiply(B, uOut, mOut, kNumAxes, kNumInputs);

    // Calculate magnitudes
    double magIn = 0.0;
    double magOut = 0.0;
    for (int i = 0; i < kNumAxes; ++i)
    {
        magIn += mIn[i] * mIn[i];
        magOut += mOut[i] * mOut[i];
    }
    magIn = std::sqrt(magIn);
    magOut = std::sqrt(magOut);

    // If both are near zero, they're trivially equal in direction
    if (magIn < kTolerance && magOut < kTolerance)
    {
        return;
    }

    // If one is zero and the other isn't, directions are not equal
    if (magIn < kTolerance || magOut < kTolerance)
    {
        std::cout << "One moment vector is zero while the other is not" << std::endl;
        return;
    }

    // Normalize to unit vectors
    double unitIn[kNumAxes];
    double unitOut[kNumAxes];
    for (int i = 0; i < kNumAxes; ++i)
    {
        unitIn[i] = mIn[i] / magIn;
        unitOut[i] = mOut[i] / magOut;
    }

    // Check if unit vectors are equal (same direction)
    for (int i = 0; i < kNumAxes; ++i)
    {
        std::cout << "unitIn[" << i << "] = " << unitIn[i] << ", unitOut[" << i << "] = " << unitOut[i] << std::endl;
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

    // double desiredControls[4] = {300.0, 300.0, 900.0, 300.0};
    // double desiredControls[4] = { -300.0, 229.9, 1899.04, 119.21 };
    double desiredControls[4] = { -200.0, 229.9, 189.04, 119.21 };
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
    std::cout << "\n";
    checkMomentsEqual(desiredControls, allocatedControls);
    checkMomentDirectionEqual(desiredControls, allocatedControls);
    std::cout << "\n\n";

    double maxRoll = kLx * 2 * 300.0; // max thrust per motor
    double maxPitch = 300.0 * kL * std::sqrt(2.0);
    double maxYaw = maxPitch;
    std::cout << "Max Roll Moment: " << maxRoll << std::endl;
    std::cout << "Max Pitch Moment: " << maxPitch << std::endl;
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