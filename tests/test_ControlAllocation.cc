#include "UnitTest++/UnitTest++.h"
#include "ControlAllocation.hh"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace
{   // TODO rename to ALL_CAPS constants?
    constexpr double kMinT      = 30.0;
    constexpr double kMaxT      = 300.0;
    constexpr int    kNumInputs = 4;
    constexpr int    kNumAxes   = 3;
    constexpr double kTolerance = FLOATING_POINT_TOLERANCE;

    constexpr double kLx = 0.5;
    constexpr double kL  = 20.0;
    constexpr double kS  = 0.70710678118; // sin(45deg) or cos(45deg)

    // Control effectiveness matrix B (3 x 4, row-major)
    const double B[kNumAxes * kNumInputs] =
    {
        //  u0      u1      u2      u3
          kLx,    -kLx,    kLx,    -kLx,   // roll axis
         -kL*kS, -kL*kS,  kL*kS,  kL*kS,   // pitch axis
         -kL*kS,  kL*kS,  kL*kS, -kL*kS    // yaw axis
    };

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

    bool isInBounds(const double* u, int n, double minT, double maxT)
    {
        for (int i = 0; i < n; ++i)
        {
            double val = u[i];
            if (val < -FLOATING_POINT_TOLERANCE || (val > FLOATING_POINT_TOLERANCE && val < minT) || val > maxT)
                return false;
        }
        return true;
    }

    void checkMomentsEqual(const double* uIn, const double* uOut)
    {
        double mIn[kNumAxes]  = {};
        double mOut[kNumAxes] = {};

        matVecMultiply(B, uIn,  mIn,  kNumAxes, kNumInputs);
        matVecMultiply(B, uOut, mOut, kNumAxes, kNumInputs);

        for (int i = 0; i < kNumAxes; ++i)
        {
            // std::cout << "mIn[" << i << "] = " << mIn[i] << ", mOut[" << i << "] = " << mOut[i] << std::endl;
            CHECK_CLOSE(mIn[i], mOut[i], kTolerance);
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
            CHECK(false && "One moment vector is zero while the other is not");
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
            CHECK_CLOSE(unitIn[i], unitOut[i], kTolerance);
        }
    }

    double thrustSqSum(const double* u, int n)
    {
        double sum = 0.0;
        for (int i = 0; i < n; ++i)
        {
            sum += u[i] * u[i];
        }
        return sum;
    }

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
}

struct ControlAllocationFixture
{
    ControlAllocation ca;
    double inputs[kNumInputs];
    double outputs[kNumInputs];

    ControlAllocationFixture()
        : ca(kMinT, kMaxT, kNumInputs)
        , inputs{}
        , outputs{}
    {
    }
};

SUITE(ControlAllocationTests)
{

TEST_FIXTURE(ControlAllocationFixture, Constructor_SetsParameters)
{
    CHECK_CLOSE(kMinT, ca.getMinT(), kTolerance);
    CHECK_CLOSE(kMaxT, ca.getMaxT(), kTolerance);
    CHECK_EQUAL(kNumInputs, ca.getNumberInputs());
}

TEST_FIXTURE(ControlAllocationFixture, Setters_UpdateParameters)
{
    ca.setMinT(2.0);
    ca.setMaxT(6.0);
    ca.setNumberInputs(6);

    CHECK_CLOSE(2.0, ca.getMinT(), kTolerance);
    CHECK_CLOSE(6.0, ca.getMaxT(), kTolerance);
    CHECK_EQUAL(6, ca.getNumberInputs());
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_AlreadyInBounds_MinIsZero_StaysInBounds)
{
    inputs[0] = 0.0;
    inputs[1] = 30.0;
    inputs[2] = 100.0;
    inputs[3] = 200.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_WithNegatives_ProducesInBoundsOutputs)
{
    inputs[0] = -20.0;
    inputs[1] = 100.5;
    inputs[2] = 200.0;
    inputs[3] = 140.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_WithValuesAboveMax_ProducesInBoundsOutputs)
{
    inputs[0] = 100.0;
    inputs[1] = 200.0;
    inputs[2] = 305.5;
    inputs[3] = 300.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_WithValuesAboveMax_CloseDeadband_ProducesInBoundsOutputs)
{
    inputs[0] = 100.0;
    inputs[1] = 101.0;
    inputs[2] = 305.5;
    inputs[3] = 300.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_LargeSpread_TriggersScalingIntoRange)
{
    inputs[0] = -100.0;
    inputs[1] = 0.0;
    inputs[2] = 500.0;
    inputs[3] = 200.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentDirectionEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_WithValuesAboveMax_TriggersScaling)
{   // edge case maxInput - minInput < _maxT - _minT
    inputs[0] = 30.0;
    inputs[1] = 31.0;
    inputs[2] = 305.0;
    inputs[3] = 300.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentDirectionEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_MaxShift_MinToZero_SecondMinToMinT)
{
    inputs[0] = 35.0;
    inputs[1] = 5.0;
    inputs[2] = 305.0;
    inputs[3] = 300.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_NegativeMax_TriggersScaling)
{
    inputs[0] = -30.0;
    inputs[1] = -200.0;
    inputs[2] = -305.0;
    inputs[3] = -300.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentDirectionEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_AllSameInputs_ProducesZeroOutputs)
{
    inputs[0] = -100.0;
    inputs[1] = -100.0;
    inputs[2] = -100.0;
    inputs[3] = -100.0;

    ca.allocateControls(inputs, outputs);

    for (int i = 0; i < kNumInputs; ++i)
    {
        CHECK_CLOSE(0.0, outputs[i], kTolerance);
    }
    checkMomentsEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_AllInputsInDeadband)
{
    inputs[0] = 25.0;
    inputs[1] = 20.0;
    inputs[2] = 15.0;
    inputs[3] = 10.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_AllInputsNegativeInDeadband)
{
    inputs[0] = -25.0;
    inputs[1] = -20.0;
    inputs[2] = -15.0;
    inputs[3] = -10.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_InputInDeadband_CannotReduceThrust)
{
    inputs[0] = 25.0;
    inputs[1] = 30.0;
    inputs[2] = 135.0;
    inputs[3] = 130.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_InputInDeadband_CanReduceThrust)
{
    inputs[0] = 25.0;
    inputs[1] = 130.0;
    inputs[2] = 135.0;
    inputs[3] = 140.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
    CHECK(thrustSqSum(outputs, kNumInputs) < thrustSqSum(inputs, kNumInputs));
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_InBoundsInput_ShiftToZeroSucceed)
{
    inputs[0] = 50.0;
    inputs[1] = 100.0;
    inputs[2] = 150.0;
    inputs[3] = 200.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_InBoundsInput_ShiftToMinSucceed)
{
    inputs[0] = 50.0;
    inputs[1] = 40.0;
    inputs[2] = 120.0;
    inputs[3] = 160.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_DoubleMinShift)
{
    inputs[0] = -30.0;
    inputs[1] = 40.0;
    inputs[2] = -40.0;
    inputs[3] = 160.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_DoubleMinShift_TriggerScale)
{
    inputs[0] = -30.0;
    inputs[1] = 40.0;
    inputs[2] = -40.0;
    inputs[3] = 240.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentDirectionEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_DoubleMaxShift)
{
    inputs[0] = 40.0;
    inputs[1] = 40.0;
    inputs[2] = 40.0;
    inputs[3] = 330.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_DoubleMaxShift_TriggerScale)
{
    inputs[0] = 40.0;
    inputs[1] = 50.0;
    inputs[2] = 40.0;
    inputs[3] = 330.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentDirectionEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_LargePositveVector)
{
    double m[3] = {10000.0, 10000.0, 10000.0};
    controlVectorBuilder(m, inputs);

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentDirectionEqual(inputs, outputs);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_MaxAllAxes)
{
    double m[3] = {300.0, 8485.28, 8485.28};
    controlVectorBuilder(m, inputs);

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentDirectionEqual(inputs, outputs);
}

// TODO test if you can minmax normilize and then shift to zero
TEST_FIXTURE(ControlAllocationFixture, AllocateControls_MaxRollMaxPitchMaxYaw)
{
    inputs[0] = 300.0;
    inputs[1] = 300.0;
    inputs[2] = 900.0;
    inputs[3] = 300.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentDirectionEqual(inputs, outputs);

    CHECK_CLOSE(0.0, *std::min_element(outputs, outputs + kNumInputs), FLOATING_POINT_TOLERANCE);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_MaxRollMaxPitchMaxYaw_FloatingPointError)
{
    inputs[0] = 300.0;
    inputs[1] = 299.99;
    inputs[2] = 899.98;
    inputs[3] = 299.99;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentDirectionEqual(inputs, outputs);

    CHECK_CLOSE(0.0, *std::min_element(outputs, outputs + kNumInputs), FLOATING_POINT_TOLERANCE);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_MaxRollMaxPitchMaxYaw_FloatingPointErrorMixed)
{
    inputs[0] = -300.0;
    inputs[1] = 229.9;
    inputs[2] = 1899.04;
    inputs[3] = 119.21;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentDirectionEqual(inputs, outputs);

    CHECK_CLOSE(0.0, *std::min_element(outputs, outputs + kNumInputs), FLOATING_POINT_TOLERANCE);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_MaxRollMaxPitchMaxYaw_FloatingPointTolerancePositive)
{
    inputs[0] = 0.0000001;
    inputs[1] = 0.0000001;
    inputs[2] = 0.0000001;
    inputs[3] = 0.0000001;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentDirectionEqual(inputs, outputs);

    CHECK_CLOSE(0.0, *std::min_element(outputs, outputs + kNumInputs), FLOATING_POINT_TOLERANCE);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_MaxRollMaxPitchMaxYaw_FloatingPointToleranceNegative)
{
    inputs[0] = -0.0000001;
    inputs[1] = -0.0000001;
    inputs[2] = -0.0000001;
    inputs[3] = -0.0000001;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentDirectionEqual(inputs, outputs);

    CHECK_CLOSE(0.0, *std::min_element(outputs, outputs + kNumInputs), FLOATING_POINT_TOLERANCE);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_MaxRollMaxPitchMaxYaw_FloatingPointToleranceMixedSign)
{
    inputs[0] = -0.0000001;
    inputs[1] = 0.0000001;
    inputs[2] = -0.0000001;
    inputs[3] = 0.0000001;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentDirectionEqual(inputs, outputs);

    CHECK_CLOSE(0.0, *std::min_element(outputs, outputs + kNumInputs), FLOATING_POINT_TOLERANCE);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_MaxRollMaxPitchMaxYaw_FloatingPointError_Shift)
{
    inputs[0] = -20.0;
    inputs[1] = 229.9;
    inputs[2] = 189.04;
    inputs[3] = 119.21;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);

    CHECK_CLOSE(0.0, *std::min_element(outputs, outputs + kNumInputs), FLOATING_POINT_TOLERANCE);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_MaxRollMaxPitchMaxYaw_FloatingPointError_Shift_LargeNegativeInput)
{
    inputs[0] = -200.0;
    inputs[1] = 29.9;
    inputs[2] = 19.04;
    inputs[3] = 19.21;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);

    CHECK_CLOSE(0.0, *std::min_element(outputs, outputs + kNumInputs), FLOATING_POINT_TOLERANCE);
}

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_MaxRollMaxPitchMaxYaw_FloatingPointError_Shift_Deadband)
{
    inputs[0] = 200.0;
    inputs[1] = 29.9;
    inputs[2] = 19.04;
    inputs[3] = 19.21;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

} // SUITE(ControlAllocationTests)

int main()
{
    return UnitTest::RunAllTests();
}
