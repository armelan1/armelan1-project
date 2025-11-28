#include "UnitTest++/UnitTest++.h"
#include "ControlAllocation.hh"

namespace
{
    constexpr double kMinT      = 30.0;
    constexpr double kMaxT      = 300.0;
    constexpr int    kNumInputs = 4;
    constexpr int    kNumAxes   = 3;
    constexpr double kTolerance = 1e-12;

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
    // TODO refactor so that isInBounds only takes double array as input?
    bool isInBounds(const double* u, int n, double minT, double maxT)
    {
        for (int i = 0; i < n; ++i)
        {
            double val = u[i];
            if (val < 0.0 || (val > 0.0 && val < minT) || val > maxT)
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
            CHECK_CLOSE(mIn[i], mOut[i], kTolerance);
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

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_AlreadyInBounds_StaysInBounds)
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

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_LargeSpread_TriggersScalingIntoRange)
{
    inputs[0] = -100.0;
    inputs[1] = 0.0;
    inputs[2] = 500.0;
    inputs[3] = 200.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
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

TEST_FIXTURE(ControlAllocationFixture, AllocateControls_InBoundsInput_ShiftToZeroFail)
{
    inputs[0] = 40.0;
    inputs[1] = 50.0;
    inputs[2] = 120.0;
    inputs[3] = 160.0;

    ca.allocateControls(inputs, outputs);

    CHECK(isInBounds(outputs, kNumInputs, kMinT, kMaxT));
    checkMomentsEqual(inputs, outputs);
}

} // SUITE(ControlAllocationTests)

int main()
{
    return UnitTest::RunAllTests();
}
