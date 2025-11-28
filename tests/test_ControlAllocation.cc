#include "UnitTest++/UnitTest++.h"
#include "ControlAllocation.hh"

// TODO add logic to compute original moment and resulting moment to compare after allocating controls
namespace
{
    constexpr double kMinT      = 30.0;
    constexpr double kMaxT      = 300.0;
    constexpr int    kNumInputs = 4;

    bool isInBoundsTest(const double* u)
    {
        for (int i = 0; i < kNumInputs; ++i)
        {
            double val = u[i];

            if (val < 0.0)
                return false;

            if (val > 0.0 && val < kMinT)
                return false;

            if (val > kMaxT)
                return false;
        }
        return true;
    }
}

namespace
{
    // rows x cols matrix A (row-major) times vector x (length cols)
    // result y (length rows)
    void matVecMultiply(const double* A,
                        const double* x,
                        double* y,
                        int rows,
                        int cols)
    {
        for (int r = 0; r < rows; ++r)
        {
            double sum = 0.0;
            for (int c = 0; c < cols; ++c)
            {
                sum += A[r * cols + c] * x[c]; // row-major: A[r, c]
            }
            y[r] = sum;
        }
    }
}

namespace
{
    constexpr int numAxes   = 3;
    constexpr int numInputs = 4;
    constexpr double Lx = 0.5;
    constexpr double L = 20.0;
    constexpr double s = 0.70710678118; // sin(45deg) or cos(45deg)

    // Control effectiveness matrix B (3 x 4, row-major)
    // Each column is the moment contribution of one thruster.
    const double B[numAxes * numInputs] =
    {
        //  u0    u1    u2    u3
          Lx,  -Lx,   Lx,  -Lx,   // roll axis
         -L*s, -L*s,  L*s,  L*s,   // pitch axis
         -L*s,  L*s,  L*s, -L*s    // yaw axis (all equal)
    };
}


TEST(Constructor_SetsParameters)
{
    ControlAllocation ca(kMinT, kMaxT, kNumInputs);

    CHECK_CLOSE(kMinT,      ca.getMinT(),      1e-12);
    CHECK_CLOSE(kMaxT,      ca.getMaxT(),      1e-12);
    CHECK_EQUAL(kNumInputs, ca.getNumberInputs());
}

TEST(Setters_UpdateParameters)
{
    ControlAllocation ca(kMinT, kMaxT, kNumInputs);

    ca.setMinT(2.0);
    ca.setMaxT(6.0);
    ca.setNumberInputs(6);

    CHECK_CLOSE(2.0, ca.getMinT(), 1e-12);
    CHECK_CLOSE(6.0, ca.getMaxT(), 1e-12);
    CHECK_EQUAL(6,   ca.getNumberInputs());
}


TEST(AllocateControls_AlreadyInBounds_StaysInBounds)
{
    ControlAllocation ca(kMinT, kMaxT, kNumInputs);


    double inputs[kNumInputs]  = {0.0, 30.0, 100.0, 200.0};
    double outputs[kNumInputs] = {};

    ca.allocateControls(inputs, outputs);

    CHECK(isInBoundsTest(outputs));

    // Compute moments: M = B * u
    double mIn[numAxes]  = {};
    double mOut[numAxes] = {};

    matVecMultiply(B, inputs,  mIn,  numAxes, numInputs);
    matVecMultiply(B, outputs, mOut, numAxes, numInputs);

    for (int i = 0; i < numAxes; ++i)
    {
        CHECK_CLOSE(mIn[i], mOut[i], 1e-12);
    }
}

TEST(AllocateControls_WithNegatives_ProducesInBoundsOutputs)
{
    ControlAllocation ca(kMinT, kMaxT, kNumInputs);

    double inputs[kNumInputs]  = {-20.0, 100.5, 200.0, 140.0};
    double outputs[kNumInputs] = {};

    ca.allocateControls(inputs, outputs);

    CHECK(isInBoundsTest(outputs));
    // Compute moments: M = B * u
    double mIn[numAxes]  = {};
    double mOut[numAxes] = {};

    matVecMultiply(B, inputs,  mIn,  numAxes, numInputs);
    matVecMultiply(B, outputs, mOut, numAxes, numInputs);

    for (int i = 0; i < numAxes; ++i)
    {
        CHECK_CLOSE(mIn[i], mOut[i], 1e-12);
    }
}

TEST(AllocateControls_WithValuesAboveMax_ProducesInBoundsOutputs)
{
    ControlAllocation ca(kMinT, kMaxT, kNumInputs);

    double inputs[kNumInputs]  = {100.0, 200.0, 305.5, 300.0};
    double outputs[kNumInputs] = {};

    ca.allocateControls(inputs, outputs);

    CHECK(isInBoundsTest(outputs));

    // Compute moments: M = B * u
    double mIn[numAxes]  = {};
    double mOut[numAxes] = {};

    matVecMultiply(B, inputs,  mIn,  numAxes, numInputs);
    matVecMultiply(B, outputs, mOut, numAxes, numInputs);

    for (int i = 0; i < numAxes; ++i)
    {
        CHECK_CLOSE(mIn[i], mOut[i], 1e-12);
    }
}

TEST(AllocateControls_LargeSpread_TriggersScalingIntoRange)
{
    ControlAllocation ca(kMinT, kMaxT, kNumInputs);

    double inputs[kNumInputs]  = {-100.0, 0.0, 500.0, 200.0};
    double outputs[kNumInputs] = {};

    ca.allocateControls(inputs, outputs);


    for (int i = 0; i < kNumInputs; ++i)
    {
        double v = outputs[i];
        CHECK((v == 0.0) || (v >= kMinT && v <= kMaxT)); //TODO check if v is close to zero?
    }
}

TEST(AllocateControls_AllSameInputs_ProducesZeroOutputs)
{
    ControlAllocation ca(kMinT, kMaxT, kNumInputs);

    double inputs[kNumInputs]  = {-100.0, -100.0, -100.0, -100.0};
    double outputs[kNumInputs] = {};

    ca.allocateControls(inputs, outputs);

    for (int i = 0; i < kNumInputs; ++i)
    {
        CHECK_CLOSE(0.0, outputs[i], 1e-12);
    }
    // Compute moments: M = B * u
    double mIn[numAxes]  = {};
    double mOut[numAxes] = {};

    matVecMultiply(B, inputs,  mIn,  numAxes, numInputs);
    matVecMultiply(B, outputs, mOut, numAxes, numInputs);

    for (int i = 0; i < numAxes; ++i)
    {
        CHECK_CLOSE(mIn[i], mOut[i], 1e-12);
    }
}

TEST(AllocateControls_InBoundsInput_ShiftToZeroSucceed)
{
    ControlAllocation ca(kMinT, kMaxT, kNumInputs);

    double inputs[kNumInputs]  = {50.0, 100.0, 150.0, 200.0};
    double outputs[kNumInputs] = {};

    ca.allocateControls(inputs, outputs);

    CHECK(isInBoundsTest(outputs));

    // Compute moments: M = B * u
    double mIn[numAxes]  = {};
    double mOut[numAxes] = {};

    matVecMultiply(B, inputs,  mIn,  numAxes, numInputs);
    matVecMultiply(B, outputs, mOut, numAxes, numInputs);

    for (int i = 0; i < numAxes; ++i)
    {
        CHECK_CLOSE(mIn[i], mOut[i], 1e-12);
    }
}

TEST(AllocateControls_InBoundsInput_ShiftToZeroFail)
{
    ControlAllocation ca(kMinT, kMaxT, kNumInputs);

    double inputs[kNumInputs]  = {40.0, 50.0, 120.0, 160.0};
    double outputs[kNumInputs] = {};

    ca.allocateControls(inputs, outputs);

    CHECK(isInBoundsTest(outputs));

    // Compute moments: M = B * u
    double mIn[numAxes]  = {};
    double mOut[numAxes] = {};

    matVecMultiply(B, inputs,  mIn,  numAxes, numInputs);
    matVecMultiply(B, outputs, mOut, numAxes, numInputs);

    for (int i = 0; i < numAxes; ++i)
    {
        CHECK_CLOSE(mIn[i], mOut[i], 1e-12);
    }
}

int main()
{
    return UnitTest::RunAllTests();
}
