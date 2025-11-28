#include "UnitTest++/UnitTest++.h"
#include "ControlAllocation.hh"

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
}

TEST(AllocateControls_WithNegatives_ProducesInBoundsOutputs)
{
    ControlAllocation ca(kMinT, kMaxT, kNumInputs);

    double inputs[kNumInputs]  = {-20.0, 100.5, 200.0, 140.0};
    double outputs[kNumInputs] = {};

    ca.allocateControls(inputs, outputs);

    CHECK(isInBoundsTest(outputs));
}

TEST(AllocateControls_WithValuesAboveMax_ProducesInBoundsOutputs)
{
    ControlAllocation ca(kMinT, kMaxT, kNumInputs);

    double inputs[kNumInputs]  = {100.0, 200.0, 305.5, 300.0};
    double outputs[kNumInputs] = {};

    ca.allocateControls(inputs, outputs);

    CHECK(isInBoundsTest(outputs));
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
        CHECK((v == 0.0) || (v >= kMinT && v <= kMaxT));
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
}


int main()
{
    return UnitTest::RunAllTests();
}
