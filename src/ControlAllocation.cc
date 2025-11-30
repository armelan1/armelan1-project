#include "ControlAllocation.hh"
#include <cmath>

// TODO check floating point tolerance issues.
// TODO maybe keep this input as const double* and do a different method to round/floor to zero
bool ControlAllocation::isInBounds(double* controlInputs) {
    for (int i = 0; i < _numberInputs; i++) {
        if (std::fabs(controlInputs[i]) < ROUNDING_TOLERANCE) {
            controlInputs[i] = 0.0;
        }
        if ((controlInputs[i] < -FLOATING_POINT_TOLERANCE) || 
            ((controlInputs[i] < _minT) && (controlInputs[i] > FLOATING_POINT_TOLERANCE)) || 
            controlInputs[i] > _maxT) {
            return false;
        }
    }
    return true;
}

void ControlAllocation::shift(const double* controlInputs, double* controlOutputs,
                               Direction direction, double amount) {
    for (int i = 0; i < _numberInputs; i++) {
        controlOutputs[i] = controlInputs[i] + (static_cast<int>(direction) * amount);
    }
}

void ControlAllocation::scale(const double* controlInputs, double* controlOutputs, double minInput, double maxInput) {
    for (int i = 0; i < _numberInputs; i++) {
        double normalized = (controlInputs[i] - minInput) / (maxInput - minInput);
        controlOutputs[i] = normalized * (_maxT - _minT) + _minT;
    }
}

void ControlAllocation::shiftAfterScale(double* controlOutputs) {
    double tempInputs[_numberInputs];
    shift(controlOutputs, tempInputs, Direction::COPY, 0.0); // copy outputs to tempInputs
    shift(tempInputs, controlOutputs, Direction::NEGATIVE, _minT);
    if (isInBounds(controlOutputs))
        return;
    shift(tempInputs, controlOutputs, Direction::COPY, 0.0); // copy tempInputs to controlOutputs
}

void ControlAllocation::allocateControls(double* controlInputs, double* controlOutputs) {
    double maxInput = getMax(controlInputs);
    double minInput = getMin(controlInputs);

    if ((maxInput - minInput) > _maxT) {  // (maxInput - minInput) > (_maxT - _minT) handled at end of function
        scale(controlInputs, controlOutputs, minInput, maxInput);
        shiftAfterScale(controlOutputs); // try to shift after scaling to reduce thrust
        return;
    }

    shift(controlInputs, controlOutputs, Direction::POSITIVE, -minInput); // shift minInput to zero
    if (isInBounds(controlOutputs))
        return;
    shift(controlInputs, controlOutputs, Direction::POSITIVE, -(minInput - _minT)); // shift minInput to minT
    if (isInBounds(controlOutputs))
        return;

    // edge case maxInput - minInput > _maxT - _minT
    scale(controlInputs, controlOutputs, minInput, maxInput);
    shiftAfterScale(controlOutputs); // try to shift after scaling to reduce thrust
}