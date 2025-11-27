#include "ControlAllocation.hh"

bool ControlAllocation::isInBounds(const double* controlInputs) {
    for (int i = 0; i < _numberInputs; i++) {
        if ((controlInputs[i] < 0) || 
            ((controlInputs[i] < _minT) && (controlInputs[i] > 0)) || 
            controlInputs[i] > _maxT) {
            return false;
        }
    }
    return true;
}

void ControlAllocation::shift(const double* controlInputs, double* controlOutputs,
                               int direction, double amount) {
    for (int i = 0; i < _numberInputs; i++) {
        controlOutputs[i] = controlInputs[i] + (direction * amount);
    }
}

void ControlAllocation::scale(const double* controlInputs, double* controlOutputs) {
    double maxInput = getMax(controlInputs);
    double minInput = getMin(controlInputs);
    
    // Handle edge case where all inputs are the same
    if (maxInput == minInput) {
        double targetValue = (_minT + _maxT) / 2.0;
        for (int i = 0; i < _numberInputs; i++) {
            controlOutputs[i] = targetValue;
        }
        return;
    }
    
    for (int i = 0; i < _numberInputs; i++) {
        double normalized = (controlInputs[i] - minInput) / (maxInput - minInput);
        controlOutputs[i] = normalized * (_maxT - _minT) + _minT;
    }
}

void ControlAllocation::allocateControls(const double* controlInputs, 
                                         double* controlOutputs) {
    double maxInput = getMax(controlInputs);
    double minInput = getMin(controlInputs);

    if ((maxInput - minInput) > _maxT) {
        scale(controlInputs, controlOutputs);
        return;
    }

    if (minInput < 0) {
        shift(controlInputs, controlOutputs, 1, -minInput);
        if (isInBounds(controlOutputs))
            return;
        shift(controlInputs, controlOutputs, 1, -(minInput - _minT));
        if (isInBounds(controlOutputs))
            return;
    } else if (maxInput > _maxT) {
        shift(controlInputs, controlOutputs, -1, maxInput - _maxT);
        if (isInBounds(controlOutputs))
            return;
        shift(controlInputs, controlOutputs, -1, (maxInput - _maxT) + (minInput - _minT));
        if (isInBounds(controlOutputs))
            return;
    }
    
    scale(controlInputs, controlOutputs);
}