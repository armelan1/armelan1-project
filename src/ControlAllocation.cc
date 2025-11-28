#include "ControlAllocation.hh"

// TODO check floating point tolerance issues.
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
    if ((maxInput - minInput) < 1e-6) {
        for (int i = 0; i < _numberInputs; i++) {
            controlOutputs[i] = 0.0; // turn off lateral thrusters to conserve axial
        }
        return;
    }
    
    for (int i = 0; i < _numberInputs; i++) {
        double normalized = (controlInputs[i] - minInput) / (maxInput - minInput);
        controlOutputs[i] = normalized * (_maxT - _minT) + _minT;
    }
}
// TODO check if we can shift in bound command to reduce thrust.
void ControlAllocation::allocateControls(const double* controlInputs, 
                                         double* controlOutputs) {
    double maxInput = getMax(controlInputs);
    double minInput = getMin(controlInputs);

    if ((maxInput - minInput) > _maxT) {
        scale(controlInputs, controlOutputs);
        return;
    }
    // check to see if we can shift the min to zero to conserve thrust
    if (isInBounds(controlInputs)) {
        shift(controlInputs, controlOutputs, -1, minInput);
        if (isInBounds(controlOutputs)) {
            return;
        }else {
            // if shifting to zero doesn't work, revert and continue
            for (int i = 0; i < _numberInputs; i++) {
                controlOutputs[i] = controlInputs[i];
            }
            return;
        }
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