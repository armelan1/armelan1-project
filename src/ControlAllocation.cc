#include "ControlAllocation.hh"
#include <iostream>

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
                               Direction direction, double amount) {
    for (int i = 0; i < _numberInputs; i++) {
        controlOutputs[i] = controlInputs[i] + (static_cast<int>(direction) * amount);
    }
}

void ControlAllocation::scale(const double* controlInputs, double* controlOutputs) {
    double maxInput = getMax(controlInputs);
    double minInput = getMin(controlInputs);
    
    for (int i = 0; i < _numberInputs; i++) {
        double normalized = (controlInputs[i] - minInput) / (maxInput - minInput);
        controlOutputs[i] = normalized * (_maxT - _minT) + _minT;
    }
}
// void ControlAllocation::roundToZero(double* controlOutputs) {
//     for (int i = 0; i < _numberInputs; i++) {
//         if (controlOutputs[i] < ROUNDING_TOLERANCE) {
//             controlOutputs[i] = 0.0;
//         }
//     }
// }
void ControlAllocation::shiftAfterScale(double* controlOutputs) {
    double tempInputs[_numberInputs];
    shift(controlOutputs, tempInputs, Direction::COPY, 0.0); // copy outputs to tempInputs
    shift(tempInputs, controlOutputs, Direction::NEGATIVE, _minT);
    // roundToZero(controlOutputs);
    if (isInBounds(controlOutputs))
        return;
    shift(tempInputs, controlOutputs, Direction::COPY, 0.0); // copy tempInputs to controlOutputs
}


void ControlAllocation::allocateControls(const double* controlInputs, 
                                         double* controlOutputs) {
    double maxInput = getMax(controlInputs);
    double minInput = getMin(controlInputs);

    if ((maxInput - minInput) > _maxT) {  // maxInput - minInput < _maxT - _minT handled at end
        scale(controlInputs, controlOutputs);
        shiftAfterScale(controlOutputs);
        return;
    }
    // check to see if we can shift the min to conserve thrust
    if (isInBounds(controlInputs)) {
        if (minInput < FLOATING_POINT_TOLERANCE) {
            shift(controlInputs, controlOutputs, Direction::COPY, 0.0); // copy inputs to outputs
            return; // already in bounds and minInput is zero
        }

        // try shifting minInput to zero
        shift(controlInputs, controlOutputs, Direction::NEGATIVE, minInput);
        if (isInBounds(controlOutputs)) {
            return;
        }
        // try shifting minInput to minT
        shift(controlInputs, controlOutputs, Direction::NEGATIVE, minInput - _minT);
        if (isInBounds(controlOutputs)) {
            return;
        }
    }

    if (minInput < 0) {
        shift(controlInputs, controlOutputs, Direction::POSITIVE, -minInput); // shift minInput up to zero
        if (isInBounds(controlOutputs))
            return;
        shift(controlInputs, controlOutputs, Direction::POSITIVE, -(minInput - _minT)); // shift minInput up to minT
        if (isInBounds(controlOutputs))
            return;
    }else if (maxInput > _maxT) {
        shift(controlInputs, controlOutputs, Direction::NEGATIVE, maxInput - _maxT); // shift maxInput down to maxT
        if (isInBounds(controlOutputs))
            return;
        shift(controlInputs, controlOutputs, Direction::NEGATIVE, (maxInput - _maxT) + (minInput - _minT)); // shift maxInput below maxT
        if (isInBounds(controlOutputs))
            return;
    }else { // input in deadband
        // reduce thrust
        shift(controlInputs, controlOutputs, Direction::NEGATIVE, minInput); // shift minInput down to zero
        if(isInBounds(controlOutputs))
            return;
        // increase thrust
        shift(controlInputs, controlOutputs, Direction::POSITIVE, _minT - minInput); // shift minInput up to minT
        if(isInBounds(controlOutputs))
            return;
    }
    // edge case maxInput - minInput < _maxT - _minT
    scale(controlInputs, controlOutputs);
    shiftAfterScale(controlOutputs);
}