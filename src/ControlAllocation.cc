#include "ControlAllocation.hh"
#include <algorithm>

bool ControlAllocation::isInBounds(double* controlInputs) {
    for (int i = 0; i < _numberInputs; i++) {
        if ((controlInputs[i] < 0) || 
            ((controlInputs[i] < _minT) && (controlInputs[i] > 0)) || 
            controlInputs[i] > _maxT) {
            return false;
        }
    }
    return true;
}

double* ControlAllocation::shift(const double* controlInputs, int direction, double amount) {
    double* controlOutputs = new double[_numberInputs];
    for (int i = 0; i < _numberInputs; i++) {
        controlOutputs[i] = controlInputs[i] + (direction * amount);
    }
    return controlOutputs;
}

double* ControlAllocation::scale(const double* controlInputs) {
    double* controlOutputs = new double[_numberInputs];
    double maxInput = *std::max_element(controlInputs, controlInputs + _numberInputs);
    double minInput = *std::min_element(controlInputs, controlInputs + _numberInputs);
    for (size_t i = 0; i < n; ++i)
    {
        double normalized = (controlInputs[i] - minInput) / (maxInput - minInput);  // [0,1]
        controlOutputs[i] = normalized * (_maxT - _minT) + _minT; // [a,b]
    }
    return controlOutputs;
}

double* ControlAllocation::allocateControls(const double* controlInputs) {
    double* controlOutputs = new double[_numberInputs];
    double maxInput = *std::max_element(controlInputs, controlInputs + _numberInputs);
    double minInput = *std::min_element(controlInputs, controlInputs + _numberInputs);

    if ((maxInput - minInput) > _maxT) {
        controlOutputs = scale(controlInputs);
        return controlOutputs;
    }

    if (minInput < 0) {
        controlOutputs = shift(controlInputs, 1, -minInput);
        if (isInBounds(controlOutputs))
            return controlOutputs;
        controlOutputs = shift(controlInputs, 1, -(minInput - _minT));
        if (isInBounds(controlOutputs))
            return controlOutputs;
    }else if (maxInput > _maxT) {
        controlOutputs = shift(controlInputs, -1, maxInput - _maxT);
        if (isInBounds(controlOutputs))
            return controlOutputs;
        controlOutputs = shift(controlInputs, -1, (maxInput - _maxT) + (minInput - _minT));
        if (isInBounds(controlOutputs))
            return controlOutputs;
    }
    controlOutputs = scale(controlInputs);
    return controlOutputs;
}