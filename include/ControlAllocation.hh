#ifndef CONTROL_ALLOCATION_HH
#define CONTROL_ALLOCATION_HH

class ControlAllocation
{
public:
    ControlAllocation(double minT, double maxT, int numInputs) : 
        _minT(minT), _maxT(maxT), numberInputs(numInputs) {};
    ~ControlAllocation();

    bool isInBounds(double* controlInputs);
    double* shift(const double* controlInputs, int direction, double amount);
    double* scale(const double* controlInputs);
    double* allocateControls(const double* controlInputs);

private:
    double _minT, _maxT;
    int numberInputs;
};

#endif // CONTROL_ALLOCATION_HH