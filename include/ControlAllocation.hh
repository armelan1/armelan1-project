#ifndef CONTROL_ALLOCATION_HH
#define CONTROL_ALLOCATION_HH

class ControlAllocation
{
public:
    ControlAllocation(double minT, double maxT, int numInputs) : 
        _minT(minT), _maxT(maxT), _numberInputs(numInputs) {};
    ~ControlAllocation();

    void allocateControls(const double* controlInputs, double* controlOutputs);

private:
    int _numberInputs;
    double _minT;
    double _maxT;
    
    bool isInBounds(const double* controlInputs);
    void shift(const double* controlInputs, double* controlOutputs, 
               int direction, double amount);
    void scale(const double* controlInputs, double* controlOutputs);
    
    double getMin(const double* array) {
        double minVal = array[0];
        for (int i = 1; i < _numberInputs; i++) {
            if (array[i] < minVal) {
                minVal = array[i];
            }
        }
        return minVal;
    }
    
    double getMax(const double* array) {
        double maxVal = array[0];
        for (int i = 1; i < _numberInputs; i++) {
            if (array[i] > maxVal) {
                maxVal = array[i];
            }
        }
        return maxVal;
    }
};

#endif // CONTROL_ALLOCATION_HH