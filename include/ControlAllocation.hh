#ifndef CONTROL_ALLOCATION_HH
#define CONTROL_ALLOCATION_HH

// grab this from a common header file if it exists
#define FLOATING_POINT_TOLERANCE 1e-6
#define ROUNDING_TOLERANCE 1e-2 // check this against specs?

class ControlAllocation
{
public:
    ControlAllocation(double minT, double maxT, int numInputs) : 
        _minT(minT), _maxT(maxT), _numberInputs(numInputs) {};
    // ~ControlAllocation();

    double getMinT(){
        return _minT;
    };
    double getMaxT(){
        return _maxT;
    };
    void setMinT(double minT){
        _minT = minT;
    };
    void setMaxT(double maxT){
        _maxT = maxT;
    };
    int getNumberInputs(){
        return _numberInputs;
    };
    void setNumberInputs(int numInputs){
        _numberInputs = numInputs;
    };

    void allocateControls(double* controlInputs, double* controlOutputs);

private:
    double _minT;
    double _maxT;
    int _numberInputs;

    enum class Direction : int {
        POSITIVE = 1,
        NEGATIVE = -1,
        COPY = 0
    };
    
    bool isInBounds(double* controlInputs);
    void shift(const double* controlInputs, double* controlOutputs, 
               Direction direction, double amount);
    void scale(const double* controlInputs, double* controlOutputs);
    void shiftAfterScale(double* controlOutputs);
    
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