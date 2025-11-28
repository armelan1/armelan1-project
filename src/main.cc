#include <iostream>
#include "ControlAllocation.hh"


int main() 
{

    ControlAllocation myCtrlAllocator(30.0, 300.0, 4);

    // double desiredControls[4] = {330.0, 30.0, 30.0, 30.0};
    // double desiredControls[4] = {330.0, 20.0, 30.0, 30.0};
    // double desiredControls[4] = {330.0, 40.0, 100.0, 100.0};
    // double desiredControls[4] = {270.0, -30.0, 100.0, 100.0};
    // double desiredControls[4] = {280.0, -30.0, 100.0, 100.0};
    // double desiredControls[4] = {280.0, 30.0, 100.0, 100.0};
    // double desiredControls[4] = {280.0, 30.0, 40.0, 100.0};
    // double desiredControls[4] = {280.0, 0.000000001, 40.0, 100.0};
    // double desiredControls[4] = {100.0, 200.0, 305.0, 300.0};
    // double desiredControls[4] = {100.0, 100.0, 100.0, 100.0};
    double desiredControls[4] = {40.0, 40.0, 100.0, 330.0};
    double allocatedControls[4];  // Stack allocation

    myCtrlAllocator.allocateControls(desiredControls, allocatedControls);
    std::cout << "Allocated Controls: ";
    for (int i = 0; i < 4; i++) {
        std::cout << allocatedControls[i] << " ";
    }
    std::cout << std::endl;

    return 0;
}