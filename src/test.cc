#include <iostream>
#include <vector>

int main() 
{
    std::vector<int> myVector = {1,2,3,5,8,13,21};
    std::cout << "Hello, World!" << std::endl;

    for (auto i : myVector)
    {
        std::cout << i << std::endl;
    }
    return 0;
}