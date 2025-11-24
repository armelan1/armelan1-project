#include <iostream>
#include <vector>
#include "Circle.hh"
#include "Rectangle.hh"
#include <memory>

namespace my_test
{
    class namespaceTestClass
    {
        public:
            namespaceTestClass(int counter) : _counter(counter) {};

            int getCounter()
            {
                return _counter;
            }

            void setCounter(int c)
            {
                _counter = c;
            }

            void operator++(int)
            {
                _counter++;
            }
        
        private:
            int _counter;
    };

    void printCounter(namespaceTestClass* inputClass)
    {
        std::cout << "counter: " << inputClass->getCounter() << std::endl;
    }
}

int main() 
{
    std::vector<int> myVector = {1,2,3,5,8,13,21};
    std::cout << "Hello, World!" << std::endl;
    std::cout << "\n";

    for (auto i : myVector)
    {
        std::cout << i << std::endl;
    }
    std::cout << "\n";

    Circle myCircle(1.0);
    double myCircleArea = myCircle.calcArea();
    double myCircleCircumference = myCircle.calcCircumference();

    std::cout << "Circle Circumference: " << myCircleCircumference << std::endl;
    std::cout << "Circle Area: " << myCircleArea << std::endl;
    std::cout << "\n";

    Rectangle myRectangle(2.0, 3.0);
    std::cout << "Rectangle Area: " << myRectangle.calcArea() << std::endl;
    std::cout << "\n";

    std::vector<std::unique_ptr<Shape>> shapes;
    shapes.push_back(std::make_unique<Circle>(2.0));
    shapes.push_back(std::make_unique<Rectangle>(3.0, 4.0));

    double shapeCircleArea = shapes[0]->calcArea();
    double shapeRectangleArea = shapes[1]->calcArea();
    double totalArea = shapeCircleArea + shapeRectangleArea;

    std::cout << "shape circle area: " << shapeCircleArea << std::endl;
    std::cout << "shape rectangle area: " << shapeRectangleArea << std::endl;
    std::cout << "shape total area: " << totalArea << std::endl;
    std::cout << "\n";

    double calcTotalArea = 0;
    for (auto &shape : shapes)
    {
        double shapeArea = shape->calcArea();
        std::cout << "calculating " + shape->name() + " area: " << shapeArea << std::endl;
        calcTotalArea += shapeArea;
    }

    std::cout << "total area: " << calcTotalArea << std::endl;
    std::cout << "\n";
    
    my_test::namespaceTestClass myNamespaceTestClass(0);
    int trackCounter = myNamespaceTestClass.getCounter();
    std::cout << "initial counter: " << trackCounter << std::endl;
    std::cout << "incrementing counter" << std::endl;
    my_test::printCounter(&myNamespaceTestClass);
    myNamespaceTestClass++;
    my_test::printCounter(&myNamespaceTestClass);

    my_test::namespaceTestClass newClass(1);
    trackCounter = newClass.getCounter();
    std::cout << "initial counter: " << trackCounter << std::endl;
    std::cout << "incrementing counter" << std::endl;
    my_test::printCounter(&newClass);
    newClass++;
    my_test::printCounter(&newClass);
    std::cout << "ending namespace test" << std::endl;

    return 0;
}