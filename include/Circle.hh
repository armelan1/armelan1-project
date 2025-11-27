#ifndef CIRCLE_HH
#define CIRCLE_HH

#include "Shape.hh"
#include <cmath>

class Circle : public Shape
{
    public:
        explicit Circle(double radius) : _radius(radius) {};

        double calcArea() const override;
        double calcPerimeter() const override;
        double calcCircumference();
        std::string name() const override;

    private:
        double _radius;
};


#endif // CIRCLE_HH