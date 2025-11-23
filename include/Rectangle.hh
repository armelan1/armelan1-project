#ifndef RECTANGLE_HH
#define RECTANGLE_HH

#include "Shape.hh"

class Rectangle : public Shape
{
    public:
        explicit Rectangle(double length, double width) : _length(length), _width(width) {};

        double calcArea() const override;
        double calcPerimeter() const override;
        std::string name() const override;

    private:
        double _length;
        double _width;
};

#endif // RECTANGLE_HH