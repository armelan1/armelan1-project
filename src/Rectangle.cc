#include "Rectangle.hh"

double Rectangle::calcArea() const
{
    return _length * _width;
}

double Rectangle::calcPerimeter() const
{
    return 2 * (_length * _width);
}

std::string Rectangle::name() const
{
    return "Rectangle";
}