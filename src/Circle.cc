#include "Circle.hh"

double Circle::calcArea() const
{
    return M_PI * _radius * _radius;
}

double Circle::calcPerimeter() const
{
    return M_PI * 2 * _radius;
}

double Circle::calcCircumference()
{
    return calcPerimeter();
}

std::string Circle::name() const
{
    return "Circle";
}