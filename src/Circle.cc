#include "Circle.hh"

double Circle::calcArea()
{
    return PI*_radius*_radius;
}
double Circle::calcCircumference()
{
    return PI*2*_radius;
}