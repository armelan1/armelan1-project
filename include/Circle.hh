#ifndef CIRCLE_HH
#define CIRCLE_HH

#define PI 3.14159

class Circle
{
    public:
        Circle(double radius) : _radius(radius) {};

        double calcArea();
        double calcCircumference();

    private:
        double _radius;
};


#endif // CIRCLE_HH