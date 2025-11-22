#ifndef SHAPE_HH
#define SHAPE_HH


class Shape
{
    public:

        virtual ~Shape() = default;

        virtual double calcPerimeter() const = 0;
        virtual double calcArea() const = 0;
};

#endif // SHAPE_HH