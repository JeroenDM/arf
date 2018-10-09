# Trajectory point class design

Final goal, create a trajectory point like this.

```c++
// x-value in interval [-1, 1] with nominal value 0
TolerancedNumber x_tol(0, -1, 1);
// Only tolerance on x value, y and z coordinates are just fixed numbers
TrajectoryPoint tp(x_tol, 0.5, 0.6);
```

## Using polymorphism

```c++
class TrajectoryPoint
{
public:
  TrajectoryPoint(Number x, Number y, Number z)
  {
    if (x.hasTolerance())
    {
      // Do toleranced number stuff.
    }
    else
    {
      // Do normal number stuff.
    }
    // Do the same for y and z.
  }
}
```

The achieve this we create a Base class `Number` and a child `TolerancedNumber`.

```c++
class Number
{
  double value_;
public:
  Number(double val) : value_(val) {}
  bool hasTolerance() { return false; }
}

class TolerancedNumber : public Number
{
  double tolerance_;
public:
  Number(double val, double tol) : Number(value), tolerance_(tol) {}
  bool hasTolerance() { return true; }
}
```

To make polymorphism work, we need to use pointers or references to pass values to the constructor of `TrajectoryPoint`.
```c++
class TrajectoryPoint
{
public:
  TrajectoryPoint(Number* x, Number* y, Number* z)
  {
    if ((x->hasTolerance())
    {
      // Do toleranced number stuff.
    }
    else
    {
      // Do normal number stuff.
    }
    // Do the same for y and z.
  }
}
```

We are now arrived at an interface that leaves us with an ugly wrapper class for the build in type `double`.

```c++
TolerancedNumber x_tol(0, -1, 1);
Number y(0.5), z(0.6);
TrajectoryPoint tp(&x_tol, &y, &z);
```

If it where not for the pass by pointer in the `TrajectoryPoint` constructor, the following calls [would be equivalent](https://stackoverflow.com/questions/5756169/conversion-from-built-in-types-to-custom-classes):
```c++
TrajectoryPoint tp(x_tol, Number(0.5), Number(0.6));
TrajectoryPoint tp(x_tol, 0.5, 0.6);
```
because `Number` has a constructor that takes and the compiler makes an implicit conversion to `double`.