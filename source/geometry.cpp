#include "geometry.h"

Point::Point(Quantity<Units::length> x,
             Quantity<Units::length> y,
             Quantity<Units::length> z):
  x{x},
  y{y},
  z{z}
{}