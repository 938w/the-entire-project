
#include "vex.h"

using namespace vex;

class pid2 {
public:
  void drive2(double target, double velocity = 12000, double dir = 999);
  void drive(double target, double velocity, double slowDistance = 16, double dir = 999);
  void turnToHeading(double heading);
};