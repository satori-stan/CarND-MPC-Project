#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "helper_functions.h"

using namespace std;

namespace CarND {

struct MpcSolution {
  double acceleration;
  double steering;
  vector<double> x_points;
  vector<double> y_points;
};

class MPC {
 public:
  // This value assumes the model presented in the classroom is used.
  //
  // It was obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on a
  // flat terrain.
  //
  // Lf was tuned until the the radius formed by the simulating the model
  // presented in the classroom matched the previous radius.
  //
  // This is the length from front to CoG that has a similar radius.
  // The distance from the steering axel to the center of gravity
  static constexpr double kLf = 2.67;
  // The maximum angle (in radians) that the steering may go (25° in radians).
  static constexpr double kMaxSteerAngle = 0.43633231299858239423092269212215;

  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  //vector<double>
  MpcSolution
  Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

}  // namespace CarND

#endif /* MPC_H */
