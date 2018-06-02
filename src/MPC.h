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
