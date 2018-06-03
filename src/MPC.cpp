#include <fstream>
#include <iostream>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "helper_functions.h"
#include "MPC.h"

using CppAD::AD;

namespace CarND {

// The timestep length and duration
size_t N = 10;
double dt = 0.1;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
const size_t kXStart = 0;
const size_t kYStart = kXStart + N;
const size_t kPsiStart = kYStart + N;
const size_t kVStart = kPsiStart + N;
const size_t kCteStart = kVStart + N;
const size_t kEpsiStart = kCteStart + N;
const size_t kDeltaStart = kEpsiStart + N;
const size_t kAStart = kDeltaStart + N - 1;

class FG_eval {
 public:
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs_;
  FG_eval(Eigen::VectorXd coeffs) : coeffs_(coeffs) {
    std::ifstream config_file("params.txt");
    if (config_file.is_open()) {
      
      config_file >>
        reference_speed_ >>
        speed_factor_ >>
        cte_factor_ >>
        epsi_factor_ >>
        delta_factor_ >>
        acceleration_factor_ >>
        delta_ratio_factor_ >>
        acceleration_ratio_factor_;

      config_file.close();
    }
  }

  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable
    // values (state & actuators)
    size_t t;

    //
    // Cost calculation
    //
    // We store cost value in the first index of the fg vector.
    fg[0] = 0;  // Initialize to zero

    // It is important to note that not all calculations share the same scale,
    // so we apply factors to each cost component to make sure the car follows
    // the rules that we place more emphasis on. Ideally though, we should find
    // a more generic solution, like normalizing.

    // Add the cost of the reference state
    for (t = 0; t < N; t++) {
      fg[0] +=
            speed_factor_ * CppAD::pow(vars[kVStart + t] - reference_speed_, 2)
          + cte_factor_ * CppAD::pow(vars[kCteStart + t], 2)
          + epsi_factor_ * CppAD::pow(vars[kEpsiStart + t], 2);
    }

    // Add the cost of using the actuators
    // We actually want to penalize the use of heavy actuator values (steering,
    // accelerating/braking too hard), which is why we apply factors.
    for (t = 0; t < N-1; t++) {
      fg[0] +=
            delta_factor_ * CppAD::pow(vars[kDeltaStart + t], 2)
          + acceleration_factor_ * CppAD::pow(vars[kAStart + t], 2);
    }

    // Add the cost of changes in the actuators
    // Here again, we penalize sharp changes in the actuator values.
    for (t = 0; t < N-2; t++) {
      fg[0] +=
            delta_ratio_factor_ * CppAD::pow(vars[kDeltaStart + t + 1] - vars[kDeltaStart + t], 2)
          + acceleration_ratio_factor_ * CppAD::pow(vars[kAStart + t + 1] - vars[kAStart + t], 2);
    }

    //
    // Constraint equations
    //

    // Initial constraints
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`. This bumps up the position of all the other values.
    fg[1 + kXStart] = vars[kXStart];
    fg[1 + kYStart] = vars[kYStart];
    fg[1 + kPsiStart] = vars[kPsiStart];
    fg[1 + kVStart] = vars[kVStart];
    fg[1 + kCteStart] = vars[kCteStart];
    fg[1 + kEpsiStart] = vars[kEpsiStart];

    // The rest of the constraints
    for (t = 1; t < N; t++) {
      AD<double> x1 = vars[kXStart + t];
      AD<double> y1 = vars[kYStart + t];
      AD<double> psi1 = vars[kPsiStart + t];
      AD<double> v1 = vars[kVStart + t];
      AD<double> cte1 = vars[kCteStart + t];
      AD<double> epsi1 = vars[kEpsiStart + t];

      AD<double> x0 = vars[kXStart + t - 1];
      AD<double> y0 = vars[kYStart + t - 1];
      AD<double> psi0 = vars[kPsiStart + t - 1];
      AD<double> v0 = vars[kVStart + t - 1];
      AD<double> cte0 = vars[kCteStart + t - 1];
      AD<double> epsi0 = vars[kEpsiStart + t - 1];
      AD<double> delta0 = vars[kDeltaStart + t - 1];
      AD<double> a0 = vars[kAStart + t - 1];

      // The expected position obtained by evaluating the current trajectory
      // estimation.
      AD<double> f0 = HelperFunctions::PolyEval(coeffs_, x0);
      // The destination heading, obtained from the tangent of the trajectory's
      // derivative.
      AD<double> psi_des0 = CppAD::atan(HelperFunctions::PolyEval(HelperFunctions::Derivative(coeffs_), x0));

      // Note the use of `AD<double>` and use of `CppAD`!
      // This is so CppAD can compute derivatives and pass these to the solver.

      fg[1 + kXStart + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + kYStart + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + kPsiStart + t] = psi1 - (psi0 + v0 * delta0 * dt / MPC::kLf);
      fg[1 + kVStart + t] = v1 - (v0 + a0 * dt);
      fg[1 + kCteStart + t] = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + kEpsiStart + t] = epsi1 - ((psi0 - psi_des0) - v0 * delta0 * dt / MPC::kLf);
    }
  }

 private:
  double reference_speed_ ;
  double speed_factor_ ;
  double cte_factor_ ;
  double epsi_factor_ ;
  double delta_factor_ ;
  double acceleration_factor_ ;
  double delta_ratio_factor_ ;
  double acceleration_ratio_factor_;
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

MpcSolution
MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Number of independent variables and constraints
  size_t n_state = 6;
  size_t n_actuators = 2;
  size_t n_constraints = N * n_state;
  size_t n_vars = n_constraints + (N - 1) * n_actuators;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set lower and upper limits for variables.

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (i = 0; i < kDeltaStart; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The actuator upper and lower limits are set to -1 and 1
  for (i = kDeltaStart; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[kXStart] = x;
  constraints_lowerbound[kYStart] = y;
  constraints_lowerbound[kPsiStart] = psi;
  constraints_lowerbound[kVStart] = v;
  constraints_lowerbound[kCteStart] = cte;
  constraints_lowerbound[kEpsiStart] = epsi;

  constraints_upperbound[kXStart] = x;
  constraints_upperbound[kYStart] = y;
  constraints_upperbound[kPsiStart] = psi;
  constraints_upperbound[kVStart] = v;
  constraints_upperbound[kCteStart] = cte;
  constraints_upperbound[kEpsiStart] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {
    solution.x[kAStart],
    solution.x[kDeltaStart],
    std::vector<double> (solution.x.data() + kXStart, solution.x.data() + kYStart),
    std::vector<double> (solution.x.data() + kYStart, solution.x.data() + kPsiStart)};
}

}  // namespace CarND
