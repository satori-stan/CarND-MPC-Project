#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

namespace CarND {

class HelperFunctions {
 public:
  // Evaluate a polynomial.
  template <typename T>
  static inline T polyeval(Eigen::VectorXd coeffs, T x) {
    T result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
      result += coeffs[i] * pow(x, i);
    }
    return result;
  }

  // Fit a polynomial.
  // Adapted from
  // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
  static inline Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                          int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
      A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
      for (int i = 0; i < order; i++) {
        A(j, i + 1) = A(j, i) * xvals(j);
      }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
  }
  
  /**
   * Calculate the derivative of our polinomial coefficients
   * @returns A vector of coefficients consistent with the derivative
   */
  static inline Eigen::VectorXd derivative(Eigen::VectorXd coeffs) {
    Eigen::VectorXd derivative(coeffs.size() - 1);
    for (int i = 1; i < coeffs.size(); ++i) {
      derivative[i - 1] = coeffs[i] * i;
    }
    return derivative;
  }

};

}  // namespace CarND

#endif  // HELPER_FUNCTIONS_H_
