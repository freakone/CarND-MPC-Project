#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct MPCResult
{
  double throttle;
  double angle;
  std::vector<double> x_prediction;
  std::vector<double> y_prediction;
};

class MPC {
 public:
  MPC();
  virtual ~MPC();

  Eigen::MatrixXd transformCoordinates( const vector<double>& vx, const vector<double>& vy, const double& px, const double& py, const double& psi );
  MPCResult Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  std::vector<double> toStdVector( const Eigen::VectorXd& input);

  static constexpr double Lf = 2.67;
  static constexpr double ref_v = 40;
  static constexpr int N = 18;
  static constexpr double dt = 0.1;
};

#endif /* MPC_H */
