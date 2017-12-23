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
};

#endif /* MPC_H */
