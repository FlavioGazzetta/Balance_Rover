#include "mpc_2wheel.h"

MPCPlannerBalance::MPCPlannerBalance(double dt, int horizon)
  : dt_(dt), N_(horizon)
{
  mpc_.dt = dt_;
  mpc_.horizon = N_;
  // default reference = zero
  x_ref_ = Eigen::VectorXd::Zero(4);
  buildModel();
}

void MPCPlannerBalance::setReference(const Eigen::VectorXd& x_ref) {
  x_ref_ = x_ref;
}

bool MPCPlannerBalance::solve(const Eigen::VectorXd& x0, double& u_opt) {
  // Build/updat e prediction, cost, constraints
  buildPrediction();
  buildCost();
  buildConstraints(x0);

  // setup solver
  solver_.settings()->setVerbosity(0);
  solver_.settings()->setWarmStart(true);
  solver_.data()->setNumberOfVariables(mpc_.H_sparse.rows());
  solver_.data()->setNumberOfConstraints(mpc_.A_sparse.rows());
  solver_.data()->setHessianMatrix(mpc_.H_sparse);
  solver_.data()->setGradient(mpc_.f);
  solver_.data()->setLinearConstraintsMatrix(mpc_.A_sparse);
  solver_.data()->setLowerBound(mpc_.lowerBound);
  solver_.data()->setUpperBound(mpc_.upperBound);

  if (!solver_.initSolver()) {
    ROS_ERROR("[MPC] solver init failed");
    return false;
  }
  if (solver_.solve() != OsqpEigen::ErrorExitFlag::NoError) {
    ROS_ERROR("[MPC] solver solve failed");
    return false;
  }
  Eigen::VectorXd sol = solver_.getSolution();
  u_opt = sol(0);  // first control action
  return true;
}

void MPCPlannerBalance::buildModel() {
  // Continuous-time LTI model linearized about upright
  // x = [theta; theta_dot; p; p_dot]; u = torque
  double m = 0.8, l = 0.1, I = 0.005, b = 0.1, g = 9.81;
  Eigen::MatrixXd A_c(4,4), B_c(4,1);
  A_c << 0, 1, 0, 0,
         m*g*l/I, -b/I, 0, 0,
         0, 0, 0, 1,
         0, 0, 0, -b/m;
  B_c << 0,
         1.0/I,
         0,
         1.0/m;
  // Discretize (Euler)
  mpc_.A = Eigen::MatrixXd::Identity(4,4) + A_c * dt_;
  mpc_.B = B_c * dt_;
}

void MPCPlannerBalance::buildPrediction() {
  // Build M, C for horizon N_
  int n = 4;
  mpc_.M.resize(n*N_, n);
  mpc_.C.resize(n*N_, N_);
  mpc_.M.setZero();
  mpc_.C.setZero();
  Eigen::MatrixXd A_pow = Eigen::MatrixXd::Identity(n,n);
  for (int i=0; i<N_; ++i) {
    A_pow = (i==0 ? mpc_.A : mpc_.A * A_pow);
    mpc_.M.block(i*n, 0, n, n) = A_pow;
    for (int j=0; j<=i; ++j) {
      Eigen::MatrixXd A_j = (j==0 ? Eigen::MatrixXd::Identity(n,n)
                         : mpc_.A.pow(j));
      mpc_.C.block(i*n, j, n, 1) = A_j * mpc_.B;
    }
  }
}

void MPCPlannerBalance::buildCost() {
  int n = 4, m = 1;
  // Q and R
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n,n);
  Q(0,0)=100; Q(1,1)=10; Q(2,2)=1; Q(3,3)=1;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m,m)*0.1;
  // Build Q_bar, R_bar
  mpc_.Q_bar = Eigen::MatrixXd::Zero(n*N_, n*N_);
  for (int i=0; i<N_; ++i) {
    mpc_.Q_bar.block(i*n, i*n, n, n) = (i==N_-1 ? Q*10 : Q);
  }
  mpc_.R_bar = Eigen::MatrixXd::Identity(N_,N_) * R(0,0);
  // Hessian H = C^T Q_bar C + R_bar
  mpc_.H = mpc_.C.transpose() * mpc_.Q_bar * mpc_.C + mpc_.R_bar;
  mpc_.H_sparse = mpc_.H.sparseView();
  // gradient f = 2 C^T Q_bar (M x0 - X_ref_big)
  Eigen::VectorXd Xr = Eigen::VectorXd::Zero(n*N_);
  for (int i=0; i<N_; ++i)
    Xr.segment(i*n, n) = x_ref_;
  // f = (M x0 - Xr)^T Q_bar C -> transpose
  // compute later in buildConstraints when x0 known
}

void MPCPlannerBalance::buildConstraints(const Eigen::VectorXd& x0) {
  int n=4;
  // gradient f
  Eigen::VectorXd Xr = Eigen::VectorXd::Zero(n*N_);
  for (int i=0; i<N_; ++i)
    Xr.segment(i*n, n) = x_ref_;
  Eigen::VectorXd Mx0 = mpc_.M * x0;
  mpc_.f = (mpc_.C.transpose() * mpc_.Q_bar * (Mx0 - Xr));
  // input bounds
  double umax = 2.0;
  mpc_.lowerBound = Eigen::VectorXd::Constant(N_, -umax);
  mpc_.upperBound = Eigen::VectorXd::Constant(N_, +umax);
  // no state constraints
  mpc_.A_sys = Eigen::MatrixXd::Identity(N_, N_);
  mpc_.A_sparse = mpc_.A_sys.sparseView();
}