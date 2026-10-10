// Independent Ceres covariance check: a monocular single view has free depth.
// This validates the diagnosis's geometry, not the particular native failure.
#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include <iostream>
struct Projection {
  double baseline;
  template<class T> bool operator()(const T* p,T* residual) const {
    residual[0]=(p[0]-T(baseline))/p[2]-T((.3-baseline)/3.);
    residual[1]=p[1]/p[2]-T(.4/3.);
    return true;
  }
};
bool recover(int views) {
  double point[3]={.3,.4,3.};ceres::Problem problem;
  for(int i=0;i<views;++i)
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Projection,2,3>(new Projection{.2*i}),nullptr,point);
  ceres::Covariance::Options options;options.min_reciprocal_condition_number=1e-15;
  ceres::Covariance covariance(options);
  std::vector<std::pair<const double*,const double*>> blocks{{point,point}};
  if(!covariance.Compute(blocks,&problem))return false;
  double block[9];covariance.GetCovarianceBlock(point,point,block);
  Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>> matrix(block);
  return matrix.allFinite()&&Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(matrix).eigenvalues().minCoeff()>0;
}
int main() {
  if(recover(1)||!recover(2))return 2;
  std::cout<<"PASS: same Ceres covariance rejects single-view depth degeneracy and recovers finite positive covariance with two translated views\n";
}
