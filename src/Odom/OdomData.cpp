//created by zzh
#include "OdomData.h"

namespace VIEO_SLAM{

using namespace Eigen;

double IMUDataBase::mdMultiplyG=1.0,IMUDataBase::mdRefG=9.810;//u can change refG here
Matrix3d IMUDataBase::mSigmag=Matrix3d::Identity(),IMUDataBase::mSigmaa=Matrix3d::Identity();
Matrix3d IMUDataBase::mSigmabg=Matrix3d::Identity(),IMUDataBase::mSigmaba=Matrix3d::Identity();
double IMUDataBase::mInvSigmabg2=1.,IMUDataBase::mInvSigmaba2=1.;
Matrix3d IMUDataDerived::mSigmaI(Matrix3d::Identity());
int IMUDataBase::mdt_cov_noise_fixed = 0;
double IMUDataBase::mFreqRef = 0;

Matrix3d IMUDataDerived::skew(const Vector3d&v)
{
  Matrix3d m;
  m.fill(0.);
  m(0,1)  = -v(2);
  m(0,2)  =  v(1);
  m(1,2)  = -v(0);
  m(1,0)  =  v(2);
  m(2,0) = -v(1);
  m(2,1) = v(0);
  return m;
}
Matrix3d IMUDataDerived::getJacoright(){
  AngleAxisd angaxi=AngleAxisd(Eigen::Quaterniond(quat));//angleaxisi need explicit conversion!
  double th=angaxi.angle();
  Matrix3d skewa=skew(angaxi.axis());
  if (th<1E-5){
    return (Matrix3d::Identity()-th*skewa/2);
  }
  return (Matrix3d::Identity()-(1-cos(th))/th*skewa+(1-sin(th)/th)*skewa*skewa);
}

double EncData::mvscale=1,EncData::mrc=1;
Matrix2d EncData::mSigma=Matrix2d::Identity();
Matrix6d EncData::mSigmam=Matrix6d::Identity();
int EncData::mdt_cov_noise_fixed = 0;
double EncData::mFreqRef = 0;
EncData::EncData(const double* v,const double &tm):mtm(tm){
  mv[0]=v[0]*mvscale;mv[1]=v[1]*mvscale;
}

}
