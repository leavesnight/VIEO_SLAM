// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_SIM_3
#define G2O_SIM_3

#include "se3_ops.h"
#include <Eigen/Geometry>

namespace g2o
{
  using namespace Eigen;

  typedef  Matrix <double, 7, 1> Vector7d;
  typedef  Matrix <double, 7, 7> Matrix7d;
  
  //like SE3Quat(stored data use 7*1), has r&&t, but Sim3(stored data use 8*1) adds s(double)
  struct Sim3
  {
  protected:
    Quaterniond r;//internal::traits<Quaternion> must have a template class like "template<class T> internal::traits{};" and \
    template<typename _Scalar,int _Options> struct traits<Quaternion<_Scalar,_Options> > is a partial specialized template class of it
    Vector3d t;
    double s;


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Sim3()
    {
      r.setIdentity();
      t.fill(0.);
      s=1.;
    }

    Sim3(const Quaterniond & r, const Vector3d & t, double s)
      : r(r),t(t),s(s)
    {
    }

    Sim3(const Matrix3d & R, const Vector3d & t, double s)
      : r(Quaterniond(R)),t(t),s(s)
    {
    }


    Sim3(const Vector7d & update)//same as exp(Vector7d)
    {

      Vector3d omega;
      for (int i=0; i<3; i++)
        omega[i]=update[i];//phi=theta*a,rotation first in g2o

      Vector3d upsilon;
      for (int i=0; i<3; i++)
        upsilon[i]=update[i+3];//p, translation second in g2o

      double sigma = update[6];//sigma
      double theta = omega.norm();
      Matrix3d Omega = skew(omega);//phi^
      s = std::exp(sigma);//s=e^(sigma)
      Matrix3d Omega2 = Omega*Omega;//a^a^=a*a.t()-I;here is Omega2=theta^2*a^a^
      Matrix3d I;
      I.setIdentity();//I
      Matrix3d R;

      double eps = 0.00001;//1E-5
      double A,B,C;
      if (fabs(sigma)<eps)//if sigma is very small, for RGBD/Monocular
      {
        C = 1;//C=(s-1)/sigma=limit(sigma->0)((e^sigma-1)/sigma)=limit(sigma->0)(e^sigma/1)=1
        if (theta<eps)//if theta(angleaxis' angle>=0) is very small
        {
          A = 1./2.;//A~=(omit sigma&&less)=(1-cos(theta))/theta^2~=(omit theta^2&&less)=1/2
          B = 1./6.;//B~=(omit sigma&&less)=(theta-sin(theta))/theta^3~=(omit theta^3&&less)=1/3!=1/6
          R = (I + Omega + Omega2/2);//R=I+(1-cos(theta))*a^a^+sin(theta)*a^~=(omit O(theta^3))=I+theta^2/2*a^a^+theta*a^, Omega2/2 rectified by zzh
        }
        else
        {
          double theta2= theta*theta;
          A = (1-cos(theta))/(theta2);//A=(sigma*s*sin(theta)+(1-s*cos(theta))*theta)/(sigma^2+theta^2)/theta~=(omit sigma&&less)=(1-cos(theta))/theta^2
          B = (theta-sin(theta))/(theta2*theta);//B=[1-((s*cos(theta)-1)*sigma+s*sin(theta)*theta)/(sigma^2+theta^2)]/theta^2~=(omit sigma&&less)=(theta-sin(theta))/theta^3
          R = I + sin(theta)/theta *Omega + (1-cos(theta))/(theta*theta)*Omega2;//R=I+(1-cos(theta))*a^a^+sin(theta)*a^
        }
      }
      else//sigma is large/C!~=1 for Monocular
      {
        C=(s-1)/sigma;
        if (theta<eps)//if theta is very small<<1
        {
          double sigma2= sigma*sigma;
          A = ((sigma-1)*s+1)/sigma2;//A=(sigma*s*sin(theta)+(1-s*cos(theta))*theta)/(sigma^2+theta^2)/theta~=(omit theta^2&&less)=(sigma*s+(1-s))/(sigma^2)=((sigma-1)*s+1)/sigma2
          B= ((0.5*sigma2-sigma+1)*s-1)/(sigma2*sigma);//B=[C-((s*cos(theta)-1)*sigma+s*sin(theta)*theta)/(sigma^2+theta^2)]/theta^2~=(omit O(theta^2))=\
	  (1/2*s*sigma-s)/(sigma^2)+[C-(s-1)*sigma/(sigma^2+theta^2)]/theta^2~=(0.5*sigma^2*s-s*sigma)/sigma^3+[s-1]/sigma^3=[s*(0.5*sigma^2-sigma+1)-1]/sigma^3,\
	  -1 rectified by zzh
          R = (I + Omega + Omega2/2);//R=I+(1-cos(theta))*a^a^+sin(theta)*a^~=I+theta^2/2*a^a^+theta*a^, /2 rectified by zzh
        }
        else
        {
          R = I + sin(theta)/theta *Omega + (1-cos(theta))/(theta*theta)*Omega2;//R=exp(phi^)=cos(theta)*I+(1-cos(theta))*a*a.t()+sin(theta)*a^=I+(1-cos(theta))*a^a^+sin(theta)*a^

          double a=s*sin(theta);
          double b=s*cos(theta);
          double theta2= theta*theta;
          double sigma2= sigma*sigma;

          double c=theta2+sigma2;
          A = (a*sigma+ (1-b)*theta)/(theta*c);//A=(sigma*s*sin(theta)+(1-s*cos(theta))*theta)/(sigma^2+theta^2)/theta
          B = (C-((b-1)*sigma+a*theta)/(c))*1./(theta2);//B=[C-((s*cos(theta)-1)*sigma+s*sin(theta)*theta)/(sigma^2+theta^2)]/theta^2
        }
      }
      r = Quaterniond(R);

      //C=(s-1)/sigma,A=(sigma*s*sin(theta)+(1-s*cos(theta))*theta)/(sigma^2+theta^2)/theta,B=[C-((s*cos(theta)-1)*sigma+s*sin(theta)*theta)/(sigma^2+theta^2)]/theta^2
      Matrix3d W = A*Omega + B*Omega2 + C*I;//Js=C*I+A*(theta*a^)+B*(theta^2*a^a^)
      t = W*upsilon;//Js*p
    }

     Vector3d map (const Vector3d& xyz) const {
      return s*(r*xyz) + t;
    }//X3d_mapped=[sR|t]*X3d;or X1=(S12*[X2|1])(0:2)

    Vector7d log() const
    {
      Vector7d res;
      double sigma = std::log(s);//sigma=log(s)=log(e^sigma)

      

   
      Vector3d omega;//phi
      Vector3d upsilon;//p


      Matrix3d R = r.toRotationMatrix();
      double d =  0.5*(R(0,0)+R(1,1)+R(2,2)-1);//(tr(R)-1)/2=cos(theta)

      Matrix3d Omega;//phi^

      double eps = 0.00001;
      Matrix3d I = Matrix3d::Identity();//I

      double A,B,C;
      if (fabs(sigma)<eps)//if sigma<<1 for RGBD/Monocular
      {
        C = 1;
        if (d>1-eps)
        {
          omega=0.5*deltaR(R);
          Omega = skew(omega);
          A = 1./2.;
          B = 1./6.;
        }
        else
        {
          double theta = acos(d);
          double theta2 = theta*theta;
          omega = theta/(2*sqrt(1-d*d))*deltaR(R);
          Omega = skew(omega);
          A = (1-cos(theta))/(theta2);
          B = (theta-sin(theta))/(theta2*theta);
        }
      }
      else//sigma not small for Monocular
      {
        C=(s-1)/sigma;
        if (d>1-eps)//theta <<1
        {

          double sigma2 = sigma*sigma;
          omega=0.5*deltaR(R);
          Omega = skew(omega);
          A = ((sigma-1)*s+1)/(sigma2);
          B = ((0.5*sigma2-sigma+1)*s-1)/(sigma2*sigma);//use limit(theta->0)(B)=limit(theta->0){[(sigma2+theta2)*(s*sigma*sin(theta)-s*sin(theta)-s*theta*cos(theta))+(s*cos(theta)*sigma-sigma+s*sin(theta)*theta)*2*theta]/(2*theta)}=\
	    =limit(theta->0)(s*sigma-s)*sin(theta)/(2*(sigma2+theta2)*theta)+limit(theta->0)[-s*cos(theta)/(2*(sigma2+theta2))+(s*cos(theta)*sigma-sigma+s*sin(theta)*theta)/(sigma2+theta2)^2]=\
	    =limit(theta->0)(s*sigma-s)*cos(theta)/(2*(sigma2+3*theta2))+-s/(2*sigma2)+(s-1)/sigma^3=\
	    =(s*sigma-s)/2/sigma2-s/2/sigma2+(s-1)/sigma^3=[(0.5*sigma2-sigma+1)*s-1]/sigma^3, \\
	    -1 rectified by zzh
        }
        else
        {
          double theta = acos(d);
          omega = theta/(2*sqrt(1-d*d))*deltaR(R);//R*a=a;(R-R.t())(vee/anti^)=(I-I)(vee)+sin(theta)/theta*(phi-(-phi))+(1-cos(theta))/theta^2*(phi^phi^-phi^phi^)(vee)=phi*2*sin(theta)/theta \
	  => phi=theta*a=(notice theta>=0)=theta/(2*sin(theta))*(R-R.t())(vee)
          Omega = skew(omega);
          double theta2 = theta*theta;
          double a=s*sin(theta);
          double b=s*cos(theta);
          double c=theta2 + sigma*sigma;
          A = (a*sigma+ (1-b)*theta)/(theta*c);
          B = (C-((b-1)*sigma+a*theta)/(c))*1./(theta2);
        }
      }

      Matrix3d W = A*Omega + B*Omega*Omega + C*I;//Js

      upsilon = W.lu().solve(t);//t=Js*p, like p=Js^(-1)*t


      for (int i=0; i<3; i++)
        res[i] = omega[i];//phi first in g2o

       for (int i=0; i<3; i++)
        res[i+3] = upsilon[i];

      res[6] = sigma;

      return res;
      
    }


    Sim3 inverse() const
    {
      return Sim3(r.conjugate(), r.conjugate()*((-1./s)*t), 1./s);//[sR|t].inverse()=[1/s*R.t() | -(1/s*R.t())*t]
    }
    

    double operator[](int i) const
    {
      assert(i<8);
      if (i<4){

        return r.coeffs()[i];
      }
      if (i<7){
        return t[i-4];
      }
      return s;
    }

    double& operator[](int i)
    {
      assert(i<8);
      if (i<4){

        return r.coeffs()[i];
      }
      if (i<7)
      {
        return t[i-4];
      }
      return s;
    }

    Sim3 operator *(const Sim3& other) const {//S'=S1*S2
      Sim3 ret;
      ret.r = r*other.r;//R'=R1*R2
      ret.t=s*(r*other.t)+t;//t'=s1R1t2+t1
      ret.s=s*other.s;//s'=s1*s2
      return ret;
    }

    Sim3& operator *=(const Sim3& other){
      Sim3 ret=(*this)*other;
      *this=ret;
      return *this;
    }

    inline const Vector3d& translation() const {return t;}

    inline Vector3d& translation() {return t;}

    inline const Quaterniond& rotation() const {return r;}

    inline Quaterniond& rotation() {return r;}

    inline const double& scale() const {return s;}

    inline double& scale() {return s;}

  };

  inline std::ostream& operator <<(std::ostream& out_str,
                                   const Sim3& sim3)
  {
    out_str << sim3.rotation().coeffs() << std::endl;
    out_str << sim3.translation() << std::endl;
    out_str << sim3.scale() << std::endl;

    return out_str;
  }

} // end namespace


#endif
