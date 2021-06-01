//created by zzh
#include "OdomPreIntegrator.h"
#include "IMUInitialization.h"//for color cout
#ifdef USE_G2O_NEWEST
#else
#include "../Thirdparty/g2o/g2o/types/se3_ops.h"
#endif

using namespace std;

namespace VIEO_SLAM{

using namespace Eigen;

void EncPreIntegrator::PreIntegration(const double &timeStampi,const double &timeStampj,
				      const listeig(EncData)::const_iterator &iterBegin,const listeig(EncData)::const_iterator &iterEnd){
  if (iterBegin!=iterEnd&&timeStampi<timeStampj){//timeStampi may >=timeStampj for Map Reuse
    Vector2d eigdeltaPijM(0,0);//deltaPii=0
    double deltaThetaijMz=0;//deltaTheta~iiz=0
    mSigmaEij.setZero();//SigmaEii=0
    const double EPS = 1E-5;
    
//     listeig(EncData)::const_iterator it=iterEnd;
//     std::cout<<timeStampi<<" "<<timeStampj<<" "<<timeStampi-iterBegin->mtm<<" "<<timeStampj-(--it)->mtm<<std::endl;
//     assert(abs(timeStampi-iterBegin->mtm)<0.01&&abs(timeStampj-(it)->mtm)<0.01);
    
    Matrix2d eigSigmaeta(EncData::mSigma);
    Matrix6d eigSigmaetam(EncData::mSigmam);
    double rc(EncData::mrc);
    
    for (listeig(EncData)::const_iterator iterj=iterBegin;iterj!=iterEnd;){//start iterative method from i/iteri->tm to j/iter->tm
      listeig(EncData)::const_iterator iterjm1=iterj++;//iterj-1
      
      double deltat,tj,tj_1;//deltatj-1j
      if (iterjm1==iterBegin) tj_1=timeStampi; else tj_1=iterjm1->mtm;
      if (iterj==iterEnd){
	if (timeStampj-tj_1>0) tj=timeStampj;else break;
// 	tj=timeStampj;
      }else{
	tj=iterj->mtm;
	if (tj>timeStampj) tj=timeStampj;
      }
      deltat=tj-tj_1;
      if (deltat==0) continue;
//       assert(deltat>=0);
      if (deltat>1.5){ mdeltatij=0;cout<<redSTR"Check Odometry!"<<whiteSTR<<endl;return;}//this filter is for the problem of my dataset which only contains encoder data
      
      //selete/design measurement_j-1
      double vl,vr;//vlj-1,vrj-1
      vl=iterjm1->mv[0];vr=iterjm1->mv[1];//this way seems to be more precise than the following one (arithmatical average) for Corridor004
      /*
      if (iterj!=iterEnd){
	vl=(iterjm1->mv[0]+iterj->mv[0])/2,vr=(iterjm1->mv[1]+iterj->mv[1])/2;
// 	if (iterj->mtm>timeStampj){
// 	  vl=(iterjm1->mv[0]+(iterjm1->mv[0]*(iterj->mtm-timeStampj)+iterj->mv[0]*deltat)/(iterj->mtm-iterjm1->mtm))/2;
// 	  vr=(iterjm1->mv[1]+(iterjm1->mv[1]*(iterj->mtm-timeStampj)+iterj->mv[1]*deltat)/(iterj->mtm-iterjm1->mtm))/2;
// 	}
      }else{
	vl=iterjm1->mv[0];vr=iterjm1->mv[1];
      }*/
      double vf=(vl+vr)/2,w=(-vl+vr)/2/rc;//[vf;w]k=1/2*[1 1;-1/rc 1/rc]*[vl;vr]k, here k=j-1
      
      //calculate Sigmaij firstly to use deltaThetaijMz as deltaTheta~ij-1z, maybe we can use deltaP~ij-1 instead of vf/w here
      double thetaj_1j=w*deltat;//Theta~ej-1ej
      Matrix6d A(Matrix6d::Identity()),C;Matrix<double,6,2> B;
      Matrix3d Rij_1;//delta~REj-1Ej
      Rij_1<<cos(thetaj_1j),-sin(thetaj_1j),0,
	      sin(thetaj_1j),cos(thetaj_1j),0,
	      0,0,1;
      A.block<3,3>(0,0)=Rij_1.transpose();//delta~REj-1Ej.t()
      B.setZero();B(2,1)=deltat/2/rc;B(2,0)=-B(2,1);
      Matrix<double,3,2> Bj_11;
      double dt2=deltat*deltat;
      C.setZero();C(2,5)=deltat;
      Matrix<double,3,6> Cj_11;
      double sinthdivw,one_costh_divw;
      double Bx,By,C0,C1;
      if (abs(thetaj_1j)<EPS){
// 	A.block<3,3>(0,3)=Rij_1*g2o::skew(Vector3d(-vf*deltat,0,0));
// 	Bj_11.block<2,2>(0,0)<<deltat/2,deltat/2,0,0;
// 	Cj_11.block<3,3>(3,0)=deltat*Matrix3d::Identity();Cj_11.block<3,3>(3,3).setZero();
	sinthdivw=deltat;one_costh_divw=w*dt2/2;
	Bx=-vf*w*dt2*deltat/2;By=vf*dt2/2;
	C0=0;C1=-vf*dt2/2;
      }else{
	sinthdivw=sin(thetaj_1j)/w;one_costh_divw=(1-cos(thetaj_1j))/w;
	Bx=vf/w*(deltat*cos(thetaj_1j)-sinthdivw);By=vf/w*(deltat*sin(thetaj_1j)-one_costh_divw);
	C0=vf/w*(deltat-sinthdivw);C1=-vf/w*one_costh_divw;
      }
      A.block<3,3>(0,3)=Rij_1*g2o::skew(Vector3d(-vf*sinthdivw,-vf*one_costh_divw,0));
      Bj_11.block<3,2>(0,0)<<sinthdivw/2-Bx/2/rc,sinthdivw/2+Bx/2/rc,one_costh_divw/2-By/2/rc,one_costh_divw/2+By/2/rc,0,0;
      C(0,3)=sinthdivw;C(0,4)=one_costh_divw;C(1,3)=-one_costh_divw;C(1,4)=sinthdivw;
      Cj_11<<sinthdivw,-one_costh_divw,0,0,0,Bx,
	     one_costh_divw,sinthdivw,0,0,0,By,
	     0,0,deltat,C0,C1,0;
      B.block<3,2>(3,0)=Rij_1*Bj_11;
      C.block<3,6>(3,0)=Rij_1*Cj_11;
      if (EncData::mdt_cov_noise_fixed)//eta->etad
          mSigmaEij=A*mSigmaEij*A.transpose()+B*eigSigmaeta*B.transpose()+C*eigSigmaetam*deltat*C.transpose();
      else if (!EncData::mFreqRef || deltat < 1.5 / EncData::mFreqRef)
          mSigmaEij=A*mSigmaEij*A.transpose()+B*(eigSigmaeta/deltat)*B.transpose()+C*(eigSigmaetam*deltat)*C.transpose();
      else
          mSigmaEij=A*mSigmaEij*A.transpose()+B*(eigSigmaeta*EncData::mFreqRef)*B.transpose()+C*(eigSigmaetam*deltat)*C.transpose();
      
      //update deltaPijM before update deltaThetaijM to use deltaThetaijMz as deltaTheta~ij-1z
      double thetaij=deltaThetaijMz+thetaj_1j;//Theta~eiej
      if (abs(thetaj_1j)<EPS){//or thetaj_1j==0
	double arrdTmp[4]={cos(deltaThetaijMz),sin(deltaThetaijMz),-sin(deltaThetaijMz),cos(deltaThetaijMz)};//row-major:{cos(deltaTheij-1Mz),-sin(deltaTheij-1Mz),0,sin(deltaTheij-1Mz),cos(deltaThetaij-1Mz),0,0,0,1}; but Eigen defaultly uses col-major!
	eigdeltaPijM+=Matrix2d(arrdTmp)*Vector2d(vf*deltat,0);//deltaPijM+Reiej-1*vej-1ej-1*deltat
      }else{
	eigdeltaPijM+=vf/w*Vector2d(sin(thetaij)-sin(deltaThetaijMz),cos(deltaThetaijMz)-cos(thetaij));
      }
      //update deltaThetaijM
      deltaThetaijMz=thetaij;//deltaThetaij-1M + weiej-1 * deltatj-1j, notice we may use o in the code instead of e in the paper
    }
    
    mdelxEij[0]=mdelxEij[1]=mdelxEij[5]=0;mdelxEij[2]=deltaThetaijMz;mdelxEij.segment<2>(3)=eigdeltaPijM;
    mdeltatij=timeStampj-timeStampi;
  }
}
void IMUPreIntegratorDerived::PreIntegration(const double &timeStampi,const double &timeStampj){
  if (!this->mlOdom.empty()){
    IMUDataDerived& datai=this->mlOdom.front(),dataj=this->mlOdom.back();
    mdelxRji=dataj.quat.conjugate()*datai.quat;//R~j.t()*R~i, suppose quat is normalized~
    mSigmaPhiij=IMUDataDerived::mSigmaI;
    Matrix3d Ai(mdelxRji*datai.getJacoright());
    //get SigmaIij=Ai*Sigmaetawi*Ai.t()+Jrj*Sigmaetawi*Jrj.t(),Ai=(R~j.t()*R~i)*Jr(phiwIb(ti))=(R~j.t()*R~i)*Jr(R~wibi)
    Matrix3d Jrj=dataj.getJacoright();
    mSigmaPhiij=Ai*mSigmaPhiij*Ai.transpose()+Jrj*mSigmaPhiij*Jrj.transpose();
    this->mdeltatij=timeStampj-timeStampi;
  }
}

}