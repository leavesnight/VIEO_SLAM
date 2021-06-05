#include "IMUInitialization.h"
#include "Optimizer.h"
//#include "sophus/se3.hpp"
#include "eigen_utils.h"

namespace VIEO_SLAM {

using namespace std;
using namespace Eigen;

IMUKeyFrameInit::IMUKeyFrameInit(KeyFrame& kf):mTimeStamp(kf.mTimeStamp),mTwc(kf.GetPoseInverse()),mTcw(kf.GetPose()), mpPrevKeyFrame(NULL),//GetPose() already return .clone()
mOdomPreIntIMU(kf.GetIMUPreInt()){//this func. for IMU Initialization cache of KFs, so need deep copy
  mbg_=mba_=Vector3d::Zero();//as stated in IV-A in VIORBSLAM paper 
  const listeig(IMUData) limu=kf.GetListIMUData();
  mOdomPreIntIMU.SetPreIntegrationList(limu.begin(),--limu.end());
}

cv::Mat IMUInitialization::GetGravityVec(void){
  //unique_lock<mutex> lock(mMutexInitIMU);//now we don't need mutex for it 1stly calculated only in this thread and then it will be a const!
  return mGravityVec;//.clone();//avoid simultaneous operation
}
void IMUInitialization::SetGravityVec(const cv::Mat &mat){
  //unique_lock<mutex> lock(mMutexInitIMU);//now we don't need mutex for it 1stly calculated only in this thread and then it will be a const!
  mGravityVec=mat.clone();//avoid simultaneous operation
}

void IMUInitialization::Run(){
  unsigned long initedid;
  cout<<"start VINSInitThread"<<endl;
  mbFinish=false;
  while(1){
    if(GetSensorIMU()){//at least 4 consecutive KFs, see IV-B/C VIORBSLAM paper
      KeyFrame* pCurKF=GetCurrentKeyFrame();
      if (mdStartTime==-1){ initedid=0;mdStartTime=-2;}
      if(mdStartTime<0||mdStartTime>=0&&pCurKF->mTimeStamp-mdStartTime>=mdInitTime)
        if(!GetVINSInited() && pCurKF!=NULL && pCurKF->mnId > initedid){
          initedid = pCurKF->mnId;
          if (TryInitVIO()) break;//if succeed in IMU Initialization, this thread will finish, when u want the users' pushing reset button be effective, delete break!
        }
    }
    
    ResetIfRequested();
    if(GetFinishRequest()) break;
//     usleep(3000);
    usleep(mnSleepTime);//3,1,0.5
  }
  SetFinish(true);
  cout<<"VINSInitThread is Over."<<endl;
}

    int IMUInitialization::deleteKFs_ret(vector<vector<IMUKeyFrameInit *>*> &vKFsInit) {
        for (int h = 0; h < vKFsInit.size(); ++h) {
            for (int i = 0; i < vKFsInit[h]->size(); ++i) {
                if ((*vKFsInit[h])[i]) delete (*vKFsInit[h])[i];
            }
            delete vKFsInit[h];
        }
        return 0;
    }

    int IMUInitialization::reduceKFs(const vector<char> &reduced_hids,
                                     vector<vector<IMUKeyFrameInit *>*> &vKFsInit, vector<vector<IMUKeyFrameInit *>*> &vKFsInit2,
                                     vector<int> &Ns, int &num_handlers, vector<char> &id_cams, vector<char> *id_cams_ref){
        num_handlers = reduced_hids.size();
        Ns.clear();
        char ref_pass = 0;
        id_cams.resize(num_handlers);
        if (id_cams_ref && (*id_cams_ref).size() == num_handlers)
            ref_pass = 1;
        for (int i = 0; i < num_handlers; ++i) {
            vKFsInit2.push_back(vKFsInit[reduced_hids[i]]);
            vKFsInit[reduced_hids[i]] = NULL;
            if (ref_pass)
                id_cams[i] = (*id_cams_ref)[i];
            else
                id_cams[i] = reduced_hids[i];
            Ns.push_back(vKFsInit2.back()->size());
        }
        //delete redundant KFsInit
        for (int h = 0; h < vKFsInit.size(); ++h) {
            if (NULL != vKFsInit[h]) {
                for (int i = 0; i < vKFsInit[h]->size(); ++i) {
                    if ((*vKFsInit[h])[i]) delete (*vKFsInit[h])[i];
                }
                vKFsInit[h] = NULL;
            }
        }
        vKFsInit.clear();

        return 0;
    }

//    bool IMUInitialization::TryInitVIO_zzh(void) {//now it's the version cannot allow the KFs has no inter IMUData in initial 15s!!!
//        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//
//        int num_var_opt = mbMonocular ? 4 : 3;
//        int num_var_opt2 = num_var_opt + 3 - 1;
//        //at least N>=4
//        if (!mpMap || mpMap->KeyFramesInMap() < 4) { return false; }//21,11, 4
//
//        //Recording data in txt files for further analysis
//        static bool fopened = false;
//        static ofstream fgw, fscale, fbiasa, fcondnum, fbiasg;
//        if (mTmpfilepath.length() > 0 && !fopened) {
//            if (kVerbDeb < verbose)
//                cout<<"open "<<mTmpfilepath<<"...";
//            // Need to modify this to correct path
//            fbiasg.open(mTmpfilepath + "biasg2.txt");//optimized initial bg for these N KFs,3*1
//            fgw.open(mTmpfilepath + "gw2.txt");//gwafter then gw before,6*1
//            fscale.open(mTmpfilepath +
//                        "scale2.txt");//scale_fine then scale_rough, 2*1 only for Monocular camera//if (mbMonocular)
//            fbiasa.open(mTmpfilepath + "biasa2.txt");//optimized initial ba for these N KFs,3*1
//            //for optimized x is 6*1 vector, see (19) in VOIRBSLAM paper, here just show these 6*1 raw data
//            fcondnum.open(mTmpfilepath + "condnum2.txt");
//            if (fbiasg.is_open() && fgw.is_open() && (fscale.is_open()) && //!mbMonocular||
//                fbiasa.is_open() && fcondnum.is_open())
//                fopened = true;
//            else {
//                cerr << "file open error in TryInitVIO" << endl;
//                fopened = false;
//            }
//            fbiasg << std::fixed << std::setprecision(9);
//            fgw << std::fixed << std::setprecision(9);
//            fscale << std::fixed << std::setprecision(9);
//            fbiasa << std::fixed << std::setprecision(9);
//            fcondnum << std::fixed << std::setprecision(9);
//            if (kVerbDeb < verbose)
//                cout<<"...ok..."<<endl;
//        }
//
//        // Cache KFs / wait for KeyFrameCulling() over
//        while (GetCopyInitKFs()){
//            if (kVerbDeb < verbose)
//                cout<<".";
//            usleep(1000);
//        }
//        if (kVerbDeb < verbose)
//            cout<<endl<<"copy init KFs...";
//        SetCopyInitKFs(true);//stop KeyFrameCulling() when this copying KFs
//
//        //see VIORBSLAM paper IV, here N=all KFs in map, not the meaning of local KFs' number
//        // Use all KeyFrames in map to compute
//        vector<KeyFrame *> vScaleGravityKF = mpMap->GetAllKeyFrames();
//        assert(vScaleGravityKF.size() && (*vScaleGravityKF.begin())->mnId == 0);
//        if (verbose && (*vScaleGravityKF.begin())->mnId != 0)
//            cerr<<"vScaleGrayvityKF begin id !=0!"<<endl;
//        int NvSGKF = vScaleGravityKF.size();
//        KeyFrame *pNewestKF = vScaleGravityKF[NvSGKF - 1];
//        // Store initialization-required KeyFrame data
//        vector<vector<IMUKeyFrameInit *>*> vKFsInit(1, new vector<IMUKeyFrameInit *>()), vKFsInit2;
//        vector<IMUKeyFrameInit *> lastKFInit(1, NULL);
//
//        for (int i = 0; i < NvSGKF; ++i) {
//            KeyFrame *pKF = vScaleGravityKF[i];
//            char id_cam = 0;//pKF->id_cam_;
////     if (pKF->mTimeStamp<pNewestKF->mTimeStamp-15) continue;//15s as the VIORBSLAM paper
//            IMUKeyFrameInit *pkfi = new IMUKeyFrameInit(*pKF);
//            if (id_cam >= vKFsInit.size()) {
//                vKFsInit.resize(id_cam + 1);
//                vKFsInit.back() = new vector<IMUKeyFrameInit *>();
//                lastKFInit.resize( id_cam + 1);
//                lastKFInit.back() = NULL;
//            }
//            if (lastKFInit[id_cam])
//                pkfi->mpPrevKeyFrame = lastKFInit[id_cam];
//            lastKFInit[id_cam] = pkfi;
//
//            vKFsInit[id_cam]->push_back(pkfi);
//        }
//
//        SetCopyInitKFs(false);
//        if (kVerbDeb < verbose)
//            cout<<"...end"<<endl;
//
//        // Step 1. / see VIORBSLAM paper IV-A
//        int num_handlers = vKFsInit.size();
//        vector<int> Ns(num_handlers);
//        if (verbose)
//            cout<<"Step1: left num_handlers="<<num_handlers<<endl;
//        vector<Vector3d> bgs_est(num_handlers, Vector3d::Zero());
//        vector<Vector3d> bas_star(num_handlers);
//        vector<char> reduced_hids, flag_remain(num_handlers);
//        //fast return judging
//        char tot_imu_init_enable = 0;
//        int min_num_unknowns = 6;
//        for (int i = 0; i < num_handlers; ++i) {
//            Ns[i] = vKFsInit[i]->size();
//            if (verbose) {
//                cout<<i<<":"<<Ns[i]<<" keyframes"<<endl;
//            }
//            if (4 <= Ns[i]) {
//                ++tot_imu_init_enable;
//                break;
//            }
//        }
//        if (!tot_imu_init_enable) {//at least 1 imu may be initialized
//            deleteKFs_ret(vKFsInit);
//            return false;
//        }
//        std::chrono::steady_clock::time_point tm_start,tm_end;
//        // Try to compute initial gyro bias, using optimization with Gauss-Newton
//        // nothing changed, just return the optimized result bg*
//        for (int i = 0; i < num_handlers; ++i) {
//            if (verbose) {
//                tm_start = std::chrono::steady_clock::now();
//            }
//            if (0 < Optimizer::OptimizeInitialGyroBias<IMUKeyFrameInit>(*vKFsInit[i], bgs_est[i]))
//                reduced_hids.push_back(i);
//            if (verbose) {
//                static double max_duration = 0, tot_duration = 0;
//                static size_t mean_times = 0;
//                tm_end = std::chrono::steady_clock::now();
//                double duration = (double)std::chrono::duration_cast<std::chrono::duration<double> >(tm_end -
//                                                                                                     tm_start).count();
//                if (max_duration < duration)
//                    max_duration = duration;
//                tot_duration += duration;
//                ++mean_times;
//                cout << "OptimizeInitialGyroBias cost : " << duration << "s; max = " << max_duration << "; mean = "<<
//                     tot_duration / mean_times << endl;
//            }
//        }
//        //reduce vKFsInit to vKFsInit2
//        vector<char> id_cams, id_cams2;
////        cerr<<"check reduce bef:"<<vKFsInit.size()<<" "<<vKFsInit[0]->size()<<" "<<(*vKFsInit[0])[1]->mOdomPreIntIMU.getlOdom().size()<<endl;
////        for (int i = 0; i < (*vKFsInit[0]).size(); ++i)
////            cerr<<" "<<SO3exd((*vKFsInit[0])[i]->mOdomPreIntIMU.mRij).log().transpose()<< ", t=" << (*vKFsInit[0])[i]->mOdomPreIntIMU.mpij.transpose()<<"; ";
////        cerr<<endl;
//        reduceKFs(reduced_hids, vKFsInit, vKFsInit2, Ns, num_handlers, id_cams, NULL);
////        cerr<<"check reduce bef:"<<vKFsInit2.size()<<" "<<vKFsInit2[0]->size()<<" "<<(*vKFsInit2[0])[1]->mOdomPreIntIMU.getlOdom().size()<<endl;
//        if (verbose) {
//            for (int i = 0; i < num_handlers; ++i)
//                cout << "bgs_est[" << (int) id_cams[i] << "]: " << bgs_est[i].transpose() << endl;
//            cout << "Step2: left num_handlers=" << num_handlers << endl;
//        }
//
//        // Update biasg and pre-integration in LocalWindow(here all KFs).
//        int num_eq = 0, num_eq2_ref = 0;
//        for (int h = 0; h < num_handlers; ++h) {
//            for (int i = 0; i < Ns[h]; ++i)
//                (*vKFsInit2[h])[i]->mbg_ = bgs_est[h];
//            num_eq += Ns[h] - 2;
//        }
//        for (int h = 0; h < num_handlers; ++h)
//            for (int i = 1; i < Ns[h]; ++i)
//                //so vKFInit[i].mOdomPreIntIMU is based on bg_bar=bgest,ba_bar=0; dbg=0 but dba/ba waits to be optimized
//                (*vKFsInit2[h])[i]->ComputePreInt();
////        for (int i = 0; i < (*vKFsInit2[0]).size(); ++i)
////            cerr<<" "<<SO3exd((*vKFsInit2[0])[i]->mOdomPreIntIMU.mRij).log().transpose()<< ", t=" << (*vKFsInit2[0])[i]->mOdomPreIntIMU.mpij.transpose()<<"; ";
////        cerr<<endl;
//
//        // Step 2. / See VIORBSLAM paper IV-B
//        // Approx Scale and Gravity vector in 'world' frame (first/0th KF's camera frame)
//        // Solve A*x=B for x=[s,gw] 4x1 vector, using SVD method
//        typedef float Tcalc_sgba;
//        typedef Eigen::Matrix<Tcalc_sgba, Dynamic, Dynamic> MatrixXXcalc;
//        typedef Eigen::Matrix<Tcalc_sgba, Dynamic, 1> VectorXcalc;
//        typedef Sophus::SE3<Tcalc_sgba> SE3calc;
//        typedef Sophus::SO3<Tcalc_sgba> SO3calc;
//        double s_star = 1;
//        MatrixXXcalc A(3 * num_eq, num_var_opt);//if num_var_opt=4 unknowns then N must >=4 else >=3
//        A.setZero();
//        VectorXcalc B(3 * num_eq);
//        B.setZero();
//        typedef Eigen::Matrix<Tcalc_sgba, 3, 3> Matrix3calc;
//        typedef Eigen::Matrix<Tcalc_sgba, 3, 1> Vector3calc;
//        Matrix3calc I3=Matrix3calc::Identity();
//        aligned_vector<aligned_vector<SO3calc>> Rcbs(1);
//        aligned_vector<aligned_vector<Vector3calc>> pcbs(1);
//        vector<vector<size_t>> id_Tcbs(1);
//        tot_imu_init_enable = 0;
//        num_eq = 0;
//        reduced_hids.clear();
//        for (int h = 0, last_h = h; h < num_handlers; ++h) {
//            int numEquations = 0;
//            for (int i = 0, last_i = i; i < Ns[h] - 2; ++i) {
//                IMUKeyFrameInit *pKF2 = (*vKFsInit2[h])[i + 1], *pKF3 = (*vKFsInit2[h])[i + 2];
//                double dt12 = pKF2->mOdomPreIntIMU.mdeltatij;//deltat12
//                double dt23 = pKF3->mOdomPreIntIMU.mdeltatij;
//                if (dt12 == 0 || dt23 == 0) {
//                    cout << redSTR << "Tm=" << fixed << setprecision(9) << pKF2->mTimeStamp << " lack IMU data!"
//                         << dt12 << "," << dt23 << whiteSTR << defaultfloat << endl;
//                    continue;
//                }
//                ++numEquations;
//                // Extrinsics
//                SO3calc Rcb[3];
//                Vector3calc pcb[3];
//                if (h > last_h) {
//                    Rcbs.resize(Rcbs.size() + 1);
//                    pcbs.resize(pcbs.size() + 1);
//                    id_Tcbs.resize(id_Tcbs.size() + 1);
//                    last_h = h;
//                }
//                for (int k = 0; k < 3; ++k) {
//                    SE3calc Tbc = Converter::toSE3<Tcalc_sgba>(Frame::mTbc);
//                    SO3calc Rbc = Tbc.so3();
//                    Vector3calc pbc = Tbc.translation();
//                    Rcb[k] = Rbc.inverse();
//                    pcb[k] = -(Rcb[k] * pbc);
//                    if (i + k + 1 > last_i) {
//                        Rcbs.back().push_back(Rcb[k]);
//                        pcbs.back().push_back(pcb[k]);
//                        last_i = i + k + 1;
//                    }
//                }
//                id_Tcbs.back().push_back(Rcbs.back().size() - 3);
//                // Pre-integrated measurements
//                Vector3calc dp12 = pKF2->mOdomPreIntIMU.mpij.cast<Tcalc_sgba>();//deltap12
//                Vector3calc dv12 = pKF2->mOdomPreIntIMU.mvij.cast<Tcalc_sgba>();
//                Vector3calc dp23 = pKF3->mOdomPreIntIMU.mpij.cast<Tcalc_sgba>();
//                // Pose of camera in world frame
//                //Twci for pwci&Rwci, not necessary for clone()
//                SE3calc Twc1 = Converter::toSE3<Tcalc_sgba>((*vKFsInit2[h])[i]->mTwc);
//                SE3calc Twc2 = Converter::toSE3<Tcalc_sgba>(pKF2->mTwc);
//                SE3calc Twc3 = Converter::toSE3<Tcalc_sgba>(pKF3->mTwc);
//                Vector3calc pc1 = Twc1.translation();//pwci
//                Vector3calc pc2 = Twc2.translation();
//                Vector3calc pc3 = Twc3.translation();
//                SO3calc Rc1 = Twc1.so3();//Rwci
//                SO3calc Rc2 = Twc2.so3();
//                SO3calc Rc3 = Twc3.so3();
//
//                // fill A/B matrix: lambda*s + beta*g = gamma(3*1), Ai(3*4)=[lambda beta], (13) in the paper
//                Matrix3calc beta = (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) / 2 * I3;
//                Vector3calc gamma =
//                        (Rc1 * pcb[0] - Rc2 * pcb[1]) * dt23 + (Rc3 * pcb[2] - Rc2 * pcb[1]) * dt12 - Rc2 * (Rcb[1] *
//                                                                                                             dp23) * dt12 - Rc1 * (Rcb[0] * dv12) * dt12 * dt23 + Rc1 * (Rcb[0] * dp12) * dt23;
//
//                Vector3calc lambda = (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;
//                if (4 == num_var_opt) {
//                    A.block<3, 1>(3 * num_eq, 0) = lambda;
//                } else {
//                    gamma -= lambda * s_star;
//                }
//                A.block<3, 3>(3 * num_eq, num_var_opt - 3) = beta;//Ai
//                // the paper missed a minus before γ(i)
//                B.segment<3>(3 * num_eq) = gamma;//gamma/B(i), but called gamma(i) in the paper
//                ++num_eq;
//            }
//            if (3 * numEquations >= num_var_opt) {
//                ++tot_imu_init_enable;
//                reduced_hids.push_back(h);
//                id_cams2.push_back(id_cams[h]);
//                num_eq2_ref += numEquations;
//            }
//        }
//        if (!tot_imu_init_enable && 3 * num_eq < num_var_opt) {//for more robust judgement instead of judging KeyFramesInMap()
//            deleteKFs_ret(vKFsInit2);
//            if (verbose) {
//                cout<< "tot_imu_init_enable failed"<<endl;
//            }
//            return false;
//        }
//        //reduce A,B rows to speed up
//        A.resize(3 * num_eq, num_var_opt);
//        B.resize(3 * num_eq);
//        // Use svd to compute A*x=B, x=[s,gw] 4x1 vector
//        // A=u*S*vt=u*w*vt, u*w*vt*x=B => x=vt'*winv*u'*B, or we call the pseudo inverse of A/A.inv()=(A.t()*A).inv()*
//        // A.t(), in SVD we have A.inv()=v*winv*u.t() where winv is the w with all nonzero term is the reciprocal of the
//        // corresponding singular value
////        A.ldlt().solve(B);
//        // Note w is 4x1 vector by SVDecomp()/SVD::compute() not the 4*4(not FULL_UV)/m*n(FULL_UV) singular matrix we
//        //  stated last line
//        Eigen::JacobiSVD<MatrixXXcalc> svdA(A, ComputeThinU | ComputeThinV);
//        VectorXcalc w = svdA.singularValues();
//        const int thresh_cond = 1e6;
//        double cond_num = w[0] / w[w.size() - 1];
//        if (cond_num > thresh_cond) {
//            deleteKFs_ret(vKFsInit2);
//            if (verbose) {
//                cout<<"cond_num="<<cond_num<<" > thresh_cond("<<thresh_cond<<")"<<endl;
//            }
//            return false;
//        }
//        MatrixXXcalc u = svdA.matrixU();
//        MatrixXXcalc v = svdA.matrixV();
//        MatrixXXcalc winv = MatrixXXcalc::Identity(w.size(), w.size());
//        for (int i = 0; i < num_var_opt; ++i) {
//            //too small in sufficient w meaning the linear dependent equations causing the solution is not unique(or
//            // A.inv() not exist)
////            if (fabs(w(i)) < 1e-10) {
////                w(i) += 1e-10;//or [] here for vector only
////                cerr << "w(i) < 1e-10, w=" << endl << w << endl;
////            }
//            winv(i, i) = 1. / w(i);
//        }
//        VectorXcalc x = v * (winv * (u.transpose() * B));
//        Vector3calc gw_star;
//        if (4 == num_var_opt) {
//            s_star = x(0); // scale should be positive
//            gw_star = x.segment(1,3); // gravity should be about ~9.8
//        } else {
//            gw_star = x;
//        }
//        cout << "gw_star: " << gw_star.transpose() << ", |gw_star|=" << gw_star.norm() << endl;
//
//        //reduce vKFsInit2 to vKFsInit
//        reduceKFs(reduced_hids, vKFsInit2, vKFsInit, Ns, num_handlers, id_cams2, &id_cams);
//        // Step 3. / See VIORBSLAM paper IV-C
//        SO3calc RwI;//for Recording
//        // Use gravity magnitude 9.810 as constraint; gIn/^gI=[0;0;1], the normalized gravity vector in an inertial
//        // frame, we can also choose gIn=[0;0;-1] as the VIORBSLAM paper
//        Vector3calc gIn = Vector3calc::Zero();
//        gIn(2) = 1;
//        Vector3calc GI = gIn * IMUData::mdRefG;//gI or GI=^gI*G
//        double s_ = s_star;
//        SO3calc RwI_;
////   for (int k=0;k<2;++k){//we prefer 1 iteration
////     if (k==1){
////       gwstar=Rwi_*GI;
////       for(int i=0;i<N;++i) vKFInit[i]->mba_=bastareig;
////       for(int i=1;i<N;++i) vKFInit[i]->ComputePreInt();
////     }
//
//        Vector3calc gwn = gw_star / gw_star.norm();//^gw=gw*/||gw*|| / Normalized approx. gravity vecotr in world frame
//        Vector3calc gInxgwn = gIn.cross(gwn);
//        double normgInxgwn = gInxgwn.norm();
//        Vector3calc vhat = gInxgwn / normgInxgwn;//RwI=Exp(theta*^v), or we can call it vn=(gI x gw)/||gI x gw||
//        //notice theta*^v belongs to [-Pi,Pi]*|^v| though theta belongs to [0,Pi]
//        double theta = std::atan2(normgInxgwn, gIn.dot(gwn));
//        RwI = Sophus::SO3exd::exp(vhat.cast<double>() * theta).cast<Tcalc_sgba>();//RwI
//
//        // Solve C*x=D for x=[s,dthetaxy,ba] (1+2+3)x1 vector
//        MatrixXXcalc C(3 * num_eq, num_var_opt2 + (num_handlers - 1) * 3);//5/6 unknowns so N must >=4
//        C.setZero();
//        VectorXcalc D(3 * num_eq);
//        D.setZero();
//        assert(Rcbs.size() == num_handlers && pcbs.size() == num_handlers );
//        int num_eq2 = 0;
//        for (int h = 0; h < num_handlers; ++h) {
//            int num_eq_h = 0;
//            size_t reduced_h = reduced_hids[h];
//            aligned_vector<SO3calc> &Rcbs_h = Rcbs[reduced_h];
//            aligned_vector<Vector3calc> &pcbs_h = pcbs[reduced_h];
//            for (int i = 0; i < Ns[h] - 2; ++i) {
//                IMUKeyFrameInit *pKF2 = (*vKFsInit[h])[i + 1], *pKF3 = (*vKFsInit[h])[i + 2];
//                const IMUPreintegrator &imupreint12 = pKF2->mOdomPreIntIMU, &imupreint23 = pKF3->mOdomPreIntIMU;
//                //d means delta
//                double dt12 = imupreint12.mdeltatij;
//                double dt23 = imupreint23.mdeltatij;
//                assert(dt12 && dt23);
//                if (dt12 == 0 || dt23 == 0)
//                    continue;
//                size_t reduced_i = id_Tcbs[reduced_h][num_eq_h];
//                const Matrix3calc &Rcb1 = Rcbs_h[reduced_i].matrix(), &Rcb2 = Rcbs_h[reduced_i + 1].matrix(), &Rcb3 =
//                        Rcbs_h[reduced_i + 2].matrix();
//                const Vector3calc &pcb1 = pcbs_h[reduced_i], &pcb2 = pcbs_h[reduced_i + 1], &pcb3 =
//                        pcbs_h[reduced_i + 2];
//                Vector3calc dp12 = imupreint12.mpij.cast<Tcalc_sgba>();
//                Vector3calc dp23 = imupreint23.mpij.cast<Tcalc_sgba>();
//                Vector3calc dv12 = imupreint12.mvij.cast<Tcalc_sgba>();
//                Matrix3calc Jav12 = imupreint12.mJavij.cast<Tcalc_sgba>();
//                Matrix3calc Jap12 = imupreint12.mJapij.cast<Tcalc_sgba>();
//                Matrix3calc Jap23 = imupreint23.mJapij.cast<Tcalc_sgba>();
//                SE3calc Twc1 = Converter::toSE3<Tcalc_sgba>((*vKFsInit[h])[i]->mTwc);
//                SE3calc Twc2 = Converter::toSE3<Tcalc_sgba>(pKF2->mTwc);
//                SE3calc Twc3 = Converter::toSE3<Tcalc_sgba>(pKF3->mTwc);
//                Vector3calc pc1 = Twc1.translation();//pwci
//                Vector3calc pc2 = Twc2.translation();
//                Vector3calc pc3 = Twc3.translation();
//                Matrix3calc Rc1 = Twc1.so3().matrix();//Rwci
//                Matrix3calc Rc2 = Twc2.so3().matrix();
//                Matrix3calc Rc3 = Twc3.so3().matrix();
//                // Stack to C/D matrix; lambda*s + phi(:,0:1)*dthetaxy + zeta*ba = psi, Ci(3*6),Di/psi(3*1)
//                Matrix3calc phi = -(dt12 * dt12 * dt23 + dt12 * dt23 * dt23) / 2 * RwI.matrix() *
//                                  Sophus::SO3ex<Tcalc_sgba>::hat(GI);//3*3 note: this has a '-', different to paper
//                Matrix3calc zeta = Rc2 * Rcb2 * Jap23 * dt12 + Rc1 * Rcb1 * Jav12 * (dt12 * dt23) -
//                                   Rc1 * Rcb1 * Jap12 * dt23;//3*3 notice here is Jav12, paper writes a wrong Jav23
//                //note:  - paper & deltatij^2 in paper means dt12^2*dt23+dt23^2*dt12
//                //notice here use Rwi*GI instead of gwstar for it's designed for iterative usage
//                Vector3calc psi = (Rc1 * pcb1 - Rc2 * pcb2) * dt23 + (Rc3 * pcb3 - Rc2 * pcb2) * dt12 - Rc2 * (Rcb2 *
//                                                                                                               dp23) * dt12 - Rc1 * (Rcb1 * dv12) * (dt12 * dt23) + Rc1 * (Rcb1 * dp12) * dt23 - (dt12 * dt12 *
//                                                                                                                                                                                                  dt23 + dt12 * dt23 * dt23) / 2 * (RwI * GI);
//                Vector3calc lambda = (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;//3*1
//                if (6 == num_var_opt2) {
//                    C.block<3, 1>(3 * num_eq2, 0) = lambda;
//                } else {
//                    psi -= lambda * s_;
//                }
//                //phi(:,0:1)(3*2) / only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
//                C.block<3, 2>(3 * num_eq2, num_var_opt2 - 5) = phi.block<3, 2>(0, 0);
//                C.block<3, 3>(3 * num_eq2, num_var_opt2 - 3 + h * 3) = zeta;
//                D.segment<3>(3 * num_eq2) = psi;
//                ++num_eq2;
//                ++num_eq_h;
//            }
//            if (kVerbDeb < verbose && Rcbs[h].size() != num_eq_h + 2)
//                cerr<<"Rcb["<<h<<"].size="<<Rcbs[h].size()<<";num_eq_h="<<num_eq_h<<endl;
//        }
//        if (num_eq2_ref != num_eq2) {
//            cerr << "num_eq2 in Step3 != num_eq2_ref in Step2!" << endl;
//        }
//        assert(num_eq2 == num_eq2_ref);
//        // Use svd to compute C*x=D, x=[s,dthetaxy,ba] 6x1 vector
//        // Note w2 is 6x1 vector by SVDecomp(), for Recording
//        Eigen::JacobiSVD<MatrixXXcalc> svdC(C, ComputeThinU | ComputeThinV);
//        VectorXcalc w2 = svdC.singularValues();
//        double cond_num2 = w2(0) / w2(w2.size() - 1);
//        if (cond_num2 > thresh_cond) {
//            deleteKFs_ret(vKFsInit);
//            if (verbose) {
//                cout<<"cond_num2="<<cond_num<<" > thresh_cond("<<thresh_cond<<")"<<endl;
//            }
//            return false;
//        }
//        MatrixXXcalc u2 = svdC.matrixU();
//        MatrixXXcalc v2 = svdC.matrixV();
//        MatrixXXcalc w2inv = MatrixXXcalc::Identity(w2.size(), w2.size());
//        for (int i = 0; i < w2.size(); ++i) {
////            if (fabs(w2(i)) < 1e-10) {
////                w2(i) += 1e-10;
////                cerr << "w2("<<i<<") < 1e-10, w=" << endl << w2 << endl;
////            }
//            w2inv(i, i) = 1. / w2(i);
//        }
//        VectorXcalc y = v2 * (w2inv * (u2.transpose() * D));// Then y/x = vt'*winv*u'*D
//        if (6 == num_var_opt2)
//            s_ = y(0);//s*_C, C means IV-C in the paper
//        //small deltatheta/dtheta=[dthetaxy.t() 0].t()
//        Vector3d dthetaeig(y(num_var_opt2 - 5), y(num_var_opt2 - 4), 0);
//        RwI_ = (RwI.cast<double>() * Sophus::SO3exd::exp(dthetaeig)).cast<Tcalc_sgba>();//RwI*_C=RwI*_B*Exp(dtheta)
////     if (k==0)
//        for (int h = 0; h < num_handlers; ++h)
//            bas_star[id_cams2[h]] = y.segment(num_var_opt2 - 3 + h * 3, 3).cast<double>();//here bai_bar=0, so dba=ba
////     else bastareig+=y.segment(3, 3).cast<double>();
////   }
//
//        // Record data for analysis
//        //direction of gwbefore is the same as gwstar, but value is different!
//        Vector3calc gwbefore = RwI * GI, gwafter = RwI_ * GI;
//        cout << "gwbefore=" << gwbefore.transpose() << ", gwafter=" << gwafter.transpose() << endl;
//
//        //Debug the frequency & sstar2&sstar
//        cout << "Time: " << fixed << setprecision(9) << pNewestKF->mTimeStamp - mdStartTime << ", s_star: " <<
//             s_star << ", s: " << s_ << defaultfloat << endl;
//        //<<" bgest: "<<bgest.transpose()<<", gw*(gwafter)="<<gwafter.t()<<", |gw*|="<<cv::norm(gwafter)<<",
//        // norm(gwbefore,gwstar)"<<cv::norm(gwbefore.t())<<" "<<cv::norm(gwstar.t())<<endl;
//        if (mTmpfilepath.length() > 0) {//Debug the Rwistar2
//            ofstream fRwi(mTmpfilepath + "Rwi.txt");
//            fRwi << RwI_.unit_quaternion().vec().transpose() << endl;
//            fRwi.close();
//        }
//        fgw << pNewestKF->mTimeStamp << " " << gwafter.transpose() << " " << gwbefore.transpose() << " " << endl;
//        fscale << pNewestKF->mTimeStamp << " " << s_ << " " << s_star << " " << endl;//if (mbMonocular)
//        for (int h = 0; h < num_handlers; ++h) {
//            fbiasg << (int)id_cams2[h] << " " << pNewestKF->mTimeStamp << " " << bgs_est[h].transpose() << " " <<
//                   endl;
//            fbiasa << (int)id_cams2[h] << " " << pNewestKF->mTimeStamp << " " << bas_star[h].transpose() << " " <<
//                   endl;
//        }
//        fcondnum << pNewestKF->mTimeStamp << " " << w2.transpose() << " " << endl;
//
//        // ********************************
//        // Todo: Add some logic or strategy to confirm init status, VIORBSLAM paper just uses 15 seconds to confirm
//        bool bVIOInited = false;
//        if (mdStartTime < 0) mdStartTime = pNewestKF->mTimeStamp;
//        cout << yellowSTR"condnum=" << cond_num2 << ";max=" << w2(0) << ";min=" <<
//             w2(w2.size() - 1) << whiteSTR << endl;
//        int min_cond_num = 500;//try 700
//        if (cond_num2 < min_cond_num and pNewestKF->mTimeStamp - mdStartTime >= mdFinalTime) {//15s in the paper V-A
//            bVIOInited = true;
//        }
//
//        //if VIO is initialized with appropriate bg*,s*,gw*,ba*, update the map(MPs' P,KFs' PRVB) like GBA/CorrrectLoop()
//        if (bVIOInited) {
//            // Set NavState , scale and bias for all KeyFrames
//            double scale = s_;
//            // gravity vector in world frame
//            Vector3f gw;
//            Vector3d gwd;
//            {
//                unique_lock<mutex> lock(mMutexInitIMU);
//                gw = (RwI_ * GI).cast<float>();
//                gwd = gw.cast<double>();
//                mGravityVec = Converter::toCvMat(gwd);
//            }
//
//            {// Update the Map needs mutex lock: Stop local mapping, like RunGlobalBundleAdjustment() in LoopClosing.cc
//                mpLocalMapper->RequestStop();//same as CorrectLoop(), suspend/stop/freeze LocalMapping thread
//                // Wait until Local Mapping has effectively stopped
//                while (!mpLocalMapper->isStopped() &&
//                       !mpLocalMapper->isFinished()) {//if LocalMapping is killed by System::Shutdown(), don't wait any more
//                    usleep(1000);
//                }
//
//                unique_lock<mutex> lockScale(
//                        mpMap->mMutexScaleUpdateLoopClosing);//notice we cannot update scale during LoopClosing or LocalBA!
//                unique_lock<mutex> lockScale2(mpMap->mMutexScaleUpdateGBA);
//                unique_lock<mutex> lock(mpMap->mMutexMapUpdate, std::defer_lock);  // Get Map Mutex
//                while (!lock.try_lock()) {
//                    if (GetReset()) {
//                        mpLocalMapper->Release();  // recover LocalMapping thread, same as CorrectLoop()
//                        return false;
//                    }
//                    usleep(3000);
//                }
//                //Update KFs' PRVB
//                //update the vScaleGravityKF to the current size, and pNewestKF is mpCurrentKeyFrame during the LocalMapping thread is stopped
//                vScaleGravityKF = mpMap->GetAllKeyFrames();
//                pNewestKF = GetCurrentKeyFrame();
//                assert(pNewestKF ==
//                       vScaleGravityKF.back());//they must be same for we change the set less func. in Map.h
//                //recover right scaled Twc&NavState from old unscaled Twc with scale
//                for (vector<KeyFrame *>::const_iterator vit = vScaleGravityKF.begin(), vend = vScaleGravityKF.end();
//                     vit != vend; ++vit) {
//                    KeyFrame *pKF = *vit;
//                    if (pKF->isBad()) continue;
//                    //we can SetPose() first even no IMU data
//                    cv::Mat Tcw = pKF->GetPose();
//                    SE3calc Twc = Converter::toSE3<Tcalc_sgba>(pKF->GetPoseInverse());//we must cache Twc first!
//                    cv::Mat tcw = Tcw.rowRange(0, 3).col(3) * scale;//right scaled pwc
//                    tcw.copyTo(Tcw.rowRange(0, 3).col(3));
//                    pKF->SetPose(Tcw);//manually SetPose(right scaled Tcw)
//                    // Position and rotation of visual SLAM
//                    Vector3calc wPc = Twc.translation();//wPc/twc
//                    SO3calc Rwc = Twc.so3();
//                    // Set position and rotation of navstate
//                    SE3calc Tbc = Converter::toSE3<Tcalc_sgba>(Frame::mTbc);
//                    SO3calc Rcb = Tbc.so3().inverse();
//                    Vector3calc pcb = -(Rcb * Tbc.translation());
//                    Vector3calc wPb = scale * wPc + Rwc * pcb;//right scaled pwb from right scaled pwc
//                    NavState ns;
//                    ns.mpwb = wPb.cast<double>();
//                    ns.mRwb = (Rwc * Rcb).cast<double>();
//                    ns.mbg = bgs_est[0];
//                    ns.mba = bas_star[0];//bg* ba*
//                    ns.mdbg = ns.mdba = Vector3d::Zero();// Set delta_bias to zero. (only updated during optimization)
//                    // Step 4. / See IV-D/(18)/(3) in VOIRBSLAM paper with ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
//                    // compute velocity
//                    if (pKF != vScaleGravityKF.back()) {
//                        KeyFrame *pKFnext = pKF->GetNextKeyFrame();
//                        assert(pKFnext && "pKFnext is NULL");
//                        if (pKFnext->GetIMUPreInt().mdeltatij == 0) {
//                            cout << "time 0" << endl;
//                            continue;
//                        }
//                        pKF->SetNavStateOnly(ns);//we must update the pKF->mbg&mba before pKFnext->PreIntegration()
//                        pKFnext->PreIntegration<IMUData>(
//                                pKF);//it's originally based on bi_bar=0, but now it's based on bi_bar=[bg* ba*], so dbgi=dbai=0
//                        const IMUPreintegrator imupreint = pKFnext->GetIMUPreInt();//IMU pre-int between pKF ~ pKFnext, though the paper seems to use the vKFInit[k].mOdomPreIntIMU so its dbgi=0 but its dbai=bai, we use more precise bi_bar here
//                        double dt = imupreint.mdeltatij;                                        // deltati_i+1
//                        Vector3calc dp = imupreint.mpij.cast<Tcalc_sgba>();//deltapi_i+1
//                        //cv::Mat Japij=Converter::toCvMat(imupreint.mJapij);    			// Ja_deltap
//                        SE3calc Twcnext = Converter::toSE3<Tcalc_sgba>(pKFnext->GetPoseInverse());
//                        Vector3calc wPcnext = Twcnext.translation().cast<Tcalc_sgba>();//wPci+1
//                        SO3calc Rwcnext = Twcnext.so3().cast<Tcalc_sgba>();//Rwci+1
//                        //-1/dt*(pwbi-pwbj+1/2*gw*dt^2+Rwbi*(dp+Japij*dbai)), pwbi=s*pwc+Rwc*pcb,
//                        // s=sw=swRightScaled_wNow
//                        Vector3calc vwbi = -1. / dt * (scale * (wPc - wPcnext) + (Rwc * pcb - Rwcnext * pcb) +
//                                                       dt * dt / 2 * gw + Rwc * Rcb * (dp));
//                        ns.mvwb = vwbi.cast<double>();
//                    } else {
//                        // If this is the last KeyFrame, no 'next' KeyFrame exists, use (3) in VOIRBSLAM paper with ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
//                        if (pKF->GetIMUPreInt().mdeltatij == 0) {
//                            cout << "time 0" << endl;
//                            continue;
//                        }
//                        KeyFrame *pKFprev = pKF->GetPrevKeyFrame();
//                        assert(pKFprev && "pKFnext is NULL");
//                        const IMUPreintegrator imupreint = pKF->GetIMUPreInt();//notice it's based on bi_bar=[bg* ba*], so dbgi=dbai=0
//                        double dt = imupreint.mdeltatij;
//                        NavState nsprev = pKFprev->GetNavState();
//                        //vwbj=vwbi+gw*dt+Rwbi*(dvij+Javij*dbai)
//                        ns.mvwb = nsprev.mvwb + gwd * dt + nsprev.mRwb * (imupreint.mvij);
//                    }
//                    pKF->SetNavStateOnly(ns);//now ns also has the right mvwb
//                }
//                //Update MPs' Position
//                vector<MapPoint *> vpMPs = mpMap->GetAllMapPoints();//we don't change the vpMPs[i] but change the *vpMPs[i]
//                for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; ++vit)
//                    (*vit)->UpdateScale(scale);
//                //Now every thing in Map is right scaled & mGravityVec is got
//                SetVINSInited(true);
//                mpMap->InformNewChange();//used to notice Tracking thread bMapUpdated
//
//                mpLocalMapper->Release();//recover LocalMapping thread, same as CorrectLoop()
//                std::cout << std::endl << "... Map scale & NavState updated ..." << std::endl << std::endl;
//                // Run global BA/full BA after inited, we use LoopClosing thread to do this job for safety!
////                Optimizer::GlobalBundleAdjustmentNavStatePRV(mpMap,GetGravityVec(),15,NULL,0,false,true/false);SetInitGBAOver(true);
//                SetInitGBA(true);
//            }
//        }
//
//        cout << yellowSTR"Used time in IMU Initialization="
//             << chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() << whiteSTR
//             << endl;
//
//        deleteKFs_ret(vKFsInit);
//        return bVIOInited;
//    }

bool IMUInitialization::TryInitVIO(void){//now it's the version cannot allow the KFs has no inter IMUData in initial 15s!!!
  chrono::steady_clock::time_point t1=chrono::steady_clock::now();
  
  //at least N>=4
  if(mpMap->KeyFramesInMap()<4){ return false;}//21,11, 4
  //Recording data in txt files for further analysis
  static bool fopened = false;
  static ofstream fgw,fscale,fbiasa,fcondnum,fbiasg;
  if(mTmpfilepath.length()>0&&!fopened){
    // Need to modify this to correct path
    fbiasg.open(mTmpfilepath+"biasg.txt");//optimized initial bg for these N KFs,3*1
    fgw.open(mTmpfilepath+"gw.txt");//gwafter then gw before,6*1
    fscale.open(mTmpfilepath+"scale.txt");//scale_fine then scale_rough, 2*1 only for Monocular camera//if (mbMonocular) 
    fbiasa.open(mTmpfilepath+"biasa.txt");//optimized initial ba for these N KFs,3*1
    fcondnum.open(mTmpfilepath+"condnum.txt");//for optimized x is 6*1 vector, see (19) in VOIRBSLAM paper, here just show these 6*1 raw data
    if(fbiasg.is_open()&& fgw.is_open() && (fscale.is_open()) && //!mbMonocular||
      fbiasa.is_open()&&fcondnum.is_open())
        fopened = true;
    else{
        cerr<<"file open error in TryInitVIO"<<endl;
        fopened = false;
    }
    fbiasg<<std::fixed<<std::setprecision(6);
    fgw<<std::fixed<<std::setprecision(6);
    fscale<<std::fixed<<std::setprecision(6);
    fbiasa<<std::fixed<<std::setprecision(6);
    fcondnum<<std::fixed<<std::setprecision(6);
  }

//   Optimizer::GlobalBundleAdjustment(mpMap, 10);//GBA by only vision 1stly, suggested by JingWang

  // Extrinsics
  cv::Mat Tbc = Frame::mTbc;
  cv::Mat Rbc = Tbc.rowRange(0,3).colRange(0,3);
  cv::Mat pbc = Tbc.rowRange(0,3).col(3);
  cv::Mat Rcb = Rbc.t();
  cv::Mat pcb = -Rcb*pbc;

  // Cache KFs / wait for KeyFrameCulling() over
  while(GetCopyInitKFs()) usleep(1000);
  SetCopyInitKFs(true);//stop KeyFrameCulling() when this copying KFs
//   if(mpMap->KeyFramesInMap()<4){ SetCopyInitKFs(false);return false;}//ensure no KeyFrameCulling() during the start of this func. till here

  //see VIORBSLAM paper IV, here N=all KFs in map, not the meaning of local KFs' number
  // Use all KeyFrames in map to compute
  vector<KeyFrame*> vScaleGravityKF = mpMap->GetAllKeyFrames();
  //sort(vScaleGravityKF.begin(),vScaleGravityKF.end(),[](const KeyFrame *a,const KeyFrame *b){return a->mnId<b->mnId;});//we change the set less/compare func. so that we don't need to sort them!
  assert((*vScaleGravityKF.begin())->mnId==0);
  int N=0,NvSGKF=vScaleGravityKF.size();
  KeyFrame* pNewestKF = vScaleGravityKF[NvSGKF-1];
  // Store initialization-required KeyFrame data
  vector<IMUKeyFrameInit*> vKFInit;

  for(int i=0;i<NvSGKF;++i){
    KeyFrame* pKF = vScaleGravityKF[i];
//     if (pKF->mTimeStamp<pNewestKF->mTimeStamp-15) continue;//15s as the VIORBSLAM paper
    IMUKeyFrameInit* pkfi=new IMUKeyFrameInit(*pKF);
    if(N>0) pkfi->mpPrevKeyFrame=vKFInit[N-1];
    vKFInit.push_back(pkfi);
    ++N;
  }

  SetCopyInitKFs(false);

  // Step 1. / see VIORBSLAM paper IV-A
  // Try to compute initial gyro bias, using optimization with Gauss-Newton
  Vector3d bgest;
  Optimizer::OptimizeInitialGyroBias<IMUKeyFrameInit>(vKFInit, bgest);//nothing changed, just return the optimized result bg*
  cout<<"bgest: "<<bgest<<endl;

  // Update biasg and pre-integration in LocalWindow(here all KFs).
  for(int i=0;i<N;++i) vKFInit[i]->mbg_=bgest;
  for(int i=1;i<N;++i) vKFInit[i]->ComputePreInt();//so vKFInit[i].mOdomPreIntIMU is based on bg_bar=bgest,ba_bar=0; dbg=0 but dba/ba waits to be optimized

  // Step 2. / See VIORBSLAM paper IV-B
  // Approx Scale and Gravity vector in 'world' frame (first/0th KF's camera frame)
  // Solve A*x=B for x=[s,gw] 4x1 vector, using SVD method
  cv::Mat A=cv::Mat::zeros(3*(N-2),4,CV_32F);//4 unknowns so N must >=4
  cv::Mat B=cv::Mat::zeros(3*(N-2),1,CV_32F);
  cv::Mat I3=cv::Mat::eye(3,3,CV_32F);
  int numEquations=0;
  for(int i=0; i<N-2; ++i){
    IMUKeyFrameInit *pKF2=vKFInit[i+1],*pKF3=vKFInit[i+2];
    double dt12 = pKF2->mOdomPreIntIMU.mdeltatij;//deltat12
    double dt23 = pKF3->mOdomPreIntIMU.mdeltatij;
    if (dt12==0||dt23==0){ cout<<redSTR<<"Tm="<<pKF2->mTimeStamp<<" lack IMU data!"<<whiteSTR<<endl;continue;}
    ++numEquations;
    // Pre-integrated measurements
    cv::Mat dp12=Converter::toCvMat(pKF2->mOdomPreIntIMU.mpij);//deltap12
    cv::Mat dv12=Converter::toCvMat(pKF2->mOdomPreIntIMU.mvij);
    cv::Mat dp23=Converter::toCvMat(pKF3->mOdomPreIntIMU.mpij);
//     cout<<fixed<<setprecision(6);
//     cout<<"dt12:"<<dt12<<" KF1:"<<vKFInit[i]->mTimeStamp<<" KF2:"<<pKF2->mTimeStamp<<" dt23:"<<dt23<<" KF3:"<<pKF3->mTimeStamp<<endl;
//     cout<<dp12.t()<<" 1id:"<<vScaleGravityKF[i]->mnId<<" 2id:"<<vScaleGravityKF[i+1]->mnId<<" 3id:"<<vScaleGravityKF[i+2]->mnId<<endl;
//     cout<<" Size12="<<pKF2->mOdomPreIntIMU.getlOdom().size()<<" Size23="<<pKF3->mOdomPreIntIMU.getlOdom().size()<<endl;
    // Pose of camera in world frame
    cv::Mat Twc1=vKFInit[i]->mTwc;//Twci for pwci&Rwci, not necessary for clone()
    cv::Mat Twc2=pKF2->mTwc;cv::Mat Twc3=pKF3->mTwc;
    cv::Mat pc1=Twc1.rowRange(0,3).col(3);//pwci
    cv::Mat pc2=Twc2.rowRange(0,3).col(3);
    cv::Mat pc3=Twc3.rowRange(0,3).col(3);
    cv::Mat Rc1=Twc1.rowRange(0,3).colRange(0,3);//Rwci
    cv::Mat Rc2=Twc2.rowRange(0,3).colRange(0,3);
    cv::Mat Rc3=Twc3.rowRange(0,3).colRange(0,3);

    // fill A/B matrix: lambda*s + beta*g = gamma(3*1), Ai(3*4)=[lambda beta], (13) in the paper
    cv::Mat lambda=(pc2-pc1)*dt23+(pc2-pc3)*dt12;
    cv::Mat beta=(dt12*dt12*dt23+dt12*dt23*dt23)/2*I3;
    cv::Mat gamma=(Rc1-Rc2)*pcb*dt23+(Rc3-Rc2)*pcb*dt12-Rc2*Rcb*dp23*dt12-Rc1*Rcb*dv12*dt12*dt23+Rc1*Rcb*dp12*dt23;
    lambda.copyTo(A.rowRange(3*i+0,3*i+3).col(0));
    beta.copyTo(A.rowRange(3*i+0,3*i+3).colRange(1,4));//Ai
    gamma.copyTo(B.rowRange(3*i+0,3*i+3));//gamma/B(i), but called gamma(i) in the paper
    // JingWang tested the formulation in paper, -gamma. Then the scale and gravity vector is -xx, or we can say the papaer missed a minus before γ(i)
  }
  if (numEquations<4){//for more robust judgement instead of judging KeyFramesInMap()
    for(int i=0;i<N;i++){//delete the newed pointer
      if(vKFInit[i]) delete vKFInit[i];
    }
    return false;
  }
  // Use svd to compute A*x=B, x=[s,gw] 4x1 vector
  // A=u*S*vt=u*w*vt, u*w*vt*x=B => x=vt'*winv*u'*B, or we call the pseudo inverse of A/A.inv()=(A.t()*A).inv()*A.t(), in SVD we have A.inv()=v*winv*u.t() where winv is the w with all nonzero term is the reciprocal of the corresponding singular value
  cv::Mat w,u,vt;// Note w is 4x1 vector by SVDecomp()/SVD::compute() not the 4*4(not FULL_UV)/m*n(FULL_UV) singular matrix we stated last line
  cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A);// A is changed in SVDecomp()(just calling the SVD::compute) with cv::SVD::MODIFY_A for speed
  cv::Mat winv=cv::Mat::eye(4,4,CV_32F);
  for(int i=0;i<4;++i){
    if(fabs(w.at<float>(i))<1e-10){//too small in sufficient w meaning the linear dependent equations causing the solution is not unique(or A.inv() not exist)
      w.at<float>(i) += 1e-10;
      cerr<<"w(i) < 1e-10, w="<<endl<<w<<endl;
    }
    winv.at<float>(i,i)=1./w.at<float>(i);
  }
  cv::Mat x=vt.t()*winv*u.t()*B;
  double sstar=x.at<float>(0);		// scale should be positive
  cv::Mat gwstar=x.rowRange(1,4);	// gravity should be about ~9.8
  cout<<"gwstar: "<<gwstar.t()<<", |gwstar|="<<cv::norm(gwstar)<<endl;

  // Step 3. / See VIORBSLAM paper IV-C
  cv::Mat Rwi;//for Recording
  cv::Mat w2,u2,vt2;// Note w2 is 6x1 vector by SVDecomp(), for Recording
  Eigen::Matrix3d Rwieig_;//for Recording
  // Use gravity magnitude 9.810 as constraint; gIn/^gI=[0;0;1], the normalized gravity vector in an inertial frame, we can also choose gIn=[0;0;-1] as the VIORBSLAM paper
  cv::Mat gIn=cv::Mat::zeros(3,1,CV_32F);gIn.at<float>(2)=1;
  cv::Mat GI=gIn*IMUData::mdRefG;//gI or GI=^gI*G
  double s_;
  cv::Mat Rwi_;
  Vector3d bastareig;
//   for (int k=0;k<2;++k){//we prefer 1 iteration
//     if (k==1){
//       gwstar=Rwi_*GI;
//       for(int i=0;i<N;++i) vKFInit[i]->mba_=bastareig;
//       for(int i=1;i<N;++i) vKFInit[i]->ComputePreInt();
//     }
    
    cv::Mat gwn=gwstar/cv::norm(gwstar);//^gw=gw*/||gw*|| / Normalized approx. gravity vecotr in world frame
    cv::Mat gInxgwn=gIn.cross(gwn);
    double normgInxgwn=cv::norm(gInxgwn);
    cv::Mat vhat=gInxgwn/normgInxgwn;//RwI=Exp(theta*^v), or we can call it vn=(gI x gw)/||gI x gw||
    double theta=std::atan2(normgInxgwn,gIn.dot(gwn));//notice theta*^v belongs to [-Pi,Pi]*|^v| though theta belongs to [0,Pi]
    Matrix3d RWIeig=IMUPreintegrator::Expmap(Converter::toVector3d(vhat)*theta);Rwi=Converter::toCvMat(RWIeig);//RwI
    
    // Solve C*x=D for x=[s,dthetaxy,ba] (1+2+3)x1 vector
    cv::Mat C=cv::Mat::zeros(3*(N-2),6,CV_32F);
    cv::Mat D=cv::Mat::zeros(3*(N-2),1,CV_32F);
    for(int i=0; i<N-2; i++){
      IMUKeyFrameInit *pKF2=vKFInit[i+1],*pKF3 = vKFInit[i+2];
      const IMUPreintegrator &imupreint12=pKF2->mOdomPreIntIMU,&imupreint23=pKF3->mOdomPreIntIMU;
      //d means delta
      double dt12=imupreint12.mdeltatij;
      double dt23=imupreint23.mdeltatij;
      if (dt12==0||dt23==0) continue;
      cv::Mat dp12=Converter::toCvMat(imupreint12.mpij);
      cv::Mat dp23=Converter::toCvMat(imupreint23.mpij);
      cv::Mat dv12=Converter::toCvMat(imupreint12.mvij);
      cv::Mat Jav12=Converter::toCvMat(imupreint12.mJavij);
      cv::Mat Jap12 = Converter::toCvMat(imupreint12.mJapij);
      cv::Mat Jap23=Converter::toCvMat(imupreint23.mJapij);
      cv::Mat Twc1=vKFInit[i]->mTwc;//Twci for pwci&Rwci, not necessary for clone()
      cv::Mat Twc2=pKF2->mTwc;
      cv::Mat Twc3=pKF3->mTwc;
      cv::Mat pc1=Twc1.rowRange(0,3).col(3);//pwci
      cv::Mat pc2=Twc2.rowRange(0,3).col(3);
      cv::Mat pc3=Twc3.rowRange(0,3).col(3);
      cv::Mat Rc1=Twc1.rowRange(0,3).colRange(0,3);//Rwci
      cv::Mat Rc2=Twc2.rowRange(0,3).colRange(0,3);
      cv::Mat Rc3=Twc3.rowRange(0,3).colRange(0,3);
      // Stack to C/D matrix; lambda*s + phi(:,0:1)*dthetaxy + zeta*ba = psi, Ci(3*6),Di/psi(3*1)
      cv::Mat lambda=(pc2-pc1)*dt23+(pc2-pc3)*dt12;//3*1
      cv::Mat phi=-(dt12*dt12*dt23+dt12*dt23*dt23)/2*Rwi*SkewSymmetricMatrix(GI);//3*3 note: this has a '-', different to paper
      cv::Mat zeta=Rc2*Rcb*Jap23*dt12+Rc1*Rcb*Jav12*dt12*dt23-Rc1*Rcb*Jap12*dt23;//3*3 notice here is Jav12, paper writes a wrong Jav23
      cv::Mat psi=(Rc1-Rc2)*pcb*dt23+(Rc3-Rc2)*pcb*dt12-Rc2*Rcb*dp23*dt12-Rc1*Rcb*dv12*dt12*dt23//note:  - paper & deltatij^2 in paper means dt12^2*dt23+dt23^2*dt12
      +Rc1*Rcb*dp12*dt23-(dt12*dt12*dt23+dt12*dt23*dt23)/2*(Rwi*GI);//notice here use Rwi*GI instead of gwstar for it's designed for iterative usage
      lambda.copyTo(C.rowRange(3*i+0,3*i+3).col(0));
      phi.colRange(0,2).copyTo(C.rowRange(3*i+0,3*i+3).colRange(1,3));//phi(:,0:1)(3*2) / only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
      zeta.copyTo(C.rowRange(3*i+0,3*i+3).colRange(3,6));
      psi.copyTo(D.rowRange(3*i+0,3*i+3));
    }
    // Use svd to compute C*x=D, x=[s,dthetaxy,ba] 6x1 vector
    cv::SVD::compute(C,w2,u2,vt2,cv::SVD::MODIFY_A);
    cv::Mat w2inv=cv::Mat::eye(6,6,CV_32F);
    for(int i=0;i<6;++i){
      if(fabs(w2.at<float>(i))<1e-10){
        w2.at<float>(i) += 1e-10;
        cerr<<"w2(i) < 1e-10, w="<<endl<<w2<<endl;
      }
      w2inv.at<float>(i,i)=1./w2.at<float>(i);
    }
    cv::Mat y=vt2.t()*w2inv*u2.t()*D;// Then y/x = vt'*winv*u'*D
    s_=y.at<float>(0);//s*_C, C means IV-C in the paper
    Eigen::Vector3d dthetaeig(y.at<float>(1),y.at<float>(2),0);//small deltatheta/dtheta=[dthetaxy.t() 0].t()
    Rwieig_=RWIeig*IMUPreintegrator::Expmap(dthetaeig);//RwI*_C=RwI*_B*Exp(dtheta)
    Rwi_=Converter::toCvMat(Rwieig_);
//     if (k==0)
    bastareig=Converter::toVector3d(y.rowRange(3,6));//here bai_bar=0, so dba=ba
//     else bastareig+=Converter::toVector3d(y.rowRange(3,6));
//   }
  

  // Record data for analysis
  cv::Mat gwbefore=Rwi*GI,gwafter=Rwi_*GI;//direction of gwbefore is the same as gwstar, but value is different!
  cout<<"gwbefore="<<gwbefore<<", gwafter="<<gwafter<<endl;
  
  cout<<"Time: "<<pNewestKF->mTimeStamp-mdStartTime<<", sstar: "<<sstar<<", s: "<<s_<<endl;//Debug the frequency & sstar2&sstar
  //<<" bgest: "<<bgest.transpose()<<", gw*(gwafter)="<<gwafter.t()<<", |gw*|="<<cv::norm(gwafter)<<", norm(gwbefore,gwstar)"<<cv::norm(gwbefore.t())<<" "<<cv::norm(gwstar.t())<<endl;
  if (mTmpfilepath.length()>0){//Debug the Rwistar2
    ofstream fRwi(mTmpfilepath+"Rwi.txt");
    fRwi<<Rwieig_(0,0)<<" "<<Rwieig_(0,1)<<" "<<Rwieig_(0,2)<<" "
	<<Rwieig_(1,0)<<" "<<Rwieig_(1,1)<<" "<<Rwieig_(1,2)<<" "
	<<Rwieig_(2,0)<<" "<<Rwieig_(2,1)<<" "<<Rwieig_(2,2)<<endl;
    fRwi.close();
  }
  fbiasg<<pNewestKF->mTimeStamp<<" "<<bgest(0)<<" "<<bgest(1)<<" "<<bgest(2)<<" "<<endl;
  fgw<<pNewestKF->mTimeStamp<<" "<<gwafter.at<float>(0)<<" "<<gwafter.at<float>(1)<<" "<<gwafter.at<float>(2)<<" "
      <<gwbefore.at<float>(0)<<" "<<gwbefore.at<float>(1)<<" "<<gwbefore.at<float>(2)<<" "<<endl;
  fscale<<pNewestKF->mTimeStamp<<" "<<s_<<" "<<sstar<<" "<<endl;//if (mbMonocular) 
  fbiasa<<pNewestKF->mTimeStamp<<" "<<bastareig[0]<<" "<<bastareig[1]<<" "<<bastareig[2]<<" "<<endl;
  fcondnum<<pNewestKF->mTimeStamp<<" "<<w2.at<float>(0)<<" "<<w2.at<float>(1)<<" "<<w2.at<float>(2)<<" "<<w2.at<float>(3)<<" "<<w2.at<float>(4)<<" "<<w2.at<float>(5)<<" "<<endl;

  // ********************************
  // Todo: Add some logic or strategy to confirm init status, VIORBSLAM paper just uses 15 seconds to confirm
  bool bVIOInited = false;
  if(mdStartTime<0) mdStartTime=pNewestKF->mTimeStamp;
  if(pNewestKF->mTimeStamp-mdStartTime>=mdFinalTime){//15s in the paper V-A
    cout<<yellowSTR"condnum="<<w2.at<float>(0)<<";"<<w2.at<float>(5)<<whiteSTR<<endl;
//     if (w2.at<float>(0)/w2.at<float>(5)<700)
      bVIOInited = true;
  }

  //if VIO is initialized with appropriate bg*,s*,gw*,ba*, update the map(MPs' P,KFs' PRVB) like GBA/CorrrectLoop()
  if(bVIOInited){
    // Set NavState , scale and bias for all KeyFrames
    double scale=s_;
    // gravity vector in world frame
    cv::Mat gw;
    {
      unique_lock<mutex> lock(mMutexInitIMU);
      mGravityVec=Rwi_*GI;gw=mGravityVec.clone();
    }
    Vector3d gweig = Converter::toVector3d(gw);

    {// Update the Map needs mutex lock: Stop local mapping, like RunGlobalBundleAdjustment() in LoopClosing.cc
      mpLocalMapper->RequestStop();//same as CorrectLoop(), suspend/stop/freeze LocalMapping thread
      // Wait until Local Mapping has effectively stopped
      while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished()){//if LocalMapping is killed by System::Shutdown(), don't wait any more
        usleep(1000);
      }

      unique_lock<mutex> lockScale(mpMap->mMutexScaleUpdateLoopClosing);//notice we cannot update scale during LoopClosing or LocalBA!
      unique_lock<mutex> lockScale2(mpMap->mMutexScaleUpdateGBA);
      unique_lock<mutex> lock(mpMap->mMutexMapUpdate, std::defer_lock);  // Get Map Mutex
      while (!lock.try_lock()) {
          if (GetReset()) {
              mpLocalMapper->Release();  // recover LocalMapping thread, same as CorrectLoop()
              return false;
          }
          usleep(3000);
      }
      //Update KFs' PRVB
      //update the vScaleGravityKF to the current size, and pNewestKF is mpCurrentKeyFrame during the LocalMapping thread is stopped
      vScaleGravityKF=mpMap->GetAllKeyFrames();
      pNewestKF=GetCurrentKeyFrame();
      assert(pNewestKF==vScaleGravityKF.back());//they must be same for we change the set less func. in Map.h
      //recover right scaled Twc&NavState from old unscaled Twc with scale
      for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; ++vit){
	KeyFrame* pKF = *vit;
	if(pKF->isBad()) continue;
	//we can SetPose() first even no IMU data
	cv::Mat Tcw=pKF->GetPose(),Twc=pKF->GetPoseInverse();//we must cache Twc first!
	cv::Mat tcw=Tcw.rowRange(0,3).col(3)*scale;//right scaled pwc
	tcw.copyTo(Tcw.rowRange(0,3).col(3));pKF->SetPose(Tcw);//manually SetPose(right scaled Tcw)
	// Position and rotation of visual SLAM
	cv::Mat wPc = Twc.rowRange(0,3).col(3);                   // wPc/twc
	cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);            // Rwc
	// Set position and rotation of navstate
	cv::Mat wPb = scale*wPc + Rwc*pcb;//right scaled pwb from right scaled pwc
	NavState ns;
	ns.mpwb=Converter::toVector3d(wPb);
	ns.setRwb(Converter::toMatrix3d(Rwc*Rcb));
	ns.mbg=bgest;ns.mba=bastareig;//bg* ba*
	ns.mdbg=ns.mdba=Vector3d::Zero();// Set delta_bias to zero. (only updated during optimization)
	// Step 4. / See IV-D/(18)/(3) in VOIRBSLAM paper with ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
	// compute velocity
	if(pKF!=vScaleGravityKF.back()){
	  KeyFrame* pKFnext=pKF->GetNextKeyFrame();
	  assert(pKFnext&&"pKFnext is NULL");
	  if (pKFnext->GetIMUPreInt().mdeltatij==0){cout<<"time 0"<<endl;continue;}
	  pKF->SetNavStateOnly(ns);//we must update the pKF->mbg&mba before pKFnext->PreIntegration()
	  pKFnext->PreIntegration<IMUData>(pKF);//it's originally based on bi_bar=0, but now it's based on bi_bar=[bg* ba*], so dbgi=dbai=0
	  const IMUPreintegrator imupreint=pKFnext->GetIMUPreInt();//IMU pre-int between pKF ~ pKFnext, though the paper seems to use the vKFInit[k].mOdomPreIntIMU so its dbgi=0 but its dbai=bai, we use more precise bi_bar here
	  double dt=imupreint.mdeltatij;                                		// deltati_i+1
	  cv::Mat dp=Converter::toCvMat(imupreint.mpij);       			// deltapi_i+1
	  //cv::Mat Japij=Converter::toCvMat(imupreint.mJapij);    			// Ja_deltap
	  cv::Mat wPcnext=pKFnext->GetPoseInverse().rowRange(0,3).col(3);		// wPci+1
	  cv::Mat Rwcnext=pKFnext->GetPoseInverse().rowRange(0,3).colRange(0,3);	// Rwci+1
	  cv::Mat vwbi=-1./dt*(scale*(wPc-wPcnext)+(Rwc-Rwcnext)*pcb+dt*dt/2*gw+Rwc*Rcb*(dp));//-1/dt*(pwbi-pwbj+1/2*gw*dt^2+Rwbi*(dp+Japij*dbai)), pwbi=s*pwc+Rwc*pcb, s=sw=swRightScaled_wNow
	  ns.mvwb=Converter::toVector3d(vwbi);
	}else{
	  // If this is the last KeyFrame, no 'next' KeyFrame exists, use (3) in VOIRBSLAM paper with ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
	  if (pKF->GetIMUPreInt().mdeltatij==0){cout<<"time 0"<<endl;continue;}
	  KeyFrame* pKFprev=pKF->GetPrevKeyFrame();
	  assert(pKFprev&&"pKFnext is NULL");
	  const IMUPreintegrator imupreint=pKF->GetIMUPreInt();//notice it's based on bi_bar=[bg* ba*], so dbgi=dbai=0
	  double dt=imupreint.mdeltatij;
	  NavState nsprev=pKFprev->GetNavState();
	  ns.mvwb=nsprev.mvwb+gweig*dt+nsprev.mRwb*(imupreint.mvij);//vwbj=vwbi+gw*dt+Rwbi*(dvij+Javij*dbai)
	}
	pKF->SetNavStateOnly(ns);//now ns also has the right mvwb
      }
      //Update MPs' Position
      vector<MapPoint*> vpMPs=mpMap->GetAllMapPoints();//we don't change the vpMPs[i] but change the *vpMPs[i]
      for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; ++vit) (*vit)->UpdateScale(scale);
      //Now every thing in Map is right scaled & mGravityVec is got
      if (!mbUsePureVision) SetVINSInited(true);
      mpMap->InformNewChange();//used to notice Tracking thread bMapUpdated
      
      mpLocalMapper->Release();//recover LocalMapping thread, same as CorrectLoop()
      std::cout<<std::endl<<"... Map scale & NavState updated ..."<<std::endl<<std::endl;
      // Run global BA/full BA after inited, we use LoopClosing thread to do this job for safety!
//       Optimizer::GlobalBundleAdjustmentNavStatePRV(mpMap,GetGravityVec(),15,NULL,0,false,true/false);SetInitGBAOver(true);
      SetInitGBA(true);
    }
  }
  
  cout<<yellowSTR"Used time in IMU Initialization="<<chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now()-t1).count()<<whiteSTR<<endl;

  for(int i=0;i<N;i++){//delete the newed pointer
    if(vKFInit[i]) delete vKFInit[i];
  }
  return bVIOInited;
}
  
}