#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace std;

int main()
{

    cout << "Hello Eigen!!!" << endl;
    cout << "This is pr2 tf frame relation!!!" << endl;
    /*
       MIT STEREO AND IMU EXTRINIC PARAMETERS
    */
    //ISO of base_footprint as Odometry
    Eigen::Isometry3d BFP_ISO = Eigen::Isometry3d::Identity();
    cout << "Odometry is same as Base_footprint:\n" << BFP_ISO.matrix() << endl;

    //ISO of imu to BFP
    Eigen::Isometry3d IMU_ISO = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond IMU_QUAT(-1.03411555577e-13,0.0,1.0,0.0);
    Eigen::Vector3d IMU_POS(-0.07977,-0.1497,0.966177604299);
    IMU_ISO.rotate(IMU_QUAT);
    IMU_ISO.pretranslate(IMU_POS);
    cout << "IMU to Base_footprint:\n" << IMU_ISO.matrix() << endl;
/*
    //ISO of left optical camera to BFP
    Eigen::Isometry3d LOC_ISO = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond LOC_QUAT(-0.492570567439,0.508333876454,-0.507621612711,0.491214010845);
    Eigen::Vector3d LOC_POS(0.0740592638618,0.0310416890391,1.30162657671);
    LOC_ISO.rotate(LOC_QUAT);
    LOC_ISO.pretranslate(LOC_POS);
    cout << "Left Optical Camera to Base_footprint:\n" << LOC_ISO.matrix() << endl;

    //ISO of right optical camera to BFP
    Eigen::Isometry3d ROC_ISO = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond ROC_QUAT(-0.492570567439,0.508333876454,-0.507621612711,0.491214010845);
    Eigen::Vector3d ROC_POS(0.0711643181657,-0.0589117187429,1.30168755054);
    ROC_ISO.rotate(ROC_QUAT);
    ROC_ISO.pretranslate(ROC_POS);
    cout << "Right Optical Camera to Base_footprint:\n" << ROC_ISO.matrix() << endl;
*/
    //ISO of left optical camera to BFP
    Eigen::Isometry3d LOC_ISO = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond LOC_QUAT(0.491214010845,-0.492570567439,0.508333876454,-0.507621612711);
    Eigen::Vector3d LOC_POS(0.0740592638618,0.0310416890391, 1.30162657671);
    LOC_ISO.rotate(LOC_QUAT);
    LOC_ISO.pretranslate(LOC_POS);
    cout << "Left Optical Camera to Base_footprint:\n" << LOC_ISO.matrix() << endl;
    cout << "Body to LOC:\n" << LOC_ISO.inverse().matrix() << endl;
    //ISO of right optical camera to BFP
    Eigen::Isometry3d ROC_ISO = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond ROC_QUAT( 0.491214010845, -0.492570567439,0.508333876454,-0.507621612711);
    Eigen::Vector3d ROC_POS(0.0711643181657,-0.0589117187429,1.30168755054);
    ROC_ISO.rotate(ROC_QUAT);
    ROC_ISO.pretranslate(ROC_POS);

    cout << "Right Optical Camera to Base_footprint:\n" << ROC_ISO.matrix() << endl;
    cout << "Body to ROC:\n" << ROC_ISO.inverse().matrix() << endl;
    //VINS-Fusion need imu_T_stereo
    Eigen::Isometry3d imu_T_LOC_tmp = LOC_ISO.inverse()*IMU_ISO;
    Eigen::Isometry3d imu_T_LOC = imu_T_LOC_tmp.inverse();
    cout << "IMU to Left Optical Camera:\n" <<imu_T_LOC.matrix()(0,0) << "," <<imu_T_LOC.matrix()(0,1) << ","<<imu_T_LOC.matrix()(0,2) << ","<<imu_T_LOC.matrix()(0,3) << ","<< endl
                                            <<imu_T_LOC.matrix()(1,0) << "," <<imu_T_LOC.matrix()(1,1) << ","<<imu_T_LOC.matrix()(1,2) << ","<<imu_T_LOC.matrix()(1,3) << ","<< endl
                                            <<imu_T_LOC.matrix()(2,0) << "," <<imu_T_LOC.matrix()(2,1) << ","<<imu_T_LOC.matrix()(2,2) << ","<<imu_T_LOC.matrix()(2,3) << ","<< endl;
    Eigen::Isometry3d imu_T_ROC_tmp = ROC_ISO.inverse()*IMU_ISO;
    Eigen::Isometry3d imu_T_ROC = imu_T_ROC_tmp.inverse();
    cout << "IMU to Right Optical Camera:\n" <<imu_T_ROC.matrix()(0,0) << "," <<imu_T_ROC.matrix()(0,1) << ","<<imu_T_ROC.matrix()(0,2) << ","<<imu_T_ROC.matrix()(0,3) << ","<< endl
                                             <<imu_T_ROC.matrix()(1,0) << "," <<imu_T_ROC.matrix()(1,1) << ","<<imu_T_ROC.matrix()(1,2) << ","<<imu_T_ROC.matrix()(1,3) << ","<< endl
                                             <<imu_T_ROC.matrix()(2,0) << "," <<imu_T_ROC.matrix()(2,1) << ","<<imu_T_ROC.matrix()(2,2) << ","<<imu_T_ROC.matrix()(2,3) << ","<< endl;
    Eigen::Isometry3d ROC_T_LOC = LOC_ISO.inverse()*ROC_ISO;
    cout << "Right to Left Optical Camera:\n" << ROC_T_LOC.matrix() << endl;
    /*
    Eigen::Isometry3d begin_iso = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond begin_quat(-0.000249630797544,-0.00273412118978,-0.018337669173,0.9998280813);
    Eigen::Vector3d begin_tran(0.0733058400331,0.0304056285943,1.30217590723);
    Eigen::Matrix3d rotate_matrix = begin_quat.toRotationMatrix();
    begin_iso.rotate(rotate_matrix);
    begin_iso.pretranslate(begin_tran);
    cout << "Begin ISO:\n" << begin_iso.matrix()  << endl;

    Eigen::Isometry3d end_iso = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d end_opt_iso = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond end_quat(-0.000322146425484,0.001034410171178,-0.0160854554433,0.999870033725);
    Eigen::Vector3d end_tran(0.0740592638618,0.0310416890391,1.30162657671);
    Eigen::Quaterniond end_opt_quat(-0.492570567439,0.508333876454,-0.507621612711,0.491214010845);

    Eigen::Matrix3d rotate_matrix1 = begin_quat.toRotationMatrix();
    end_iso.rotate(rotate_matrix1);
    end_iso.pretranslate(end_tran);

    end_opt_iso.rotate(end_opt_quat);
    end_opt_iso.pretranslate(end_tran);
    cout << "End ISO:\n" << end_iso.matrix()  << endl;

    Eigen::Isometry3d T = begin_iso*end_iso.inverse();
    cout << "T ISO:\n" << T.matrix() <<endl;
    Eigen::Isometry3d T_opt = end_iso.inverse()*end_opt_iso;
    cout << "T Opt:\n" << T_opt.matrix() <<endl;
    Eigen::Vector3d angle = T_opt.rotation().eulerAngles(2,1,0)*180/3.1415926;
    cout << "T angle:\n" << angle << endl;
    */

    /*
     TSINGHUA STEREO AND IMU EXTRINIC PARAMETERS
    */
    //Transform from FE2 to FE1
    Eigen::Isometry3d T_FE1_FE2 = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond R_FE1_FE2_QUAT(0.999991,0.00188497,0.00347595,0.00154952);
    Eigen::Vector3d P_FE1_FE2_POS(0.0639765113592,0.000148267135955,-0.000398468371714);
    T_FE1_FE2.rotate(R_FE1_FE2_QUAT);
    T_FE1_FE2.pretranslate(P_FE1_FE2_POS);
    cout << "Transform from FE2 to FE1: \n" << T_FE1_FE2.matrix() << endl;

    //Transform from IMU to FE1
    Eigen::Isometry3d T_FE1_IMU = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond R_FE1_IMU_QUAT(-0.00245956,0.00554841,0.00136098,0.99998062);
    Eigen::Vector3d P_FE1_IMU_POS(0.0106999985874,7.27595761418e-12,-2.91038304567e-11);
    T_FE1_IMU.rotate(R_FE1_IMU_QUAT);
    T_FE1_IMU.pretranslate(P_FE1_IMU_POS);
    cout << "Transform from IMU to FE1: \n" << T_FE1_IMU.matrix() << endl;


    //Transform from FE1 to IMU

    Eigen::Isometry3d T_FE2_IMU = T_FE1_FE2.inverse()*T_FE1_IMU;
    cout << "Transform from IMU to FE2: \n" << T_FE2_IMU.matrix() << endl;
    Eigen::Isometry3d T_FE1_FE2_tmp = T_FE1_IMU*T_FE2_IMU.inverse();
    Eigen::Quaterniond quat(T_FE1_FE2_tmp.rotation());
    cout << "quat: " << quat.w() << " " << quat.vec().transpose() <<endl;
    //cout << "Experimatal t:\n" << -T_FE1_IMU.rotation().transpose()*T_FE1_IMU.translation();

    //anker extrinic parameters
    Eigen::Isometry3d T_cam0_imu = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R_cam0_imu;
    R_cam0_imu << 0.006651836334813421, -0.9999669365458568, 0.004677487409696868,0.28680418806605457, -0.0025732769411657697, -0.957985770224983,0.9579661323768939, 0.007713887533344066, 0.2867775883118778;
    Eigen::Vector3d t_cam0_imu = Eigen::Vector3d(0.045454517934126325,0.03324307438799412, -0.014263154321419197);
    T_cam0_imu.rotate(R_cam0_imu);
    T_cam0_imu.pretranslate(t_cam0_imu);
    cout << "T_cam0_imu: \n" << T_cam0_imu.inverse().matrix() << endl;
    Eigen::Isometry3d T_cam1_imu = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R_cam1_imu;
    R_cam1_imu << 0.015131797539633451, -0.9994557657860389, 0.029312129575691404,0.2974404345079108, -0.02348897788717852, -0.9544513899814704,0.9546204568429092, 0.023161177751967132, 0.29692312678903443;
    Eigen::Vector3d t_cam1_imu = Eigen::Vector3d(-0.025638012940589566,0.033160873115852606,-0.014817388710937712);
    T_cam1_imu.rotate(R_cam1_imu);
    T_cam1_imu.pretranslate(t_cam1_imu);
    cout << "T_cam1_imu: \n" << T_cam1_imu.inverse().matrix() << endl;

}

/*
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs:         0
      frame_id: "t265_fisheye1"
    child_frame_id: "t265_fisheye2"
    transform:
      translation:
        x: 0.0639765113592
        y: 0.000148267135955
        z: -0.000398468371714
      rotation:
        x: 0.00188497
        y: 0.00347595
        z: 0.00154952
        w: 0.999991
  -
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs:         0
      frame_id: "t265_fisheye1"
    child_frame_id: "t265_imu"
    transform:
      translation:
        x: 0.0106999985874
        y: 7.27595761418e-12
        z: -2.91038304567e-11
      rotation:
        x: 0.00554841
        y: 0.00136098
        z: 0.99998062
        w: -0.00245956
---

*/
/*
 *
 * Begin
 * IMU
      translation:
        x: -0.07977
        y: -0.1497
        z: 0.966178909698
      rotation:
        x: 0.0
        y: 1.0
        z: 0.0
        w: -1.03411555577e-13
 * LOC
      translation:
        x: 0.0741019141012
        y: 0.0303910674988
        z: 1.30169003168
      rotation:
        x: -0.491184959448
        y: 0.509262051439
        z: -0.508957694678
        w: 0.490256630351
  * ROC
     translation:
        x: 0.0707924751392
        y: -0.0595480465056
        z: 1.3017482198
      rotation:
        x: -0.491184959448
        y: 0.509262051439
        z: -0.508957694678
        w: 0.490256630351


  **END
  * LOC
     translation:
        x: 0.0743183366663
        y: 0.0303614927378
        z: 1.30155866817
      rotation:
        x: -0.491584495607
        y: 0.509749637711
        z: -0.50857164943
        w: 0.489749801356
  * ROC
      translation:
        x: 0.0709901698307
        y: -0.0595769261704
        z: 1.30162277588
      rotation:
        x: -0.491584495607
        y: 0.509749637711
        z: -0.50857164943
        w: 0.489749801356
*/
