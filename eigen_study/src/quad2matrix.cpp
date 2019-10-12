#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace std;

int main()
{

    cout << "Hello Eigen!!!" << endl;
    cout << "This is pr2 tf frame relation!!!" << endl;
    //ISO of base_footprint as Odometry
    Eigen::Isometry3d BFP_ISO = Eigen::Isometry3d::Identity();
    cout << "Odometry is same as Base_footprint:\n" << BFP_ISO.matrix() << endl;

    //ISO of imu to BFP
    Eigen::Isometry3d IMU_ISO = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond IMU_QUAT(0.0,1.0,0.0,-1.03411555577e-13);
    Eigen::Vector3d IMU_POS(-0.07977,-0.1497,0.966176901392);
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
    Eigen::Quaterniond LOC_QUAT(-0.491184959448,0.509262051439,-0.508957694678,0.490256630351);
    Eigen::Vector3d LOC_POS(0.0741019141012,0.0303910674988,1.30169003168);
    LOC_ISO.rotate(LOC_QUAT);
    LOC_ISO.pretranslate(LOC_POS);
    cout << "Left Optical Camera to Base_footprint:\n" << LOC_ISO.matrix() << endl;

    //ISO of right optical camera to BFP
    Eigen::Isometry3d ROC_ISO = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond ROC_QUAT(-0.491184959448,0.509262051439,-0.508957694678,0.490256630351);
    Eigen::Vector3d ROC_POS( 0.0707924751392,-0.0595480465056,1.3017482198);
    ROC_ISO.rotate(ROC_QUAT);
    ROC_ISO.pretranslate(ROC_POS);
    cout << "Right Optical Camera to Base_footprint:\n" << ROC_ISO.matrix() << endl;

    //VINS-Fusion need imu_T_stereo
    Eigen::Isometry3d imu_T_LOC = LOC_ISO.inverse()*IMU_ISO;
    cout << "IMU to Left Optical Camera:\n" << imu_T_LOC.matrix() << endl;
    Eigen::Isometry3d imu_T_ROC = ROC_ISO.inverse()*IMU_ISO;
    cout << "IMU to Right Optical Camera:\n" << imu_T_ROC.matrix() << endl;
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
}

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
