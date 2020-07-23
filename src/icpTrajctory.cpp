#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <math.h>
#include "std_msgs/String.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>
#include <gps_common/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <aloam_velodyne/PLICP_Trans.h>//include custom msg type

// #include <iostream>
#include <hector_icp/icpPointToPoint.h>

#include <iostream>
#include <fstream>
using namespace std;



class icp_iter_class{

public:


  icp_iter_class(){
    std::remove("Hector.txt");
    std::remove("Tracker.txt");
    std::remove("Icp.txt");
    std::remove("TrackerOrig.txt");

    n_.param("/aloam_velodyne/sampling_round", p_sampling_round, 5);
    n_.param("/aloam_velodyne/sampling_num", p_sampling_num, 200);
    n_.param("/aloam_velodyne/iter_cycle", p_iter_cycle,5);
    n_.param("/aloam_velodyne/gap_limit", p_gap_limit, 1.0f);
    n_.param("/aloam_velodyne/angle_limit", p_angle_limit, 0.2f);
    reference_location_init = 0;
    laser_count = 0;
    hactor_count = 0;
    laser_offset = 0;
    hactor_offset = 0;
    laser_stamp_min.fromSec(0.0);
    laser_stamp_max.fromSec(0.0);
    first_call_escaper = 0;
    orientation_aline_escaper = 0;
    gps_location_init = false;
    path_flag = 1;
    min_residual = 10.00;
    round_hector_points = 0;
    round_tracker_points = 0;

    sub_ = n_.subscribe("/trajectory", 50, &icp_iter_class::hector_path_Callback, this);
    tracker_sub = n_.subscribe("/Tracker_xyz", 50, &icp_iter_class::tacker_callback, this);
    gps_sub = n_.subscribe("/odometry/gps", 50, &icp_iter_class::gps_callback, this);
    pose_sub = n_.subscribe("/Ground_Truth", 50, &icp_iter_class::pose_callback, this);
    timer  = n_.createTimer(ros::Duration(p_iter_cycle), &icp_iter_class::icp_iter, this);
    icp_path_pub = n_.advertise<pcl::PointCloud<pcl::PointXYZ> >("icp_path", 1);
    align_path_pub = n_.advertise<pcl::PointCloud<pcl::PointXYZ> >("align_path", 1);
    hector_path_pub = n_.advertise<pcl::PointCloud<pcl::PointXYZ> >("Hector_path", 1);
    tracker_path_pub = n_.advertise<pcl::PointCloud<pcl::PointXYZ> >("Tracker_path", 1);
    PCL_Trans_pub = n_.advertise<aloam_velodyne::PLICP_Trans>("PCL_Trans", 1);
    gps_pub = n_.advertise<geometry_msgs::PoseStamped>("GPS_path", 1);
  }


  void icp_iter(const ros::TimerEvent&) {
    myfileH.open("Hector.txt", std::ios_base::app);
    myfileT.open("Tracker.txt", std::ios_base::app);
    myfileI.open("Icp.txt", std::ios_base::app);
    myfileJ.open("TrackerOrig.txt", std::ios_base::app);
    if (first_call_escaper < 2){
      ROS_INFO("Skip 5 sec %d", first_call_escaper); // with /clock some reason ROS will call timer event twice when init.
      first_call_escaper++;
      ROS_INFO("pre-sampleling in %d rounds, every round with %d points, Iter every %d Seconds. ITM tigger if gap bigger than %f meters.", p_sampling_round, p_sampling_num, p_iter_cycle, p_gap_limit);
      }
    else{
      float factor;
      int32_t dim = 2;
      // int32_t num = 100;
      int32_t i = 0;
      int32_t j = 0;
      double* M = (double*)calloc(3*hactor_count,sizeof(double));
      double* T = (double*)calloc(3*laser_count,sizeof(double));
      Matrix R = Matrix::eye(dim);
      Matrix t(dim,1);
      Matrix mass_center(dim,1);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Tracker (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Hector (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ICP (new pcl::PointCloud<pcl::PointXYZ>);

      if(laser_count >= path_flag*p_sampling_num && path_flag <= p_sampling_round){
        double residual = pclAssembler(cloud_Hector, cloud_Tracker, i, j, M, T, R, t, dim, mass_center);
          // if success and residual is bigger than something!? lucas
        if (residual < min_residual){
            min_residual = residual;
            ROS_INFO("###################### Alineing Reference Frame ###################### \n");
            path_rotation = R;
            path_translation = t;
            rotatePoint(T, j, R, t, dim);
            ITMEncoder(cloud_ICP,T, j, dim, cloud_Hector);
            align_path_pub.publish(cloud_ICP);
          }
          else{
            ROS_INFO("\n######Residual not valid, ICP has not converged.###### %f", residual);
          }
          // if(path_flag == p_sampling_round){//last prepare cycle && first piblish
          //     ITMpub(laser_stamp_min, laser_stamp_max, R, t);
          // }
          path_flag ++;
        }
      else if(laser_count >= path_flag*p_sampling_num && path_flag > p_sampling_round){
          double residual = pclAssembler(cloud_Hector, cloud_Tracker, i, j, M, T, R, t, dim, mass_center);
          // if success and residual is bigger than something!? lucas
          if (residual < 10){
            ROS_INFO("###################### Sending Rotation Matrix ###################### \n");
            ROS_INFO("Residual is: %f", residual);
            rotatePoint(M, i, R, t, dim);
            ITMEncoder(cloud_ICP, M, i, dim, cloud_Hector);

            // pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
            // feature_extractor.setInputCloud (cloud_Tracker);
            // feature_extractor.compute ();
            ROS_INFO("Rotation Mass Center: X %f, Y %f, Z %f", mass_center.val[0][0], mass_center.val[1][0]);
            // icp_path_pub.publish(cloud_ICP);
            // ROS_INFO("Rotation is %f", (atan2(R.val[1][0] , R.val[0][0])));
            if ((abs(t.val[0][0]) + abs(t.val[0][1])) > p_gap_limit && abs(atan2(R.val[1][0] , R.val[0][0])) > p_angle_limit){
              icp_path_pub.publish(cloud_ICP);
              ITMpub(laser_stamp_min, laser_stamp_max, R, t, mass_center);
            }else{
              align_path_pub.publish(cloud_ICP);
              ROS_INFO("Displance or Rotation is too Small, No need to Correct");
            }
          }
          else{
            ROS_INFO("######Residual not valid, ICP has not converged.######");
          }
          path_flag ++;
        }
      else{
          ROS_INFO("waiting for more Reference points");
        }
        free(M);// free memory
        free(T);// free memory
      }
      myfileH.close();
      myfileT.close();
      myfileI.close();
      myfileJ.close();
    }

  double pclAssembler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Hector, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Tracker, int &i, int &j, double *&M, double *&T, Matrix &R, Matrix &t, int dim, Matrix &mass_center){
    pointCloudResize(cloud_Hector);
    pointCloudResize(cloud_Tracker);
    round_hector_points = hactor_count-hactor_offset;
    round_tracker_points = laser_count-laser_offset;//downsamping to match number of points

    i = hectorEncoder(cloud_Hector, M, i, dim, mass_center);
    j = referenceEncoder(cloud_Tracker, T, j, dim);

    hector_path_pub.publish(cloud_Hector);
    tracker_path_pub.publish(cloud_Tracker);



    if (path_flag <= p_sampling_round){
      ROS_INFO("Trajectory Alignment");
      IcpPointToPoint icp(M,i,dim);
      icp.setMaxIterations(400);
      return icp.fit(T,j,R,t,-1);
    }
    else{
      ROS_INFO("Trajectory Matching");
      IcpPointToPoint icp(T,j,dim);
      ROS_INFO("ICT FITTED");
      icp.setMaxIterations(400);
      return icp.fit(M,i,R,t,-1);
    }
  }


  void pointCloudResize(pcl::PointCloud<pcl::PointXYZ>::Ptr targetPC){
    targetPC->width    = 50;
    targetPC->height   = 50;
    targetPC->points.resize (targetPC->width * targetPC->height);
    pcl_conversions::toPCL(ros::Time::now(), targetPC->header.stamp);
    targetPC->header.frame_id = "/map";
  }

  int32_t referenceEncoder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Tracker, double *&T, int32_t j, int32_t dim){
    float factor = round_tracker_points / round_hector_points;
    cloud_Tracker->header.frame_id = "/map";
    pcl_conversions::toPCL(ros::Time::now(), cloud_Tracker->header.stamp);
    laser_stamp_min = Laser_Path.poses[laser_offset].header.stamp;
    laser_stamp_max = Laser_Path.poses[laser_count-1].header.stamp;

    ROS_INFO("Processing Tracker points from %d to %d, processed [%f] points, from %f to %f.", laser_offset, laser_count, round_tracker_points, laser_stamp_min.toSec(), laser_stamp_max.toSec());

    while(laser_offset < laser_count){
      T[j*dim+0] = Laser_Path.poses[laser_offset].pose.position.x;
      T[j*dim+1] = Laser_Path.poses[laser_offset].pose.position.y;
      if (factor>1){
        laser_offset = ceil(factor + laser_offset);
      }else{
        laser_offset++;
      }
      myfileJ << Laser_Path.poses[laser_offset].header.stamp << "," <<  T[j*dim+0] << "," << T[j*dim+1] << std::endl;

      j++;
    }
    if (path_flag > p_sampling_round){
      rotatePoint(T, j, path_rotation, path_translation, dim);//orientation correction from init
    }
    int jc = 0;
    while(jc <= j){
      cloud_Tracker->points[jc].x = T[jc*dim+0];
      cloud_Tracker->points[jc].y = T[jc*dim+1];

      myfileT << Laser_Path.poses[laser_offset-j+jc].header.stamp << "," <<  cloud_Tracker->points[jc].x << "," << cloud_Tracker->points[jc].y << std::endl;
      jc++;
    }

    ROS_INFO("Tracker points downsamping to %d points.", j);
    laser_offset = laser_count;
    return j;
  }

  int32_t hectorEncoder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Hector, double *&M, int32_t i, int32_t dim, Matrix &mass_center){
    float factor = round_hector_points / round_tracker_points;
    float m_c_x = 0.0;
    float m_c_y = 0.0;
    pcl_conversions::toPCL(ros::Time::now(), cloud_Hector->header.stamp);
    cloud_Hector->header.frame_id = "/map";
    ROS_INFO("Processing hactor points from %d to %d, processed [%f] points", hactor_offset, hactor_count, round_hector_points);
    while(hactor_offset < Hactor_Path.poses.size() ){
      M[i*dim+0] = Hactor_Path.poses[hactor_offset].pose.position.x;
      M[i*dim+1] = Hactor_Path.poses[hactor_offset].pose.position.y;
      // M[i*dim+2] = Hactor_Path.poses[hactor_offset].pose.position.z;
      cloud_Hector->points[i].x = Hactor_Path.poses[hactor_offset].pose.position.x;
      cloud_Hector->points[i].y = Hactor_Path.poses[hactor_offset].pose.position.y;
      myfileH << Hactor_Path.poses[hactor_offset].header.stamp << ","  << cloud_Hector->points[i].x << "," <<cloud_Hector->points[i].y << std::endl;
      if (factor>1){
        hactor_offset = ceil(factor + hactor_offset);
      }else{
        hactor_offset++;
      }
      m_c_x += M[i*dim+0];
      m_c_y += M[i*dim+1];
      i++;
    }
    mass_center.val[0][0] = m_c_x/i;
    mass_center.val[0][1] = m_c_y/i;
    ROS_INFO("Hector points downsamping to %d points.", i);
    return i;
  }

  void ITMEncoder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ICP, double *&M, int32_t i, int32_t dim, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Hector){
    pointCloudResize(cloud_ICP);
    pcl_conversions::toPCL(ros::Time::now(), cloud_ICP->header.stamp);
    cloud_ICP->header.frame_id = "map";
    for (int c = 0; c < i; ++c) {
      // ROS_INFO("Writing %d",c);
      cloud_ICP->points[c].x = M[dim * c + 0];
      cloud_ICP->points[c].y = M[dim * c + 1];
      myfileI << Hactor_Path.poses[hactor_offset-i+c].header.stamp << "," <<cloud_ICP->points[c].x << "," << cloud_ICP->points[c].y << std::endl;
    }
    // icp_path_pub.publish(cloud_ICP);
  }

  void ITMpub(ros::Time laser_stamp_min, ros::Time laser_stamp_max, Matrix Rr, Matrix t, Matrix mass_center){
    t.getData(val_tm);
    // Matrix Rr = Matrix::inv(R);
    Rr.getData(val_Rm);
    aloam_velodyne::PLICP_Trans new_frame_trans;
    new_frame_trans.header.stamp = ros::Time::now();
    new_frame_trans.header.frame_id = "map";
    new_frame_trans.child_frame_id = "pcl_transed";
    new_frame_trans.transform.data.resize(4);
    new_frame_trans.transform.data[0]= val_tm[0];
    new_frame_trans.transform.data[1]= val_tm[1];
    new_frame_trans.transform.data[2]= mass_center.val[0][0];
    new_frame_trans.transform.data[3]= mass_center.val[0][1];
    new_frame_trans.rotation.data.resize(4);
    new_frame_trans.rotation.data[0]= val_Rm[0];
    new_frame_trans.rotation.data[1]= val_Rm[1];
    new_frame_trans.rotation.data[2]= val_Rm[2];
    new_frame_trans.rotation.data[3]= val_Rm[3];
    new_frame_trans.begin_stamp = laser_stamp_min;
    new_frame_trans.end_stamp = laser_stamp_max;
    PCL_Trans_pub.publish(new_frame_trans);//msg TF transform
    ROS_INFO("Current Trans Time: %f to %f\n", laser_stamp_min.toSec(), laser_stamp_max.toSec());

  }

  void rotatePoint(double *&T, int &num2, Matrix R, Matrix t, int32_t dim) {
  	for (int i = 0; i < num2; ++i) {
  		FLOAT *val = new FLOAT[dim];
  		val[0] = (FLOAT)(T[dim * i + 0]);
  		val[1] = (FLOAT)(T[dim * i + 1]);
  		Matrix point(dim, 1, val);
  		Matrix pointout = R * point + t;
  		T[dim * i + 0] = pointout.val[0][0];
  		T[dim * i + 1] = pointout.val[1][0];
  	}
  }

  void hector_path_Callback(const nav_msgs::Path& msg)
  {
    Hactor_Path = msg;
    hactor_count = Hactor_Path.poses.size();
    // ROS_INFO("Saved [%d] Hactor points", hactor_count);
  }

  void tacker_callback( const geometry_msgs::Vector3Stamped& laser_msg){
    if (reference_location_init = 0){
        reference_location_init_x = laser_msg.vector.x;
        reference_location_init_y = laser_msg.vector.y;
        reference_location_init_z = laser_msg.vector.z;
        reference_location_init ++;
        windowsXP_offset = ros::Time::now() - laser_msg.header.stamp;
      }
    else{
      //callback every time the leica's xyz is received
      ros::Time current_time = ros::Time::now();
      geometry_msgs::PoseStamped current_point;
      current_point.header.stamp = laser_msg.header.stamp + windowsXP_offset;
      current_point.header.frame_id = "map";
      current_point.pose.position.x = (laser_msg.vector.x - reference_location_init_x);
      current_point.pose.position.y = (laser_msg.vector.y - reference_location_init_y);
      current_point.pose.position.z = (laser_msg.vector.z - reference_location_init_z);//only 2D pose for now
      // ROS_INFO("Time header [%f]", current_point.header.stamp.toSec());
      Laser_Path.header.stamp = current_point.header.stamp;
      Laser_Path.header.frame_id = "map";
      Laser_Path.poses.push_back(current_point);

      laser_count++;
    }
    // ROS_INFO("Saved [%d] Tracker points", laser_count);
  }

  void gps_callback(const nav_msgs::Odometry& gps) {
    if (reference_location_init < 5){
        ROS_INFO("GPS Initiating [%d]", reference_location_init);
        reference_location_init_x += gps.pose.pose.position.x;
        reference_location_init_y += gps.pose.pose.position.y;
        reference_location_init_z += gps.pose.pose.position.z;
        reference_location_init ++;
      }
    else if(reference_location_init == 5){
      ROS_INFO("GPS Initiating Finished[%d]", reference_location_init);
      reference_location_init_x += gps.pose.pose.position.x;
      reference_location_init_y += gps.pose.pose.position.y;
      reference_location_init_z += gps.pose.pose.position.z;

      reference_location_init_x = reference_location_init_x/6;
      reference_location_init_y = reference_location_init_y/6;
      reference_location_init_z = reference_location_init_z/6;
      reference_location_init ++;
    }
    else {
      geometry_msgs::PoseStamped current_point;
      current_point.header.stamp = gps.header.stamp;
      current_point.header.frame_id = "map";
      // ROS_INFO("GPS TO MAP Coords: %f & %f\n", slu.pose.pose.position.x, Hactor_Path.poses[Hactor_Path.poses.size()].pose.position.y);
      current_point.pose.position.x = (gps.pose.pose.position.x - reference_location_init_x);
      current_point.pose.position.y = (gps.pose.pose.position.y - reference_location_init_y);
      current_point.pose.position.z = gps.pose.pose.position.z - reference_location_init_z;
      // ROS_INFO_STREAM("Map coordinate is (" << std::fixed << current_point.pose.position.x << ", " << current_point.pose.position.y << ")");
      // ROS_INFO("GPS HIT");
      // gps_pub.publish(current_point);
      // ROS_INFO("Time header [%f]", current_point.header.stamp.toSec());
      // ROS_INFO("GPS TO MAP Coords: %f & %f\n", current_point.pose.position.x, current_point.pose.position.y);
      Laser_Path.header.stamp = gps.header.stamp;
      Laser_Path.header.frame_id = "map";
      if (laser_count==0 ){
        Laser_Path.poses.push_back(current_point);
        laser_count++;
      }else if(Laser_Path.poses[laser_count-1].pose.position.x != current_point.pose.position.x && (current_point.pose.position.x+current_point.pose.position.y) != 0){ //filter losing GPS situation
        Laser_Path.poses.push_back(current_point);
        laser_count++;
      }else{
        ROS_INFO("GPS Lost");
      }
    }
      // geometry_msgs::PoseStamped current_point;
      // current_point.header.stamp = gps.header.stamp;
      // current_point.header.frame_id = "map";
      // // ROS_INFO("GPS TO MAP Coords: %f & %f\n", slu.pose.pose.position.x, Hactor_Path.poses[Hactor_Path.poses.size()].pose.position.y);
      // current_point.pose.position.x = gps.pose.pose.position.x;
      // current_point.pose.position.y = gps.pose.pose.position.y;
      // current_point.pose.position.z = gps.pose.pose.position.z;
      // gps_pub.publish(current_point);
      // // ROS_INFO("Time header [%f]", current_point.header.stamp.toSec());
      // // ROS_INFO("GPS TO MAP Coords: %f & %f\n", current_point.pose.position.x, current_point.pose.position.y);
      // Laser_Path.header.stamp = gps.header.stamp;
      // Laser_Path.header.frame_id = "map";
      // Laser_Path.poses.push_back(current_point);
      // ROS_INFO("GPS HIT");
      // laser_count++;
  }

  void pose_callback(const geometry_msgs::PoseStamped& pose) {
    // Current_Pose = pose;
    geometry_msgs::PoseStamped current_point;
    current_point.header.stamp = pose.header.stamp;
    current_point.header.frame_id = "map";
    // ROS_INFO("GPS TO MAP Coords: %f & %f\n", slu.pose.pose.position.x, Hactor_Path.poses[Hactor_Path.poses.size()].pose.position.y);
    current_point.pose.position.x = pose.pose.position.x;
    current_point.pose.position.y = pose.pose.position.y;
    current_point.pose.position.z = pose.pose.position.z;
    // slu_pub.publish(current_point);
    // ROS_INFO("Time header [%f]", current_point.header.stamp.toSec());
    // ROS_INFO("GPS TO MAP Coords: %f & %f\n", current_point.pose.position.x, current_point.pose.position.y);
    Laser_Path.header.stamp = pose.header.stamp;
    Laser_Path.header.frame_id = "map";
    Laser_Path.poses.push_back(current_point);

    laser_count++;
  }

  // void cloud_resize(float round_tracker_points,float round_hector_points){
  //   factor = round_tracker_points / round_hector_points;
  // }

// private:
  ros::NodeHandle n_;
  ros::Subscriber sub_, kitti_gps_sub;
  ros::Subscriber tracker_sub;
  ros::Subscriber gps_sub;
  ros::Subscriber pose_sub;
  ros::Timer timer;
  ros::Publisher icp_path_pub;
  ros::Publisher align_path_pub;
  ros::Publisher hector_path_pub;
  ros::Publisher tracker_path_pub;
  ros::Publisher PCL_Trans_pub;
  ros::Publisher gps_pub;
  nav_msgs::Path Hactor_Path;
  nav_msgs::Path Laser_Path;
  // geometry_msgs::PoseStamped Current_Pose;
  tf2_ros::TransformBroadcaster pcl_br;
  int reference_location_init;
  float reference_location_init_x;
  float reference_location_init_y;
  float reference_location_init_z;
  bool gps_location_init;
  float gps_location_init_x;
  float gps_location_init_y;
  float gps_location_init_z;
  int laser_count;
  int hactor_count;
  int laser_offset;
  int hactor_offset;
  ros::Time laser_stamp_min;
  ros::Time laser_stamp_max;
  int first_call_escaper;
  int orientation_aline_escaper;
  ros::Duration windowsXP_offset;
  int path_flag;
  double min_residual;
  Matrix path_rotation;
  Matrix path_translation;
  int p_sampling_round;
  int p_sampling_num;
  int p_iter_cycle;
  float p_gap_limit;
  float p_angle_limit;
  float  round_hector_points;
  float  round_tracker_points;
  float val_tm[2*2];  //two matrix for sending transform msg
  float val_Rm[2*2];
  ofstream myfileH;
  ofstream myfileT;
  ofstream myfileI;
  ofstream myfileJ;
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "path_icp");
  icp_iter_class my_pcl;
  ros::spin();

  return 0;
}
