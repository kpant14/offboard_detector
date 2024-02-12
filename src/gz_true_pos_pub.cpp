/* Standard/Math library */
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <cmath>

/* Gazebo library */
#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <gz/msgs/pose_v.pb.h>

/* ROS2 library */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

/* Create a node that publishes Gazebo true position to the offboard detector */
class GZTruePosPublisher : public rclcpp::Node
{
  public:
    GZTruePosPublisher()
    : Node("gz_true_pos_sim_publisher")
    {
      if (!_node.Subscribe(_world_pose_topic, &GZTruePosPublisher::poseInfoCallback, this)) {
            RCLCPP_ERROR(this->get_logger(),"failed to subscribe to %s", _world_pose_topic.c_str());
      }

      /* Create a publisher with the specified name having a queue size of 10*/
      _gz_pose_ned_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(_gz_true_pose_ned_topic, 10);
      _gz_pose_enu_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(_gz_true_pose_enu_topic, 10);

      /* Adjust the callback rate as the same frequency as the camera network */
      _timer = this->create_wall_timer(100ms,std::bind(&GZTruePosPublisher::timer_callback, this));

      /* Initialize */
      _count = 0;
    }
  private:
    void timer_callback()
    {
        auto _pose_data_ned = geometry_msgs::msg::PoseStamped();
        _pose_data_ned.header.frame_id = "drone_true";
        _pose_data_ned.pose.position.x = _position_ned.X();
        _pose_data_ned.pose.position.y = _position_ned.Y();
        _pose_data_ned.pose.position.z = _position_ned.Z();
        _pose_data_ned.pose.orientation.x = _orientation_ned.X();
        _pose_data_ned.pose.orientation.y = _orientation_ned.Y();
        _pose_data_ned.pose.orientation.z = _orientation_ned.Z();
        _pose_data_ned.pose.orientation.w = _orientation_ned.W();
        _gz_pose_ned_publisher->publish(_pose_data_ned);

        auto _pose_data_enu = geometry_msgs::msg::PoseStamped();
        _pose_data_enu.header.frame_id = "drone_true";
        _pose_data_enu.pose.position.x = _position_enu.X();
        _pose_data_enu.pose.position.y = _position_enu.Y();
        _pose_data_enu.pose.position.z = _position_enu.Z();
        _pose_data_enu.pose.orientation.x = _orientation_enu.X();
        _pose_data_enu.pose.orientation.y = _orientation_enu.Y();
        _pose_data_enu.pose.orientation.z = _orientation_enu.Z();
        _pose_data_enu.pose.orientation.w = _orientation_enu.W();
        _gz_pose_enu_publisher->publish(_pose_data_enu);

        _count = 0;
    }

    void poseInfoCallback(const gz::msgs::Pose_V &_pose)
    {   
        _count += 1;

        for (int p = 0; p < _pose.pose_size(); p++) {
          if (_pose.pose(p).name() == _model_name) {

            /* Coordinate conversion (ENU -> NED) */
            /* Need to fix */
            /* PX4 local position estimate -> from initial point */
            /* Altitude difference -> find out where it is from */
            /* No attitude offset -0.27m */
            _position_ned.X(_pose.pose(p).position().y()+(generate_wgn()*0.04f));
            _position_ned.Y(_pose.pose(p).position().x()+(generate_wgn()*0.04f));
            _position_ned.Z(-_pose.pose(p).position().z()+(generate_wgn()*0.08f)-0.035);

            
            gz::math::Quaterniond _temp_orientation(_pose.pose(p).orientation().w(),
                              _pose.pose(p).orientation().x(),
                              _pose.pose(p).orientation().y(),
                              _pose.pose(p).orientation().z());

            _orientation_ned = _q_ENU_to_NED*_temp_orientation*_q_FLU_to_FRD.Inverse();

            /* Non-converted coordinate */
            _position_enu.X(_pose.pose(p).position().x()+(generate_wgn()*0.00f));
            _position_enu.Y(_pose.pose(p).position().y()+(generate_wgn()*0.00f));
            _position_enu.Z(_pose.pose(p).position().z()+(generate_wgn()*0.00f)+0.035);

            _orientation_enu = _temp_orientation;

          }
        }
    }

    float generate_wgn()
    {
      /* generate white Gaussian noise sample with unit std dev */
      /* using Box-Muller method */
      float temp=((float)(rand()+1))/(((float)RAND_MAX+1.0f));
      return sqrtf(-2.0f*logf(temp))*cosf(2.0f*M_PI*rand()/RAND_MAX);
    }

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _gz_pose_ned_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _gz_pose_enu_publisher;

    gz::transport::Node _node;
    gz::math::Vector3d _position_ned;
    gz::math::Quaterniond _orientation_ned;
    gz::math::Vector3d _position_enu;
    gz::math::Quaterniond _orientation_enu;

    // Body axis static rotation (FLU -> FRD, eul2quat([pi,0,0],'XYZ'))
	  static constexpr auto _q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);
    
	  // ENU to NED: +PI/2 rotation about Z followed by a +PI rotation about X 
    // (quatmultiply(eul2quat([0 0 pi/2],'XYZ'),eul2quat([pi 0 0],'XYZ')))
	  static constexpr auto _q_ENU_to_NED = gz::math::Quaterniond(0, 0.70711, 0.70711, 0);

    std::string _world_name = "AbuDhabi";
    std::string _model_name = "x500_1";
    std::string _world_pose_topic = "/world/" + _world_name + "/pose/info";
    std::string _gz_true_pose_ned_topic = "/px4_1/detector/in/gz_true_pose_ned";
    std::string _gz_true_pose_enu_topic = "/px4_1/detector/in/gz_true_pose_enu";

    int _count;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GZTruePosPublisher>());
  rclcpp::shutdown();
  return 0;
}
