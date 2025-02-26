#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

class TfPublish {
public:
    TfPublish() {
        camera_coords_.x = 1.0;
        camera_coords_.y = 1.0;
        camera_coords_.z = 1.0;

        ros::NodeHandle nh;
        sub_ = nh.subscribe("point_topic", 1, &TfPublish::CoordinatePointCallback, this);
    }

    void tfPublish() {
        static tf2_ros::TransformBroadcaster tf_broadcaster;

        geometry_msgs::TransformStamped gt;
        gt.header.stamp = ros::Time::now();
        gt.header.frame_id = "head_rgbd_sensor_link"; // Origin
        gt.child_frame_id = "target_frame"; // Target
        gt.transform.translation.x = camera_coords_.x;
        gt.transform.translation.y = camera_coords_.y;
        gt.transform.translation.z = camera_coords_.z;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0); 
        gt.transform.rotation.x = q.x();
        gt.transform.rotation.y = q.y();
        gt.transform.rotation.z = q.z();
        gt.transform.rotation.w = q.w();

        tf_broadcaster.sendTransform(gt);
    }

    void CoordinatePointCallback(const geometry_msgs::Point::ConstPtr& coordinate_msg) {
        try {
            camera_coords_.x = coordinate_msg->x;
            camera_coords_.y = coordinate_msg->y;
            camera_coords_.z = coordinate_msg->z;


            tfPublish();
            ROS_INFO("tf_publish --> OK");
        }
        catch (std::exception& e) {
            ROS_ERROR_STREAM("Unable to create tf: " << e.what());
        }
    }

private:
    ros::Subscriber sub_;
    geometry_msgs::Point camera_coords_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publish_node");
    TfPublish tf_publish;

    ros::spin();

    return 0;
}