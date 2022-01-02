#include <memory>
#include <string>
#include <cstring>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <eigen3/Eigen/Dense>

#include <sys/types.h>
#include <typeinfo>
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class LidarStitching : public rclcpp::Node
{
    public:
    
    
        LidarStitching()
            : Node("lidar_stitching")
        {
            
            leftSub.subscribe(this, "luminar_left_points",my_custom_qos_profile);
            rightSub.subscribe(this, "luminar_right_points",my_custom_qos_profile);
            frontSub.subscribe(this, "luminar_front_points",my_custom_qos_profile);

            
            generate_transform(left_to_ram,left_shift,2*M_PI/3);
            generate_transform(right_to_ram,right_shift,-2*M_PI/3);
            generate_transform(front_to_ram,front_shift,0);
            generate_transform(ram_to_cog,cog_shift,0);

            left_global = ram_to_cog * left_to_ram;
            right_global = ram_to_cog * right_to_ram;
            front_global = ram_to_cog * front_to_ram;

            publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_stitched", 10);
            sync_ = std::make_shared<Sync>(MySyncPolicy(3), leftSub, rightSub, frontSub);
            sync_->registerCallback(std::bind(&LidarStitching::cloud_callback, this, _1, _2, _3));
        }
    private:
        void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& leftMsg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& rightMsg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& frontMsg)
        {
            RCLCPP_INFO(this->get_logger(), "I heard: ");
            int left_size = leftMsg -> width;
            int right_size = rightMsg -> width;
            int front_size = frontMsg -> width;

            Eigen::MatrixXd LeftPoint(left_size,4);
            Eigen::MatrixXd RightPoint(right_size,4);
            Eigen::MatrixXd FrontPoint(front_size,4);
            Eigen::VectorXd Intensity(left_size + right_size + front_size);
            Eigen::VectorXd left = Eigen::VectorXd::Constant(left_size,1);
            Eigen::VectorXd right = Eigen::VectorXd::Constant(right_size,1);
            Eigen::VectorXd front = Eigen::VectorXd::Constant(front_size,1);
            Eigen::MatrixXd combined(left_size + right_size + front_size,4);

            msg_to_pointcloud(leftMsg,LeftPoint);
            msg_to_pointcloud(rightMsg,RightPoint);
            msg_to_pointcloud(frontMsg,FrontPoint);

            Intensity << LeftPoint.col(3), RightPoint.col(3),FrontPoint.col(3);
            
            LeftPoint.block(0,3,left_size,1) = left;
            RightPoint.block(0,3,right_size,1) = right;
            FrontPoint.block(0,3,front_size,1) = front;

            LeftPoint = LeftPoint * left_global.transpose();
            RightPoint = RightPoint * right_global.transpose();
            FrontPoint = FrontPoint * front_global.transpose();


            combined << LeftPoint, FrontPoint, RightPoint;

            publishXYZI(combined,frontMsg->header.stamp);
        }

        float unpackFloat(const std::vector<unsigned char>& buf, int i) {
            uint32_t temp = 0;
            temp = ((buf[i+0]) |
                    (buf[i+1] << 8) |
                    (buf[i+2] << 16) |
                    buf[i+3] << 24);
            return *((float *) &temp);
        }
        


        void generate_transform(Eigen::Matrix4d& tranform, float vector[],float angle){

            tranform << cos(angle),-sin(angle),0,vector[0],
                        sin(angle),cos(angle), 0,vector[1],
                        0,0,1,vector[2],
                        0,0,0,1;

                    
        }

        void msg_to_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& Msg,Eigen::MatrixXd& Point){

            int size = Msg -> width;
            float x,y,z,i = 0;
            int offset = 0;
            int pointstep = Msg -> point_step;
            
            for (int j = 0; j < size; j++){

              x = unpackFloat(Msg ->data,offset+0);
              y = unpackFloat(Msg ->data,offset+4);
              z = unpackFloat(Msg ->data,offset+8);
              i = unpackFloat(Msg ->data,offset+12);

              Point(j,0) = x;
              Point(j,1) = y;
              Point(j,2) = z;
              Point(j,3) = i;
              offset += pointstep;
            }
        }


        void publishXYZI(Eigen::MatrixXd& combined,auto stamp)
        {
            uint32_t point_step = 4;
            uint32_t data_size = combined.rows();
            sensor_msgs::msg::PointCloud2 msg;
            msg.header.frame_id = "base_link";
            msg.header.stamp = stamp;
            msg.fields.resize(4);
            msg.fields[0].name = "x";
            msg.fields[0].offset = 0;
            msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            msg.fields[0].count = 1;
            msg.fields[1].name = "y";
            msg.fields[1].offset = 4;
            msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            msg.fields[1].count = 1;
            msg.fields[2].name = "z";
            msg.fields[2].offset = 8;
            msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            msg.fields[2].count = 1;
            msg.fields[3].name = "intensity";
            msg.fields[3].offset = 12;
            msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
            msg.fields[3].count = 1;
            msg.data.resize(std::max((size_t)1, (size_t)data_size) * point_step);
            msg.point_step = point_step;
            msg.row_step = msg.data.size();
            msg.height = 1;
            msg.width = data_size;
            msg.is_bigendian = false;
            msg.is_dense = false;
            uint8_t *ptr = msg.data.data();
            
            for (size_t i = 0; i < data_size; i++)
            {
                *(reinterpret_cast<float*>(ptr + 0)) = combined(i,0);
                *(reinterpret_cast<float*>(ptr + 4)) = combined(i,1);
                *(reinterpret_cast<float*>(ptr + 8)) = combined(i,2);
                *(reinterpret_cast<float*>(ptr + 12)) = combined(i,3);
                ptr += point_step;
            }
            
            publisher_->publish(msg);
        }
        
        
        float left_shift[3] = {1.549, 0.267, 0.543};
        float right_shift[3] = {1.549, -0.267, 0.543};
        float front_shift[3] = {0,0,0};
        float cog_shift[3] = {-1.3206, 0.030188, -0.23598};
        Eigen::Matrix4d left_to_ram;
        Eigen::Matrix4d right_to_ram;
        Eigen::Matrix4d front_to_ram;
        Eigen::Matrix4d ram_to_cog;

        Eigen::Matrix4d left_global;
        Eigen::Matrix4d right_global;
        Eigen::Matrix4d front_global;
        
        

        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> leftSub;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> rightSub;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> frontSub;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        std::shared_ptr<Sync> sync_;
        
        const rmw_qos_profile_t my_custom_qos_profile =
        {   
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        5,
        RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false
        };
};

        

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarStitching>());
  rclcpp::shutdown();

  return 0;
}