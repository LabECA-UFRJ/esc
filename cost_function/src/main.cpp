#include "ros/ros.h"

#include "geometry_msgs/Point.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

#include "std_msgs/Float64.h"

#include "cost_function/Utils.h"

#include <cmath>

class CostFunction
{
public:
    CostFunction() : _nh(""), _tfListener(_tfBuffer)
    {
        _costPub = _nh.advertise<std_msgs::Float64>("cost", 10);
        _sourcePositionSub = _nh.subscribe("source_position", 1, &CostFunction::UpdateSourcePosition, this);

        SetupParameters();
    }

    void UpdateSourcePosition(const geometry_msgs::Point::ConstPtr &point)
    {
        _lastSourcePosition.x = point->x;
        _lastSourcePosition.y = point->y;
    }

    void Spin()
    {
        WaitTransformAvailable();

        ros::Rate rate(GetParam<float>("~rate"));
        while (_nh.ok())
        {
            auto transform = GetTransform();

            auto robot_x = transform.translation.x;
            auto robot_y = transform.translation.y;

            auto source_x = _lastSourcePosition.x;
            auto source_y = _lastSourcePosition.y;

            std_msgs::Float64 cost;
            cost.data = Evaluate(robot_x, robot_y, source_x, source_y);
            _costPub.publish(cost);

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    double Evaluate(double robot_x, double robot_y, double source_x, double source_y)
    {
        return -(pow(robot_x - source_x, 2) + pow(robot_y - source_y, 2)) + 50;
    }

    geometry_msgs::Transform GetTransform()
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = _tfBuffer.lookupTransform(ref_frame, child_frame, ros::Time(0));

            geometry_msgs::Transform transform;
            transform.translation = transformStamped.transform.translation;
            transform.rotation = transformStamped.transform.rotation;

            return transform;
        }
        catch (tf2::TransformException &e)
        {
            ROS_FATAL("%s", e.what());
            exit(-1);
        }
    }

    void WaitTransformAvailable()
    {
        while (!_tfBuffer.canTransform(child_frame, ref_frame, ros::Time(0), ros::Duration(1.0)))
        {
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    void SetupParameters()
    {
        try
        {
            ref_frame = GetParam<std::string>("~ref_frame");
            child_frame = GetParam<std::string>("~robot_frame");
        }
        catch (std::exception &e)
        {
            ROS_FATAL("%s", e.what());
            exit(-1);
        }
    }

    ros::NodeHandle _nh;

    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;

    std::string ref_frame, child_frame;

    ros::Publisher _costPub;
    ros::Subscriber _sourcePositionSub;

    geometry_msgs::Point _lastSourcePosition;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cost_function");

    CostFunction node;

    node.Spin();

    return 0;
}