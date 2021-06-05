#include "ros/ros.h"

#include "geometry_msgs/Point.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

#include "std_msgs/Float64.h"

#include "controller_msgs/Circle.h"

#include "esc_controller/HighPassFilter.h"
#include "esc_controller/LowPassFilter.h"

#include "esc_controller/Integrator.h"
#include "esc_controller/Utils.h"

#include <cmath>

class Controller
{
public:
    Controller() : _nh(""), _tfListener(_tfBuffer)
    {
        _circleReferencePub = _nh.advertise<controller_msgs::Circle>("reference", 10);
        _costSub = _nh.subscribe("cost", 1, &Controller::UpdateCost, this);

        SetupParameters();
    }

    void Spin()
    {
        WaitTransformAvailable();

        ros::Rate rate(GetParam<double>("~rate"));

        auto radius = GetParam<double>("~radius");
        auto gain = GetParam<double>("~gain");

        auto windupLimit = GetParam<double>("~windup_limit");
        Integrator integratorPositionX(windupLimit);
        Integrator integratorPositionY(windupLimit);

        auto omegaHigh = GetParam<double>("~omega_high");
        auto omegaLow = GetParam<double>("~omega_low");

        auto samplingPeriod = 1.0 / GetParam<double>("~rate");

        auto lowFilter1 = LowPassFilter(omegaLow, samplingPeriod);
        auto lowFilter2 = LowPassFilter(omegaLow, samplingPeriod);

        auto highFilter1 = HighPassFilter(omegaHigh, samplingPeriod);
        auto highFilter2 = HighPassFilter(omegaHigh, samplingPeriod);
        auto highFilter3 = HighPassFilter(omegaHigh, samplingPeriod);

        while (_nh.ok())
        {
            auto transform = GetTransform().translation;
            auto robot_x = transform.x;
            auto robot_y = transform.y;

            double filteredRobotX;
            double filteredRobotY;
            double filteredCost;

            highFilter1.update(robot_x, filteredRobotX);
            highFilter2.update(robot_y, filteredRobotY);
            highFilter3.update(_cost, filteredCost);

            double referenceVelocityX;
            double referenceVelocityY;
            lowFilter1.update(filteredRobotX * filteredCost, referenceVelocityX);
            lowFilter2.update(filteredRobotY * filteredCost, referenceVelocityY);

            integratorPositionX.Update(samplingPeriod, referenceVelocityX);
            integratorPositionY.Update(samplingPeriod, referenceVelocityY);

            controller_msgs::Circle circle;
            circle.x = integratorPositionX.Value() * gain;
            circle.y = integratorPositionY.Value() * gain;
            circle.r = radius;

            _circleReferencePub.publish(circle);

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
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
		try {
			ref_frame = GetParam<std::string>("~ref_frame");
			child_frame = GetParam<std::string>("~robot_frame");

		} catch (std::exception &e) {
			ROS_FATAL("%s", e.what());
			exit(-1);
		}
	}

    void UpdateCost(const std_msgs::Float64::ConstPtr& cost)
    {
        _cost = cost->data;
    }

    ros::NodeHandle _nh;

	tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;

	std::string ref_frame, child_frame;

    ros::Publisher _circleReferencePub;
    ros::Subscriber _costSub;

    double _cost;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "esc_controller");

    Controller node;

    node.Spin();

    return 0;
}