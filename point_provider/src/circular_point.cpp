#include "ros/ros.h"

#include "geometry_msgs/Point.h"

#include <memory>
#include <cmath>

#include "point_provider/PointProvider.h"
#include "point_provider/Utils.h"

class CircularPointStrategy : public PointProviderStrategy
{
public:
    CircularPointStrategy(float radius, float frequency)
    {
        _radius = radius;
        _frequency = frequency;
    }

    virtual geometry_msgs::Point GetPoint()
    {
        auto time = ros::Time::now();

        double t = time.toSec();

        geometry_msgs::Point point;
        point.x = _radius * sin(_frequency * t);
        point.y = _radius * cos(_frequency * t);
        point.z = 0;

        return point;
    }

private:
    float _radius;
    float _frequency;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_provider");

    auto radius = GetParamOrDefault<float>("~radius", 1.0f);
    auto frequency = GetParamOrDefault<float>("~frequency", 1.0f);

    auto pointStrategy = std::make_unique<CircularPointStrategy>(radius, frequency);

    PointProvider node(std::move(pointStrategy));
    node.Spin();

    return 0;
}

