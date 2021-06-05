#include "ros/ros.h"

#include "geometry_msgs/Point.h"

#include <memory>
#include <cmath>

#include "point_provider/PointProvider.h"

class ConstantPointStrategy : public PointProviderStrategy
{
public:
    ConstantPointStrategy(geometry_msgs::Point point)
    {
        _point = point;
    }

    virtual geometry_msgs::Point GetPoint()
    {
        return _point;
    }

private:
    geometry_msgs::Point _point;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_provider");

    geometry_msgs::Point point;
    point.x = GetParamOrDefault<double>("~x", 0.0);
    point.y = GetParamOrDefault<double>("~y", 0.0);
    point.z = 0;

    auto pointStrategy = std::make_unique<ConstantPointStrategy>(point);

    PointProvider node(std::move(pointStrategy));
    node.Spin();

    return 0;
}
