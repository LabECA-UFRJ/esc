#include "PointProviderStrategy.h"
#include "Utils.h"

#ifndef POINTPROVIDER_H
#define POINTPROVIDER_H

class PointProvider
{
public:
    PointProvider(std::unique_ptr<PointProviderStrategy> strategy) 
        : _nh(""), _strategy(std::move(strategy))
    {
	_pointPub = _nh.advertise<geometry_msgs::Point>("point", 10);
    }

    void Spin()
    {
        ros::Rate rate(GetParam<float>("~rate"));
        while (_nh.ok())
        {
            geometry_msgs::Point point = _strategy->GetPoint();

            _pointPub.publish(point);

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle _nh;

    ros::Publisher _pointPub;

    std::unique_ptr<PointProviderStrategy> _strategy;
};

#endif
