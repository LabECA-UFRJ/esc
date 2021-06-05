#ifndef POINTPROVIDERSTRATEGY_H
#define POINTPROVIDERSTRATEGY_H

class PointProviderStrategy
{
public:
    virtual geometry_msgs::Point GetPoint() = 0;

    virtual ~PointProviderStrategy() {};
};

#endif