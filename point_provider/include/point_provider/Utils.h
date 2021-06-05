#ifndef UTILS_H
#define UTILS_H

template <typename ParamType>
ParamType GetParam(const char *name)
{
	ParamType param;
	if (ros::param::get(name, param) == false)
	{
		std::stringstream ss;
		ss << "Parameter " << name << " not set";
		throw std::runtime_error(ss.str());
	}
	return param;
}

template <typename ParamType>
ParamType GetParamOrDefault(const char *name, ParamType defaultValue)
{
	ParamType param;
	if (ros::param::get(name, param) == false)
	{
		return defaultValue;
	}
	return param;
}

#endif