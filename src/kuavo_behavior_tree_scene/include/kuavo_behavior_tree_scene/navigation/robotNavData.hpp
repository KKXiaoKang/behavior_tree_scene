#ifndef ROBOT_NAV_DATA_HPP
#define ROBOT_NAV_DATA_HPP

#include <ros/ros.h>
#include <ros/package.h> 
#include <chrono> // 引入 chrono 库，用于时间操作

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

// We want to use this custom type
struct Position3D
{ 
  double x;
  double y; 
  double z;
};

struct Orientation4D
{ 
  double x;
  double y; 
  double z;
  double w;
};

namespace BT
{
    template <> inline Position3D convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Position3D output;
            output.x     = convertFromString<double>(parts[0]);
            output.y     = convertFromString<double>(parts[1]);
            output.z     = convertFromString<double>(parts[2]);
            return output;
        }
    }

    template <> inline Orientation4D convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 4)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Orientation4D output;
            output.x     = convertFromString<double>(parts[0]);
            output.y     = convertFromString<double>(parts[1]);
            output.z     = convertFromString<double>(parts[2]);
            output.w     = convertFromString<double>(parts[3]);
            return output;
        }
    }

} // end namespace BT

#endif // ROBOT_NAV_DATA_HPP