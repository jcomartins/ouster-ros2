/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_point.h
 * @brief PCL point datatype for use with ouster sensors
 */

#pragma once

#include <pcl/point_types.h>

namespace ouster_ros {

// The default/original representation of the point cloud since the driver
// inception. This shouldn't be confused with Point_LEGACY which provides exact
// mapping of the fields of Ouster LidarScan of the Legacy Profile, copying
// the same order and using the same bit representation. For example, this Point
// struct uses float data type to represent intensity (aka signal); however, the
// sensor sends the signal channel either as UINT16 or UINT32 depending on the
// active udp lidar profile.
struct EIGEN_ALIGN8 _Point {
    float x;
    float y;
    float z;
    uint8_t intensity;        // equivalent to signal
    uint8_t return_type;
    uint16_t ring;          // equivalent to channel
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Point : public _Point {

    inline Point(const _Point& pt)
    {
      x = pt.x; y = pt.y; z = pt.z;
      intensity = pt.intensity;
      ring = pt.ring;
    }

    inline Point()
    {
      x = y = z = 0.0f; 
      intensity = 0.0f;
      ring = 0;
    }

    inline const auto as_tuple() const {
        return std::tie(x, y, z, intensity, ring);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, intensity, ring);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

}   // namespace ouster_ros

// clang-format off

// DEFAULT/ORIGINAL
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint8_t, intensity, intensity)
    (std::uint8_t, return_type, return_type)
    (std::uint16_t, ring, channel)
)
