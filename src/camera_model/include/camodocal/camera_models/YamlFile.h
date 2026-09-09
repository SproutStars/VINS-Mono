#pragma once

#include <opencv2/core/persistence.hpp>

namespace camodocal
{

// ROS 2 parameter files wrap the actual parameters in
// /**/ros__parameters.  The camera model is also used with the original
// OpenCV-style calibration files, so keep both layouts supported.
inline cv::FileNode yamlParameters(const cv::FileStorage &fs)
{
    cv::FileNode node = fs["/**"];
    if (!node.empty())
    {
        cv::FileNode parameters = node["ros__parameters"];
        if (!parameters.empty())
            return parameters;
    }
    return fs.root();
}

}  // namespace camodocal
