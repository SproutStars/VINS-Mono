#pragma once

#include <sys/stat.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include <opencv2/core/persistence.hpp>

#include "camodocal/camera_models/YamlFile.h"

namespace camodocal
{

// Calibration data stored in a ROS sensor_msgs/CameraInfo style yaml file
// (camera_matrix / distortion_coefficients / image_width / image_height).
struct CameraInfoData
{
    CameraInfoData()
     : valid(false)
     , image_width(0)
     , image_height(0)
     , k1(0.0)
     , k2(0.0)
     , p1(0.0)
     , p2(0.0)
     , fx(0.0)
     , fy(0.0)
     , cx(0.0)
     , cy(0.0)
    {
    }

    bool valid;
    std::string camera_name;
    std::string distortion_model;
    int image_width;
    int image_height;
    // OpenCV plumb_bob distortion: k1, k2, p1, p2.
    double k1, k2, p1, p2;
    // Projection: fx, fy, cx, cy.
    double fx, fy, cx, cy;
};

inline bool fileExists(const std::string& path)
{
    struct stat buffer;
    return stat(path.c_str(), &buffer) == 0;
}

// Opens a YAML calibration file. ROS camera_info files usually start with a
// comment instead of the %YAML directive required by OpenCV, so the directive
// is prepended when it is missing and the content is parsed from memory.
inline bool openYamlFile(const std::string& filename, cv::FileStorage& fs)
{
    std::ifstream file(filename.c_str(), std::ios::in | std::ios::binary);
    if (!file.is_open())
        return false;

    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();

    // Drop a UTF-8 byte order mark if present.
    if (content.size() >= 3
        && static_cast<unsigned char>(content[0]) == 0xEF
        && static_cast<unsigned char>(content[1]) == 0xBB
        && static_cast<unsigned char>(content[2]) == 0xBF)
        content.erase(0, 3);

    if (content.compare(0, 5, "%YAML") != 0)
        content = "%YAML:1.0\n---\n" + content;

    try
    {
        return fs.open(content, cv::FileStorage::READ | cv::FileStorage::MEMORY);
    }
    catch (const cv::Exception&)
    {
        return false;
    }
}

// True when the node holds a ROS CameraInfo style calibration.
inline bool isCameraInfoNode(const cv::FileNode& settings)
{
    return !settings["camera_matrix"].empty();
}

// Reads a ROS sensor_msgs/CameraInfo style yaml file. Returns false when the
// file cannot be opened or does not contain a valid camera_matrix.
inline bool readCameraInfoFile(const std::string& filename, CameraInfoData& info)
{
    cv::FileStorage fs;
    if (!openYamlFile(filename, fs))
        return false;

    cv::FileNode settings = yamlParameters(fs);
    if (!isCameraInfoNode(settings))
        return false;

    std::vector<double> K, D;
    settings["camera_matrix"]["data"] >> K;
    if (!settings["distortion_coefficients"].empty())
        settings["distortion_coefficients"]["data"] >> D;
    if (K.size() < 9)
        return false;

    info.fx = K[0];
    info.fy = K[4];
    info.cx = K[2];
    info.cy = K[5];

    if (!settings["camera_name"].empty())
        settings["camera_name"] >> info.camera_name;
    if (!settings["distortion_model"].empty())
        settings["distortion_model"] >> info.distortion_model;
    if (!settings["image_width"].empty())
        info.image_width = static_cast<int>(settings["image_width"]);
    if (!settings["image_height"].empty())
        info.image_height = static_cast<int>(settings["image_height"]);

    // plumb_bob/rational_polynomial ordering: k1, k2, p1, p2, k3, ...
    if (D.size() > 0)
        info.k1 = D[0];
    if (D.size() > 1)
        info.k2 = D[1];
    if (D.size() > 2)
        info.p1 = D[2];
    if (D.size() > 3)
        info.p2 = D[3];

    if (D.size() > 4 && std::fabs(D[4]) > 1e-12)
        std::cerr << "# WARN: camera_info " << filename
                  << " contains k3=" << D[4]
                  << ", the pinhole model only uses k1/k2/p1/p2" << std::endl;

    if (!info.distortion_model.empty() &&
        info.distortion_model != "plumb_bob" &&
        info.distortion_model != "rational_polynomial" &&
        info.distortion_model != "radtan")
        std::cerr << "# WARN: camera_info " << filename << " uses distortion_model '"
                  << info.distortion_model
                  << "', only the pinhole model (k1/k2/p1/p2) is supported" << std::endl;

    info.valid = true;
    fs.release();
    return true;
}

// Resolves the camera_info file path. Relative paths are resolved against the
// directory of the given config file first, then against vins_folder.
inline std::string resolveCameraInfoPath(const std::string& config_file,
                                         const std::string& camera_info_file,
                                         const std::string& vins_folder = std::string())
{
    if (camera_info_file.empty())
        return std::string();

    if (camera_info_file[0] == '/')
        return camera_info_file;

    std::string config_dir;
    const std::string::size_type pos = config_file.find_last_of('/');
    if (pos != std::string::npos)
        config_dir = config_file.substr(0, pos + 1);

    if (!config_dir.empty())
    {
        const std::string candidate = config_dir + camera_info_file;
        if (fileExists(candidate))
            return candidate;
    }

    if (!vins_folder.empty())
    {
        std::string prefix = vins_folder;
        if (prefix.back() != '/')
            prefix += '/';
        const std::string candidate = prefix + camera_info_file;
        if (fileExists(candidate))
            return candidate;
    }

    if (fileExists(camera_info_file))
        return camera_info_file;

    return config_dir + camera_info_file;
}

}  // namespace camodocal
