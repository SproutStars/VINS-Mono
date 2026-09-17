#include "parameters.h"
#include "camodocal/camera_models/CameraInfoFile.h"
#include "camodocal/camera_models/YamlFile.h"

#include <cmath>

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    cv::FileNode settings = camodocal::yamlParameters(fsSettings);

    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");
    if (!VINS_FOLDER_PATH.empty() && VINS_FOLDER_PATH.back() != '/')
        VINS_FOLDER_PATH += '/';

    // Optional: read the intrinsics directly from a ROS camera_info yaml file.
    // When the file is readable it takes precedence over the values below.
    std::string camera_info_file;
    if (!settings["camera_info_file"].empty())
        settings["camera_info_file"] >> camera_info_file;
    const std::string camera_info_path =
        camodocal::resolveCameraInfoPath(config_file, camera_info_file, VINS_FOLDER_PATH);
    camodocal::CameraInfoData camera_info;
    const bool use_camera_info =
        !camera_info_path.empty() && camodocal::readCameraInfoFile(camera_info_path, camera_info);
    if (use_camera_info)
        ROS_INFO_STREAM("Using camera intrinsics from " << camera_info_path);
    else if (!camera_info_path.empty())
        ROS_WARN_STREAM("Failed to read camera_info file " << camera_info_path
                        << ", using intrinsics from " << config_file);

    settings["image_topic"] >> IMAGE_TOPIC;
    settings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = settings["max_cnt"];
    MIN_DIST = settings["min_dist"];
    ROW = settings["image_height"];
    COL = settings["image_width"];
    if (use_camera_info)
    {
        if (camera_info.image_height > 0)
            ROW = camera_info.image_height;
        if (camera_info.image_width > 0)
            COL = camera_info.image_width;
    }
    FREQ = settings["freq"];
    F_THRESHOLD = settings["F_threshold"];
    SHOW_TRACK = settings["show_track"];
    EQUALIZE = settings["equalize"];
    FISHEYE = settings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(use_camera_info ? camera_info_path : config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    if (use_camera_info && camera_info.fx > 0.0 && camera_info.fy > 0.0)
    {
        FOCAL_LENGTH = static_cast<int>(std::sqrt(camera_info.fx * camera_info.fy) + 0.5);
    }
    else
    {
        const cv::FileNode projection = settings["projection_parameters"];
        if (!projection.empty())
        {
            const double fx = projection["fx"];
            const double fy = projection["fy"];
            if (fx > 0.0 && fy > 0.0)
                FOCAL_LENGTH = static_cast<int>(std::sqrt(fx * fy) + 0.5);
        }
    }
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();


}
