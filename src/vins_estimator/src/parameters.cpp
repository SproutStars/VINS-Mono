#include "parameters.h"
#include "camodocal/camera_models/YamlFile.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;

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

    settings["imu_topic"] >> IMU_TOPIC;

    SOLVER_TIME = settings["max_solver_time"];
    NUM_ITERATIONS = settings["max_num_iterations"];
    MIN_PARALLAX = settings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    settings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;

    // create folder if not exists
    FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());

    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ACC_N = settings["acc_n"];
    ACC_W = settings["acc_w"];
    GYR_N = settings["gyr_n"];
    GYR_W = settings["gyr_w"];
    G.z() = settings["g_norm"];
    ROW = settings["image_height"];
    COL = settings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = settings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        const cv::FileNode rotation_node = settings["extrinsicRotation"];
        const cv::FileNode translation_node = settings["extrinsicTranslation"];
        if (rotation_node.isSeq() && translation_node.isSeq())
        {
            std::vector<double> rotation_values;
            std::vector<double> translation_values;
            rotation_node >> rotation_values;
            translation_node >> translation_values;
            if (rotation_values.size() != 9 || translation_values.size() != 3)
            {
                ROS_ERROR("extrinsicRotation must contain 9 values and extrinsicTranslation must contain 3 values");
                n.shutdown();
                return;
            }
            cv_R = cv::Mat(3, 3, CV_64F, rotation_values.data()).clone();
            cv_T = cv::Mat(3, 1, CV_64F, translation_values.data()).clone();
        }
        else
        {
            // Backward compatibility with the original !!opencv-matrix layout.
            rotation_node >> cv_R;
            translation_node >> cv_T;
        }

        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = settings["td"];
    ESTIMATE_TD = settings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = settings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = settings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    fsSettings.release();
}
