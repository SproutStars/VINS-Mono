#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CameraInfoFile.h"
#include "camodocal/camera_models/YamlFile.h"

#include <iostream>

#include <boost/algorithm/string.hpp>


#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/ScaramuzzaCamera.h"

#include "ceres/ceres.h"

namespace camodocal
{

namespace
{

// Builds a pinhole camera from a ROS sensor_msgs/CameraInfo style calibration
// (camera_matrix + OpenCV distortion coefficients).
CameraPtr createCameraFromCameraInfo(const CameraInfoData& info)
{
    PinholeCameraPtr camera(new PinholeCamera);

    PinholeCamera::Parameters params = camera->getParameters();
    params.cameraName() = info.camera_name.empty() ? std::string("camera") : info.camera_name;
    params.imageWidth() = info.image_width;
    params.imageHeight() = info.image_height;
    params.k1() = info.k1;
    params.k2() = info.k2;
    params.p1() = info.p1;
    params.p2() = info.p2;
    params.fx() = info.fx;
    params.fy() = info.fy;
    params.cx() = info.cx;
    params.cy() = info.cy;
    camera->setParameters(params);

    return camera;
}

}  // namespace

boost::shared_ptr<CameraFactory> CameraFactory::m_instance;

CameraFactory::CameraFactory()
{

}

boost::shared_ptr<CameraFactory>
CameraFactory::instance(void)
{
    if (m_instance.get() == 0)
    {
        m_instance.reset(new CameraFactory);
    }

    return m_instance;
}

CameraPtr
CameraFactory::generateCamera(Camera::ModelType modelType,
                              const std::string& cameraName,
                              cv::Size imageSize) const
{
    switch (modelType)
    {
    case Camera::KANNALA_BRANDT:
    {
        EquidistantCameraPtr camera(new EquidistantCamera);

        EquidistantCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::PINHOLE:
    {
        PinholeCameraPtr camera(new PinholeCamera);

        PinholeCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::SCARAMUZZA:
    {
        OCAMCameraPtr camera(new OCAMCamera);

        OCAMCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::MEI:
    default:
    {
        CataCameraPtr camera(new CataCamera);

        CataCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    }
}

CameraPtr
CameraFactory::generateCameraFromYamlFile(const std::string& filename)
{
    // ROS sensor_msgs/CameraInfo style files are detected and converted to the
    // equivalent pinhole model, so callers can pass either file format.
    CameraInfoData cameraInfo;
    if (readCameraInfoFile(filename, cameraInfo))
        return createCameraFromCameraInfo(cameraInfo);

    cv::FileStorage fs;
    try
    {
        fs.open(filename, cv::FileStorage::READ);
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "# ERROR: cannot parse camera calibration file " << filename
                  << ": " << e.what() << std::endl;
        return CameraPtr();
    }

    if (!fs.isOpened())
    {
        return CameraPtr();
    }

    cv::FileNode settings = yamlParameters(fs);

    Camera::ModelType modelType = Camera::MEI;
    if (!settings["model_type"].isNone())
    {
        std::string sModelType;
        settings["model_type"] >> sModelType;

        if (boost::iequals(sModelType, "kannala_brandt"))
        {
            modelType = Camera::KANNALA_BRANDT;
        }
        else if (boost::iequals(sModelType, "mei"))
        {
            modelType = Camera::MEI;
        }
        else if (boost::iequals(sModelType, "scaramuzza"))
        {
            modelType = Camera::SCARAMUZZA;
        }
        else if (boost::iequals(sModelType, "pinhole"))
        {
            modelType = Camera::PINHOLE;
        }
        else
        {
            std::cerr << "# ERROR: Unknown camera model: " << sModelType << std::endl;
            return CameraPtr();
        }
    }

    switch (modelType)
    {
    case Camera::KANNALA_BRANDT:
    {
        EquidistantCameraPtr camera(new EquidistantCamera);

        EquidistantCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    case Camera::PINHOLE:
    {
        PinholeCameraPtr camera(new PinholeCamera);

        PinholeCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    case Camera::SCARAMUZZA:
    {
        OCAMCameraPtr camera(new OCAMCamera);

        OCAMCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    case Camera::MEI:
    default:
    {
        CataCameraPtr camera(new CataCamera);

        CataCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    }

    return CameraPtr();
}

}

