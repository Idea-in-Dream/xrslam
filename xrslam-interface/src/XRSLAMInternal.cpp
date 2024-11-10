#include "XRSLAMManager.h"
#include "xrslam/xrslam.h"

// 用于创建和初始化 XRSLAM 系统配置
int XRSLAMCreate(
    const char *slam_config_path,   // slam configuration file path
    const char *device_config_path, // device configuration file path
    const char *license_path, const char *product_name, void **config) {
    
    // 检查许可证
    // if (xrslam::XRSLAMManager::Instance().CheckLicense(license_path,
    //                                                    product_name) == 0)
    //     return 0;

    // 创建了一个指向xrslam::extra::YamlConfig对象的共享智能指针（std::shared_ptr），并通过构造函数初始化该对象
    // std::make_shared 是一种创建 shared_ptr 的工厂函数，它不仅分配内存，还在内存中创建指定类型的对象。这样可以避免手动调用 new，并且会更高效，因为它在一个内存块中同时分配了对象和引用计数。
    std::shared_ptr<xrslam::extra::YamlConfig> yaml_config =
        std::make_shared<xrslam::extra::YamlConfig>(slam_config_path,
                                                    device_config_path);
    // 初始化一个 XRSLAM 系统并启动相关的工作线程
    // 调用了 XRSLAMManager 类的 Instance() 方法，返回一个 XRSLAMManager 的实例。然后通过调用 Init 方法，传递一个 yaml_config 配置对象，来初始化该实例
    xrslam::XRSLAMManager::Instance().Init(yaml_config);
    *config = static_cast<void *>(yaml_config.get());
    return 1;
}

void XRSLAMPushSensorData(XRSLAMSensorType sensor_type, // sensor type
                          void *sensor_data             // sensor data
) {
    switch (sensor_type) {
    case XRSLAM_SENSOR_CAMERA:
        xrslam::XRSLAMManager::Instance().PushImage(
            static_cast<XRSLAMImage *>(sensor_data));
        break;
    case XRSLAM_SENSOR_ACCELERATION:
        xrslam::XRSLAMManager::Instance().PushAcceleration(
            static_cast<XRSLAMAcceleration *>(sensor_data));
        break;
    case XRSLAM_SENSOR_GYROSCOPE:
        xrslam::XRSLAMManager::Instance().PushGyroscope(
            static_cast<XRSLAMGyroscope *>(sensor_data));
        break;
    case XRSLAM_SENSOR_DEPTH_CAMERA:
    case XRSLAM_SENSOR_GRAVITY:
    case XRSLAM_SENSOR_ROTATION_VECTOR:
    case XRSLAM_SENSOR_UNKNOWN:
    default:
        break;
    }
}

void XRSLAMRunOneFrame() { xrslam::XRSLAMManager::Instance().RunOneFrame(); }

void XRSLAMGetResult(XRSLAMResultType result_type, // result type
                     void *result_data             // result data
) {
    switch (result_type) {
    case XRSLAM_RESULT_BODY_POSE:
        xrslam::XRSLAMManager::Instance().GetResultBodyPose(
            static_cast<XRSLAMPose *>(result_data));
        break;
    case XRSLAM_RESULT_CAMERA_POSE:
        xrslam::XRSLAMManager::Instance().GetResultCameraPose(
            static_cast<XRSLAMPose *>(result_data));
        break;
    case XRSLAM_RESULT_STATE:
        xrslam::XRSLAMManager::Instance().GetResultState(
            static_cast<XRSLAMState *>(result_data));
        break;
    case XRSLAM_RESULT_LANDMARKS:
        xrslam::XRSLAMManager::Instance().GetResultLandmarks(
            static_cast<XRSLAMLandmarks *>(result_data));
        break;
    case XRSLAM_RESULT_FEATURES:
        xrslam::XRSLAMManager::Instance().GetResultFeatures(
            static_cast<XRSLAMFeatures *>(result_data));
        break;
    case XRSLAM_RESULT_BIAS:
        xrslam::XRSLAMManager::Instance().GetResultBias(
            static_cast<XRSLAMIMUBias *>(result_data));
        break;
    case XRSLAM_RESULT_VERSION:
        xrslam::XRSLAMManager::Instance().GetResultVersion(
            static_cast<XRSLAMStringOutput *>(result_data));
        break;
    case XRSLAM_RESULT_DEBUG_LOGS:
    case XRSLAM_RESULT_UNKNOWN:
    default:
        break;
    }
}

void XRSLAMDestroy() { xrslam::XRSLAMManager::Instance().Destroy(); }
