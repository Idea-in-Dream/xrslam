#ifndef XRSLAM_EXTRA_YAML_CONFIG_H
#define XRSLAM_EXTRA_YAML_CONFIG_H

#include <stdexcept>
#include <xrslam/xrslam.h>

namespace xrslam::extra {

class YamlConfig : public Config {
  public:
    // 
// 定义一个异常类，继承自std::runtime_error
    struct Exception : public std::runtime_error {
        // 构造函数，传入一个字符串参数，调用父类的构造函数
        Exception(const std::string &what) : std::runtime_error(what) {}
    };
    // 定义一个加载异常类，继承自Exception
    struct LoadException : public Exception {
        // 构造函数，传入一个字符串参数，调用父类的构造函数，并传入一个字符串，表示加载配置失败
        LoadException(const std::string &filename)
            : Exception("cannot load config " + filename) {}
    };
    // 定义一个解析异常类，继承自Exception
    struct ParseException : public Exception {
        // 构造函数，传入一个字符串参数，调用父类的构造函数，并传入一个字符串，表示解析失败
        ParseException(const std::string &message) : Exception(message) {}
    };
    // 定义一个配置缺失异常类，继承自Exception
    struct ConfigMissingException : public Exception {
        // 构造函数，传入一个字符串参数，调用父类的构造函数，并传入一个字符串，表示配置缺失
        ConfigMissingException(const std::string &config_path)
            : Exception("config \"" + config_path + "\" is mandatory") {}
    };
    // 定义一个类型错误异常类，继承自Exception
    struct TypeErrorException : public Exception {
        // 构造函数，传入一个字符串参数，调用父类的构造函数，并传入一个字符串，表示配置类型错误
        TypeErrorException(const std::string &config_path)
            : Exception("config \"" + config_path + "\" has wrong type") {}
    };

    // 构造函数: 该函数通过加载配置文件来初始化对象，为成员变量赋值，通常是将配置文件中的数据解析到类的成员中
    YamlConfig(const std::string &slam_config_filename,
               const std::string &device_config_filename);
    // 析构函数: 用于在对象销毁时清理任何可能的资源，如关闭文件流或释放动态分配的内存
    ~YamlConfig();

// 获取相机分辨率
    vector<2> camera_resolution() const override;
// 获取相机内参
    matrix<3> camera_intrinsic() const override;
// 获取相机畸变参数
    vector<4> camera_distortion() const override;
// 获取相机畸变标志
    size_t camera_distortion_flag() const override;
// 获取相机时间偏移
    double camera_time_offset() const override;
// 获取相机到机身旋转四元数
    quaternion camera_to_body_rotation() const override;
// 获取相机到机身平移向量
    vector<3> camera_to_body_translation() const override;
// 获取IMU到机身旋转四元数
    quaternion imu_to_body_rotation() const override;
// 获取IMU到机身平移向量
    vector<3> imu_to_body_translation() const override;

// 获取特征点噪声协方差矩阵
    matrix<2> keypoint_noise_cov() const override;
// 获取陀螺仪噪声协方差矩阵
    matrix<3> gyroscope_noise_cov() const override;
// 获取加速度计噪声协方差矩阵
    matrix<3> accelerometer_noise_cov() const override;
// 获取陀螺仪偏置噪声协方差矩阵
    matrix<3> gyroscope_bias_noise_cov() const override;
// 获取加速度计偏置噪声协方差矩阵
    matrix<3> accelerometer_bias_noise_cov() const override;

// 获取输出到机身旋转四元数
    quaternion output_to_body_rotation() const override;
// 获取输出到机身平移向量
    vector<3> output_to_body_translation() const override;

// 获取滑动窗口大小
    size_t sliding_window_size() const override;
// 获取滑动窗口子帧大小
    size_t sliding_window_subframe_size() const override;
// 获取滑动窗口追踪频率
    size_t sliding_window_tracker_frequent() const override;
// 获取滑动窗口强制关键帧特征点数量
    size_t sliding_window_force_keyframe_landmarks() const override;

// 获取特征点追踪器最小关键点距离
    double feature_tracker_min_keypoint_distance() const override;
// 获取特征点追踪器最大关键点检测数量
    size_t feature_tracker_max_keypoint_detection() const override;
// 获取特征点追踪器最大初始化帧数
    size_t feature_tracker_max_init_frames() const override;
// 获取特征点追踪器最大帧数
    size_t feature_tracker_max_frames() const override;
// 获取特征点追踪器CLAHE裁剪限制
    double feature_tracker_clahe_clip_limit() const override;
// 获取特征点追踪器CLAHE宽度
    size_t feature_tracker_clahe_width() const override;
// 获取特征点追踪器CLAHE高度
    size_t feature_tracker_clahe_height() const override;
// 获取特征点追踪器是否预测关键点
    bool feature_tracker_predict_keypoints() const override;

// 获取初始化器关键帧数量
    size_t initializer_keyframe_num() const override;
// 获取初始化器关键帧间隔
    size_t initializer_keyframe_gap() const override;
// 获取初始化器最小匹配数量
    size_t initializer_min_matches() const override;
// 获取初始化器最小视差
    double initializer_min_parallax() const override;
// 获取初始化器最小三角化数量
    size_t initializer_min_triangulation() const override;
// 获取初始化器最小特征点数量
    size_t initializer_min_landmarks() const override;
// 获取初始化器是否优化IMU
    bool initializer_refine_imu() const override;

// 获取视觉定位是否启用
    bool visual_localization_enable() const override;
// 获取视觉定位配置IP
    std::string visual_localization_config_ip() const override;
// 获取视觉定位配置端口
    size_t visual_localization_config_port() const override;

// 获取求解器迭代次数限制
    size_t solver_iteration_limit() const override;
// 获取求解器时间限制
    double solver_time_limit() const override;

// 获取Parsac标志
    bool parsac_flag() const override;
// 获取Parsac动态概率
    double parsac_dynamic_probability() const override;
// 获取Parsac阈值
    double parsac_threshold() const override;
// 获取Parsac归一化尺度
    double parsac_norm_scale() const override;
// 获取Parsac关键帧检查大小
    size_t parsac_keyframe_check_size() const override;

// 获取旋转对齐阈值
    double rotation_misalignment_threshold() const override;
// 获取旋转RANSAC阈值
    double rotation_ransac_threshold() const override;

  private:
    // 相机分辨率
    vector<2> m_camera_resolution;
    // 相机内参
    matrix<3> m_camera_intrinsic;
    // 相机畸变参数
    vector<4> m_camera_distortion;
    // 相机时间偏移
    size_t m_camera_distortion_flag;
    double m_camera_time_offset;
    // 相机到机体的旋转
    quaternion m_camera_to_body_rotation;
    // 相机到机体的平移
    vector<3> m_camera_to_body_translation;
    // IMU到机体的旋转
    quaternion m_imu_to_body_rotation;
    // IMU到机体的平移
    vector<3> m_imu_to_body_translation;
    // 特征点噪声协方差
    matrix<2> m_keypoint_noise_cov;
    // 陀螺仪噪声协方差
    matrix<3> m_gyroscope_noise_cov;
    // 加速度计噪声协方差
    matrix<3> m_accelerometer_noise_cov;
    // 陀螺仪偏置噪声协方差
    matrix<3> m_gyroscope_bias_noise_cov;
    // 加速度计偏置噪声协方差
    matrix<3> m_accelerometer_bias_noise_cov;

    // 输出到机体的旋转
    quaternion m_output_to_body_rotation;
    // 输出到机体的平移
    vector<3> m_output_to_body_translation;

    // 滑动窗口大小
    size_t m_sliding_window_size;
    // 滑动窗口子帧大小
    size_t m_sliding_window_subframe_size;
    // 滑动窗口追踪频率
    size_t m_sliding_window_tracker_frequent;
    // 滑动窗口强制关键帧特征点
    size_t m_sliding_window_force_keyframe_landmarks;

    // 特征点追踪器最小关键点距离
    double m_feature_tracker_min_keypoint_distance;
    // 特征点追踪器最大关键点检测
    size_t m_feature_tracker_max_keypoint_detection;
    // 特征点追踪器最大初始化帧
    size_t m_feature_tracker_max_init_frames;
    // 特征点追踪器最大帧
    size_t m_feature_tracker_max_frames;
    // 特征点追踪器CLAHE裁剪限制
    double m_feature_tracker_clahe_clip_limit;
    // 特征点追踪器CLAHE宽度
    size_t m_feature_tracker_clahe_width;
    // 特征点追踪器CLAHE高度
    size_t m_feature_tracker_clahe_height;
    // 特征点追踪器预测关键点
    bool m_feature_tracker_predict_keypoints;

    // 初始化器关键帧数量
    size_t m_initializer_keyframe_num;
    // 初始化器关键帧间隔
    size_t m_initializer_keyframe_gap;
    // 初始化器最小匹配
    size_t m_initializer_min_matches;
    // 初始化器最小视差
    double m_initializer_min_parallax;
    // 初始化器最小三角化
    size_t m_initializer_min_triangulation;
    // 初始化器最小特征点
    size_t m_initializer_min_landmarks;
    // 初始化器优化IMU
    bool m_initializer_refine_imu;

    // 可视化定位使能
    bool m_visual_localization_enable;
    // 可视化定位IP
    std::string m_visual_localization_ip;
    // 可视化定位端口
    size_t m_visual_localization_port;

    // 求解器迭代限制
    size_t m_solver_iteration_limit;
    // 求解器时间限制
    double m_solver_time_limit;

    // PARSCAC标志
    bool m_parsac_flag;
    // PARSCAC动态概率
    double m_parsac_dynamic_probability;
    // PARSCAC阈值
    double m_parsac_threshold;
    // PARSCAC归一化尺度
    double m_parsac_norm_scale;
    // PARSCAC关键帧检查大小
    size_t m_parsac_keyframe_check_size;

    // 旋转对准阈值
    double m_rotation_misalignment_threshold;
    // 旋转RANSAC阈值
    double m_rotation_ransac_threshold;
};

} // namespace xrslam::extra

#endif // XRSLAM_EXTRA_YAML_CONFIG_H
