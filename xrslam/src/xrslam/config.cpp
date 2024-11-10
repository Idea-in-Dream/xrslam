#include <xrslam/common.h>

namespace xrslam {
// 析构函数使用默认的实现 
// 析构函数是一个类中的特殊成员函数，它会在对象生命周期结束时被自动调用。析构函数通常用来释放对象的资源，例如内存、文件句柄、网络连接等。
// default 告诉编译器自动生成该函数的默认实现
Config::~Config() = default;
// 相机内参
vector<4> Config::camera_distortion() const { return vector<4>::Zero(); }

// 输出body坐标系到相机坐标系的旋转四元数，默认为单位矩阵
quaternion Config::output_to_body_rotation() const {
    return quaternion::Identity();
}

// 输出body坐标系到相机坐标系的平移向量，默认为零向量
vector<3> Config::output_to_body_translation() const {
    return vector<3>::Zero();
}

// 返回滑动窗口大小，默认为10
size_t Config::sliding_window_size() const { return 10; }

// 返回滑动窗口子帧大小，默认为3
size_t Config::sliding_window_subframe_size() const { return 3; }

// 返回滑动窗口强制关键帧特征点数量，默认为35
size_t Config::sliding_window_force_keyframe_landmarks() const { return 35; }

// 返回特征跟踪器最小关键点距离
double Config::feature_tracker_min_keypoint_distance() const { return 20.0; }

// 返回特征跟踪器最大关键点检测数量
size_t Config::feature_tracker_max_keypoint_detection() const { return 150; }

// 返回特征跟踪器最大初始化帧数量
size_t Config::feature_tracker_max_init_frames() const { return 60; }

// 返回特征跟踪器最大帧数量
size_t Config::feature_tracker_max_frames() const { return 200; }

// 返回特征跟踪器CLAHE裁剪限制
double Config::feature_tracker_clahe_clip_limit() const { return 6.0; }

// 返回特征跟踪器CLAHE宽度
size_t Config::feature_tracker_clahe_width() const { return 8; }

// 返回特征跟踪器CLAHE高度
size_t Config::feature_tracker_clahe_height() const { return 8; }

// 返回是否预测关键点
bool Config::feature_tracker_predict_keypoints() const { return true; }

// 返回初始化器关键帧数量
size_t Config::initializer_keyframe_num() const { return 8; }

// 返回初始化器关键帧间隔
size_t Config::initializer_keyframe_gap() const { return 5; }

// 返回初始化器最小匹配数量
size_t Config::initializer_min_matches() const { return 50; }

// 返回初始化器最小视差
double Config::initializer_min_parallax() const { return 10; }

// 返回初始化器最小三角测量数量
size_t Config::initializer_min_triangulation() const { return 50; }

// 返回初始化器最小特征点数量
size_t Config::initializer_min_landmarks() const { return 30; }

// 返回是否优化IMU
bool Config::initializer_refine_imu() const { return true; }

// 返回是否启用视觉定位
bool Config::visual_localization_enable() const { return false; }

// 返回视觉定位配置IP
std::string Config::visual_localization_config_ip() const { return "0.0.0.0"; }

// 返回视觉定位配置端口
size_t Config::visual_localization_config_port() const { return 0; }

// 返回求解器迭代限制
size_t Config::solver_iteration_limit() const { return 10; }

// 返回求解器时间限制
double Config::solver_time_limit() const { return 1.0e6; }

// 返回旋转不匹配阈值
double Config::rotation_misalignment_threshold() const { return 0.1; }

// 返回旋转RANSAC阈值
double Config::rotation_ransac_threshold() const { return 10; }

// 返回随机数
int Config::random() const { return 648; }

// 返回是否启用Parsac
bool Config::parsac_flag() const { return false; }

// 返回Parsac动态概率
double Config::parsac_dynamic_probability() const { return 0.0; }

// 返回Parsac阈值
double Config::parsac_threshold() const { return 3.0; }

// 返回Parsac归一化比例
double Config::parsac_norm_scale() const { return 1.0; }

// 返回Parsac关键帧检查大小
size_t Config::parsac_keyframe_check_size() const { return 3; }

// 返回滑动窗口跟踪器频率
size_t Config::sliding_window_tracker_frequent() const { return 1; }

// Config 类的配置信息记录到日志中
void Config::log_config() const {
    // 将数据格式化并转换成字符串
    std::stringstream ss;
    // 设置输出格式 科学技术法 布尔值的文本输出 浮点数输出的精度
    ss << std::scientific << std::boolalpha << std::setprecision(5);

    ss << "Config::camera_intrinsic:\n"
       << camera_intrinsic() << "\n"
       << std::endl;

    ss << "Config::camera_distortion_flag:\n"
       << camera_distortion_flag() << "\n"
       << std::endl;

    ss << "Config::camera_distortion:\n"
       << camera_distortion().transpose() << "\n"
       << std::endl;

    ss << "Config::camera_time_offset:\n"
       << camera_time_offset() << "\n"
       << std::endl;

    ss << "Config::camera_to_body_rotation:\n"
       << camera_to_body_rotation().coeffs().transpose() << "\n"
       << std::endl;

    ss << "Config::camera_to_body_translation:\n"
       << camera_to_body_translation().transpose() << "\n"
       << std::endl;

    ss << "Config::imu_to_body_rotation:\n"
       << imu_to_body_rotation().coeffs().transpose() << "\n"
       << std::endl;

    ss << "Config::imu_to_body_translation:\n"
       << imu_to_body_translation().transpose() << "\n"
       << std::endl;

    ss << "Config::keypoint_noise_cov:\n"
       << keypoint_noise_cov() << "\n"
       << std::endl;

    ss << "Config::gyroscope_noise_cov:\n"
       << gyroscope_noise_cov() << "\n"
       << std::endl;

    ss << "Config::accelerometer_noise_cov:\n"
       << accelerometer_noise_cov() << "\n"
       << std::endl;

    ss << "Config::gyroscope_bias_noise_cov:\n"
       << gyroscope_bias_noise_cov() << "\n"
       << std::endl;

    ss << "Config::accelerometer_bias_noise_cov:\n"
       << accelerometer_bias_noise_cov() << "\n"
       << std::endl;

    ss << "Config::sliding_window_size: " << sliding_window_size() << std::endl;

    ss << "Config::sliding_window_subframe_size: "
       << sliding_window_subframe_size() << std::endl;

    ss << "Config::sliding_window_force_keyframe_landmarks: "
       << sliding_window_force_keyframe_landmarks() << std::endl;

    ss << "Config::sliding_window_tracker_frequent: "
       << sliding_window_tracker_frequent() << std::endl;

    ss << "Config::feature_tracker_min_keypoint_distance: "
       << feature_tracker_min_keypoint_distance() << std::endl;

    ss << "Config::feature_tracker_max_keypoint_detection: "
       << feature_tracker_max_keypoint_detection() << std::endl;

    ss << "Config::feature_tracker_max_init_frames: "
       << feature_tracker_max_init_frames() << std::endl;

    ss << "Config::feature_tracker_max_frames: " << feature_tracker_max_frames()
       << std::endl;

    ss << "Config::feature_tracker_predict_keypoints: "
       << feature_tracker_predict_keypoints() << std::endl;

    ss << "Config::feature_tracker_clahe_clip_limit: "
       << feature_tracker_clahe_clip_limit() << std::endl;

    ss << "Config::feature_tracker_clahe_width: "
       << feature_tracker_clahe_width() << std::endl;

    ss << "Config::feature_tracker_clahe_height: "
       << feature_tracker_clahe_height() << std::endl;

    ss << "Config::initializer_keyframe_gap: " << initializer_keyframe_gap()
       << std::endl;

    ss << "Config::initializer_min_matches: " << initializer_min_matches()
       << std::endl;

    ss << "Config::initializer_min_parallax: " << initializer_min_parallax()
       << std::endl;

    ss << "Config::initializer_min_triangulation: "
       << initializer_min_triangulation() << std::endl;

    ss << "Config::initializer_min_landmarks: " << initializer_min_landmarks()
       << std::endl;

    ss << "Config::initializer_refine_imu: " << initializer_refine_imu()
       << std::endl;

    ss << "Config::visual_localization_enable: " << visual_localization_enable()
       << std::endl;

    ss << "Config::visual_localization_config_ip: "
       << visual_localization_config_ip() << std::endl;

    ss << "Config::visual_localization_config_port: "
       << visual_localization_config_port() << std::endl;

    ss << "Config::solver_iteration_limit: " << solver_iteration_limit()
       << std::endl;

    ss << "Config::solver_time_limit: " << solver_time_limit() << std::endl;

    ss << "Config::parsac_flag: " << parsac_flag() << std::endl;

    ss << "Config::parsac_dynamic_probability: " << parsac_dynamic_probability()
       << std::endl;

    ss << "Config::parsac_threshold: " << parsac_threshold() << std::endl;

    ss << "Config::parsac_norm_scale: " << parsac_norm_scale() << std::endl;

    ss << "Config::rotation_misalignment_threshold: "
       << rotation_misalignment_threshold() << std::endl;

    ss << "Config::rotation_ransac_threshold: " << rotation_ransac_threshold()
       << std::endl;

#if defined(XRSLAM_ENABLE_THREADING)
    ss << std::endl;
    ss << "THREADING ENABLE" << std::endl;
#else
    ss << std::endl;
    ss << "THREADING DISABLE" << std::endl;
#endif

    log_info("Configurations: \n%s", ss.str().c_str());
}

} // namespace xrslam
