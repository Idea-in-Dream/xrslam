#include <sstream>
#include <xrslam/extra/yaml_config.h>
#include <yaml-cpp/yaml.h>

namespace xrslam::extra {
 
// 用于在 YAML 配置中根据给定path(节点名)查找某个节点，并且可以选择是否抛出异常 mandatory：示是否强制要求找到该节点
static YAML::Node find_node(const YAML::Node &root, const std::string &path,
                            bool mandatory = false) {
    // 将传入的路径 path 转换为 stringstream 对象
    std::stringstream ss(path);
    // 定义一个字符串 child，用于存储路径中的每个节点名
    std::string child;
    // 初始化 node 变量为 root
    YAML::Node node = root;
    // 使用 while 循环遍历路径中的每个节点名
    // 使用 std::getline 从 ss 中逐个读取 child（即路径的每一部分），并通过 . 作为分隔符进行分割
    while (std::getline(ss, child, '.')) {
        // 每读取一部分 child，就通过 node[child] 获取当前 node 节点的子节点（child 对应的子节点）。
        // 然后，调用 reset 方法更新 node，使其指向新的子节点。这样，node 会逐级深入，直到路径的最后一部分
        node.reset(node[child]);
    }
    // 检查 node 是否有效 如果在路径查找过程中，任何一个节点不存在，node 将变为空节点
    if (!node) {
        // 如果节点无效且 mandatory 为 true，抛出一个自定义异常 YamlConfig::ConfigMissingException，并传递 path 作为错误信息
        if (mandatory) {
            throw YamlConfig::ConfigMissingException(path);
        }
    } else {
        // 如果节点有效，则将节点的标签设置为 path，即节点的名称
        node.SetTag(path);
    }
    // 返回找到的节点 node
    return node;
}

// 检查一个给定的 YAML::Node 是否符合预期的大小 n 且其每个元素都是标量值（即简单数据类型，如整数、浮点数、字符串等）
static void require_vector(const YAML::Node &node, size_t n) {
    // 返回 node 的元素数量  是否为预期值，否则抛出 YamlConfig::TypeErrorException，并传递 node 的标签作为错误信息
    if (node.size() != n) {
        throw YamlConfig::TypeErrorException(node.Tag());
    }
    // 遍历 node 的每个元素，检查其是否为标量值
    for (size_t i = 0; i < n; ++i) {
        if (!node[i].IsScalar()) {
            throw YamlConfig::TypeErrorException(node.Tag());
        }
    }
}

// 根据给定类型将 YAML 节点中的数据赋值到传入的变量 布尔型
static void assign(bool &value, const YAML::Node &node) {
    if (!node.IsScalar()) {
        throw YamlConfig::TypeErrorException(node.Tag());
    }
    value = node.as<bool>();
}

// 字符串型
static void assign(std::string &value, const YAML::Node &node) {
    if (!node.IsScalar()) {
        throw YamlConfig::TypeErrorException(node.Tag());
    }
    value = node.as<std::string>();
}

// 整型
static void assign(size_t &value, const YAML::Node &node) {
    if (!node.IsScalar()) {
        throw YamlConfig::TypeErrorException(node.Tag());
    }
    value = node.as<size_t>();
}
// 浮点型
static void assign(double &value, const YAML::Node &node) {
    if (!node.IsScalar()) {
        throw YamlConfig::TypeErrorException(node.Tag());
    }
    value = node.as<double>();
}

// 定义了一个模板函数 assign_vector，它将 YAML 节点中的数据赋值到传入的标量（vec）
template <typename V>
static void assign_vector(V &vec, const YAML::Node &node) {
    // 检查 node 是否是一个标量，并且其大小是否与 vec 的大小相同
    require_vector(node, vec.size());
    // 遍历 node 的每个元素，并将其值赋给 vec 的对应位置
    for (size_t i = 0; i < vec.size(); ++i) {
        vec[i] = node[i].as<double>();
    }
}

// 定义了一个模板函数 assign_matrix，它将 YAML 节点中的数据赋值到传入的矩阵（mat）
template <typename M>
static void assign_matrix(M &mat, const YAML::Node &node) {
    // 检查 node 是否是一个标量，并且其大小是否与 mat 的大小相同
    require_vector(node, mat.rows() * mat.cols());
    for (size_t i = 0; i < mat.rows(); ++i) {
        for (size_t j = 0; j < mat.cols(); ++j) {
            mat(i, j) = node[i * mat.cols() + j].template as<double>();
        }
    }
}

// 加载和解析这些配置文件
YamlConfig::YamlConfig(const std::string &slam_config_filename,
                       const std::string &device_config_filename) {
    // 为配置文件加载初值 Config类 调用的是xrslam的config
    
    // 获取Config类中机体系到相机系的旋转矩阵初始值
    m_output_to_body_rotation = Config::output_to_body_rotation();
    // 获取Config类中机体系到相机系的平移初始值
    m_output_to_body_translation = Config::output_to_body_translation();
    // 获取Config类中滑动窗口初始值
    m_sliding_window_size = Config::sliding_window_size();
    // 获取Config类中滑动窗口子帧大小初始值
    m_sliding_window_subframe_size = Config::sliding_window_subframe_size();
    // 获取Config类中滑动窗口跟踪器频率初始值
    m_sliding_window_tracker_frequent =
        Config::sliding_window_tracker_frequent();

    // 获取Config类滑动窗口键帧特征数量初始值
    m_sliding_window_force_keyframe_landmarks =
        Config::sliding_window_force_keyframe_landmarks();
    // 获取Config类特征跟踪器最小关键点距离初始值
    m_feature_tracker_min_keypoint_distance =
        Config::feature_tracker_min_keypoint_distance();
    // 获取Config类特征跟踪器最大关键点检测初始值
    m_feature_tracker_max_keypoint_detection =
        Config::feature_tracker_max_keypoint_detection();
    // 获取Config类的特征跟踪器最大初始化帧数
    m_feature_tracker_max_init_frames =
        Config::feature_tracker_max_init_frames();
    // 获取Config类的特征跟踪器最大帧数
    m_feature_tracker_max_frames = Config::feature_tracker_max_frames();
    // 获取Config类的特征跟踪器CLAHE裁剪限制
    m_feature_tracker_clahe_clip_limit =
        Config::feature_tracker_clahe_clip_limit();
    // 获取Config类的特征跟踪器CLAHE宽度
    m_feature_tracker_clahe_width = Config::feature_tracker_clahe_width();
    // 获取Config类的特征跟踪器CLAHE高度
    m_feature_tracker_clahe_height = Config::feature_tracker_clahe_height();
    // 获取Config类的特征跟踪器预测关键点
    m_feature_tracker_predict_keypoints =
        Config::feature_tracker_predict_keypoints();
    // 获取Config类的初始化关键帧数量
    m_initializer_keyframe_num = Config::initializer_keyframe_num();
    // 获取Config类的初始化关键帧间隔
    m_initializer_keyframe_gap = Config::initializer_keyframe_gap();
    // 获取Config类的初始化最小匹配数
    m_initializer_min_matches = Config::initializer_min_matches();
    // 获取Config类的初始化最小视差
    m_initializer_min_parallax = Config::initializer_min_parallax();
    // 获取Config类的初始化最小三角测量
    m_initializer_min_triangulation = Config::initializer_min_triangulation();
    // 获取Config类的初始化最小特征点
    m_initializer_min_landmarks = Config::initializer_min_landmarks();
    // 获取Config类的初始化是否优化IMU
    m_initializer_refine_imu = Config::initializer_refine_imu();
    // 获取Config类的ceres求解器迭代次数初始值
    m_solver_iteration_limit = Config::solver_iteration_limit();
    // 获取Config类的ceres求解器时间限制初始值
    m_solver_time_limit = Config::solver_time_limit();

    // 获取Config类的Parsac标志
    m_parsac_flag = Config::parsac_flag();
    // 获取Config类的Parsac动态概率
    m_parsac_dynamic_probability = Config::parsac_dynamic_probability();
    // 获取Config类的Parsac阈值
    m_parsac_threshold = Config::parsac_threshold();
    // 获取Config类的Parsac归一化尺度
    m_parsac_norm_scale = Config::parsac_norm_scale();

    // 获取Config类的旋转不匹配阈值
    m_rotation_misalignment_threshold =
        Config::rotation_misalignment_threshold();
    // 获取Config类的旋转RANSAC阈值
    m_rotation_ransac_threshold = Config::rotation_ransac_threshold();
    // 获取Config类的视觉定位是否启用
    m_visual_localization_enable = Config::visual_localization_enable();
    // 获取Config类的视觉定位配置IP
    m_visual_localization_ip = Config::visual_localization_config_ip();
    // 获取Config类的视觉定位配置端口
    m_visual_localization_port = Config::visual_localization_config_port();

    // 定义两个YAML::Node类型的变量，用于存储SLAM配置和设备配置
    YAML::Node slam_config;
    YAML::Node device_config;
    try {
        // 加载SLAM 算法参数 YAML 配置文件
#if defined(XRSLAM_IOS)
        slam_config = YAML::Load(slam_config_filename);
#else
        slam_config = YAML::LoadFile(slam_config_filename);
#endif
        // 捕获并处理解析异常
        // 如果 YAML 解析出现问题（YAML::ParserException），则抛出自定义的 ParseException 异常
    } catch (const YAML::ParserException &parse_error) {
        throw ParseException(parse_error.what());
        // 捕获并处理其他异常 都会抛出自定义的 LoadException，并附带文件名作为错误信息。
    } catch (...) {
        throw LoadException(slam_config_filename);
    }

    // 加载 设备参数 YAML 配置文件
    try {
#if defined(XRSLAM_IOS)
        device_config = YAML::Load(device_config_filename);
#else
        device_config = YAML::LoadFile(device_config_filename);
#endif
        // 捕获并处理解析异常
        // 如果 YAML 解析出现问题（YAML::ParserException），则抛出自定义的 ParseException 异常
    } catch (const YAML::ParserException &parse_error) {
        throw ParseException(parse_error.what());
        // 捕获并处理其他异常 都会抛出自定义的 LoadException，并附带文件名作为错误信息。
    } catch (...) {
        throw LoadException(device_config_filename);
    }
    // 从设备配置中获取相机内参
    if (auto intrinsic = find_node(device_config, "cam0.intrinsics", true)) {
        // 检查内参是否为4个元素的标量
        require_vector(intrinsic, 4);
        // 将相机内参矩阵初始化为单位矩阵
        m_camera_intrinsic.setIdentity();
        // 将内参赋值给相机内参矩阵
        m_camera_intrinsic(0, 0) = intrinsic[0].as<double>();
        m_camera_intrinsic(1, 1) = intrinsic[1].as<double>();
        m_camera_intrinsic(0, 2) = intrinsic[2].as<double>();
        m_camera_intrinsic(1, 2) = intrinsic[3].as<double>();
    }
    // 从设备配置中获取相机畸变系数
    if (auto node = find_node(device_config, "cam0.distortion", true)) {
        assign_vector(m_camera_distortion, node);
    }
    // 从设备配置中获取相机畸变标志
    if (auto node =
            find_node(device_config, "cam0.camera_distortion_flag", true)) {
        assign(m_camera_distortion_flag, node);
    }
    // 从设备配置中获取相机IMU时间偏移
    if (auto node = find_node(device_config, "cam0.time_offset", true)) {
        assign(m_camera_time_offset, node);
    }
    // 从设备配置中获取相机分辨率
    if (auto node = find_node(device_config, "cam0.resolution", true)) {
        assign_vector(m_camera_resolution, node);
    }
    // 从设备配置中获取相机到设备的旋转
    if (auto node = find_node(device_config, "cam0.extrinsic.q_bc", true)) {
        assign_vector(m_camera_to_body_rotation.coeffs(), node);
    }
    // 从设备配置中获取相机到设备的平移
    if (auto node = find_node(device_config, "cam0.extrinsic.p_bc", true)) {
        assign_vector(m_camera_to_body_translation, node);
    }
    // 从设备配置中获取相机噪声协方差矩阵
    if (auto node = find_node(device_config, "cam0.noise", true)) {
        assign_matrix(m_keypoint_noise_cov, node);
    }
    // 从设备配置中获取IMU到设备的旋转
    if (auto node = find_node(device_config, "imu.extrinsic.q_bi", true)) {
        assign_vector(m_imu_to_body_rotation.coeffs(), node);
    }
    // 从设备配置中获取IMU到设备的平移
    if (auto node = find_node(device_config, "imu.extrinsic.p_bi", true)) {
        assign_vector(m_imu_to_body_translation, node);
    }
    // 从设备配置中获取陀螺仪噪声协方差矩阵
    if (auto node = find_node(device_config, "imu.noise.cov_g", true)) {
        assign_matrix(m_gyroscope_noise_cov, node);
    }
    // 从设备配置中获取加速度计噪声协方差矩阵
    if (auto node = find_node(device_config, "imu.noise.cov_a", true)) {
        assign_matrix(m_accelerometer_noise_cov, node);
    }
    // 从设备配置中获取陀螺仪偏置噪声协方差矩阵
    if (auto node = find_node(device_config, "imu.noise.cov_bg", true)) {
        assign_matrix(m_gyroscope_bias_noise_cov, node);
    }
    // 从设备配置中获取加速度计偏置噪声协方差矩阵
    if (auto node = find_node(device_config, "imu.noise.cov_ba", true)) {
        assign_matrix(m_accelerometer_bias_noise_cov, node);
    }
    // 从设备配置中获取输出到设备的旋转 （非必须）
    if (auto node = find_node(slam_config, "output.q_bo", false)) {
        assign_vector(m_output_to_body_rotation.coeffs(), node);
    }
    // 从设备配置中获取输出到设备的旋转 （非必须）
    if (auto node = find_node(slam_config, "output.p_bo", false)) {
        assign_vector(m_output_to_body_translation, node);
    }
    // 从算法配置中获取滑动窗口大小 （非必须）
    if (auto node = find_node(slam_config, "sliding_window.size", false)) {
        assign(m_sliding_window_size, node);
    }
    // 从算法配置中获取滑动窗口子帧大小 （非必须）
    if (auto node =
            find_node(slam_config, "sliding_window.subframe_size", false)) {
        assign(m_sliding_window_subframe_size, node);
    }
    // 从算法配置中获取滑动窗口跟踪频率 （非必须）
    if (auto node =
            find_node(slam_config, "sliding_window.tracker_frequent", false)) {
        assign(m_sliding_window_tracker_frequent, node);
    }
    // 从算法配置中强制关键帧特征点 （非必须）
    if (auto node = find_node(
            slam_config, "sliding_window.force_keyframe_landmarks", false)) {
        assign(m_sliding_window_force_keyframe_landmarks, node);
    }
    // 从算法配置中获取关键点最小距离 （非必须）
    if (auto node = find_node(slam_config,
                              "feature_tracker.min_keypoint_distance", false)) {
        assign(m_feature_tracker_min_keypoint_distance, node);
    }
    // 从算法配置中获取最大关键点检测数 （非必须）
    if (auto node = find_node(
            slam_config, "feature_tracker.max_keypoint_detection", false)) {
        assign(m_feature_tracker_max_keypoint_detection, node);
    }
    // 从算法配置中获取最大初始化帧数 （非必须）
    if (auto node =
            find_node(slam_config, "feature_tracker.max_init_frames", false)) {
        assign(m_feature_tracker_max_init_frames, node);
    }
    // 从算法配置中获取最大跟踪帧数 （非必须）
    if (auto node =
            find_node(slam_config, "feature_tracker.max_frames", false)) {
        assign(m_feature_tracker_max_frames, node);
    }
    // 从算法配置中获取CLAHE裁剪限制 （非必须）
    if (auto node =
            find_node(slam_config, "feature_tracker.clahe_clip_limit", false)) {
        assign(m_feature_tracker_clahe_clip_limit, node);
    }
    // 从算法配置中获取CLAHE宽高 （非必须）
    if (auto node =
            find_node(slam_config, "feature_tracker.clahe_width", false)) {
        assign(m_feature_tracker_clahe_width, node);
    }

    if (auto node =
            find_node(slam_config, "feature_tracker.clahe_height", false)) {
        assign(m_feature_tracker_clahe_height, node);
    }
    // 从算法配置中获取是否预测关键点 （非必须）
    if (auto node = find_node(slam_config, "feature_tracker.predict_keypoints",
                              false)) {
        assign(m_feature_tracker_predict_keypoints, node);
    }
    // 从算法配置中获取初始化关键帧数量 （非必须）
    if (auto node = find_node(slam_config, "initializer.keyframe_num", false)) {
        assign(m_initializer_keyframe_num, node);
    }
    // 从算法配置中获取初始化关键帧间隔 （非必须）
    if (auto node = find_node(slam_config, "initializer.keyframe_gap", false)) {
        assign(m_initializer_keyframe_gap, node);
    }
    // 从算法配置中获取初始化最小匹配数 （非必须）
    if (auto node = find_node(slam_config, "initializer.min_matches", false)) {
        assign(m_initializer_min_matches, node);
    }
    // 从算法配置中获取初始化最小视差 （非必须）
    if (auto node = find_node(slam_config, "initializer.min_parallax", false)) {
        assign(m_initializer_min_parallax, node);
    }
    // 从算法配置中获取初始化最小三角化 （非必须）
    if (auto node = 
            find_node(slam_config, "initializer.min_triangulation", false)) {
        assign(m_initializer_min_triangulation, node);
    }
    // 从算法配置中获取初始化最小特征点数 （非必须）
    if (auto node =
            find_node(slam_config, "initializer.min_landmarks", false)) {
        assign(m_initializer_min_landmarks, node);
    }
    // 从算法配置中获取初始化是否优化IMU （非必须）
    if (auto node = find_node(slam_config, "initializer.refine_imu", false)) {
        assign(m_initializer_refine_imu, node);
    }
    // 从算法配置中获取是否启用视觉定位 （非必须）
    if (auto node =
            find_node(slam_config, "visual_localization.enable", false)) {
        assign(m_visual_localization_enable, node);
    }
    // 从算法配置中获取视觉定位IP （非必须）
    if (auto node = find_node(slam_config, "visual_localization.ip", false)) {
        assign(m_visual_localization_ip, node);
    }
    // 从算法配置中获取视觉定位端口 （非必须）
    if (auto node = find_node(slam_config, "visual_localization.port", false)) {
        assign(m_visual_localization_port, node);
    }
    // 从算法配置中获取迭代器迭代次数限制 （非必须）
    if (auto node = find_node(slam_config, "solver.iteration_limit", false)) {
        assign(m_solver_iteration_limit, node);
    }
    // 从算法配置中获取迭代器时间限制 （非必须）
    if (auto node = find_node(slam_config, "solver.time_limit", false)) {
        assign(m_solver_time_limit, node);
    }
    // 从算法配置中获取是否启用parsac （非必须）
    if (auto node = find_node(slam_config, "parsac.parsac_flag", false)) {
        assign(m_parsac_flag, node);
    }
    // 从算法配置中获取parsac动态概率 （非必须）
    if (auto node =
            find_node(slam_config, "parsac.dynamic_probability", false)) {
        assign(m_parsac_dynamic_probability, node);
    }
    // 从算法配置中获取parsac阈值 （非必须）
    if (auto node = find_node(slam_config, "parsac.threshold", false)) {
        assign(m_parsac_threshold, node);
    }
    // 从算法配置中获取parsac归一化尺度 （非必须）
    if (auto node = find_node(slam_config, "parsac.norm_scale", false)) {
        assign(m_parsac_norm_scale, node);
    }
    // 从算法配置中获取parsac关键帧检查大小 （非必须）
    if (auto node =
            find_node(slam_config, "parsac.keyframe_check_size", false)) {
        assign(m_parsac_keyframe_check_size, node);
    }
    // 从算法配置中获取旋转不匹配阈值 （非必须）
    if (auto node =
            find_node(slam_config, "rotation.misalignment_threshold", false)) {
        assign(m_rotation_misalignment_threshold, node);
    }
    // 从算法配置中获取旋转ransac阈值 （非必须）
    if (auto node =
            find_node(slam_config, "rotation.ransac_threshold", false)) {
        assign(m_rotation_ransac_threshold, node);
    }
}

YamlConfig::~YamlConfig() = default;
// 从相机配置中获取相机内参
matrix<3> YamlConfig::camera_intrinsic() const { return m_camera_intrinsic; }
// 从相机配置中获取相机畸变参数
vector<4> YamlConfig::camera_distortion() const { return m_camera_distortion; }
// 从相机配置中获取相机畸变标志
size_t YamlConfig::camera_distortion_flag() const {
    return m_camera_distortion_flag;
}
// 获取相机时间偏移
double YamlConfig::camera_time_offset() const { return m_camera_time_offset; }
// 获取相机分辨率
vector<2> YamlConfig::camera_resolution() const { return m_camera_resolution; }
// 获取相机到机体系的旋转
quaternion YamlConfig::camera_to_body_rotation() const {
    return m_camera_to_body_rotation;
}
// 获取相机到机体系的平移
vector<3> YamlConfig::camera_to_body_translation() const {
    return m_camera_to_body_translation;
}
// 获取IMU到机体系的旋转
quaternion YamlConfig::imu_to_body_rotation() const {
    return m_imu_to_body_rotation;
}
// 获取IMU到机体系的平移
vector<3> YamlConfig::imu_to_body_translation() const {
    return m_imu_to_body_translation;
}

// 返回关键点噪声协方差矩阵
matrix<2> YamlConfig::keypoint_noise_cov() const {
    return m_keypoint_noise_cov;
}

// 返回陀螺仪噪声协方差矩阵
matrix<3> YamlConfig::gyroscope_noise_cov() const {
    return m_gyroscope_noise_cov;
}

// 返回加速度计噪声协方差矩阵
matrix<3> YamlConfig::accelerometer_noise_cov() const {
    return m_accelerometer_noise_cov;
}

// 返回陀螺仪偏置噪声协方差矩阵
matrix<3> YamlConfig::gyroscope_bias_noise_cov() const {
    return m_gyroscope_bias_noise_cov;
}

// 返回加速度计偏置噪声协方差矩阵
matrix<3> YamlConfig::accelerometer_bias_noise_cov() const {
    return m_accelerometer_bias_noise_cov;
}

// 返回输出到机体系的旋转四元数
quaternion YamlConfig::output_to_body_rotation() const {
    return m_output_to_body_rotation;
}

// 返回输出到机体系的平移向量
vector<3> YamlConfig::output_to_body_translation() const {
    return m_output_to_body_translation;
}

// 返回滑动窗口大小
size_t YamlConfig::sliding_window_size() const { return m_sliding_window_size; }

// 返回滑动窗口子帧大小
size_t YamlConfig::sliding_window_subframe_size() const {
    return m_sliding_window_subframe_size;
}

// 返回滑动窗口跟踪器频率
size_t YamlConfig::sliding_window_tracker_frequent() const {
    return m_sliding_window_tracker_frequent;
}

// 返回滑动窗口强制关键帧地图点数量
size_t YamlConfig::sliding_window_force_keyframe_landmarks() const {
    return m_sliding_window_force_keyframe_landmarks;
}

// 返回特征跟踪器关键点相邻最小距离
double YamlConfig::feature_tracker_min_keypoint_distance() const {
    return m_feature_tracker_min_keypoint_distance;
}

// 返回特征跟踪器最大关键点检测数量
size_t YamlConfig::feature_tracker_max_keypoint_detection() const {
    return m_feature_tracker_max_keypoint_detection;
}

// 返回特征跟踪器最大初始化帧数
size_t YamlConfig::feature_tracker_max_init_frames() const {
    return m_feature_tracker_max_init_frames;
}

// 返回特征跟踪器最大跟踪帧
size_t YamlConfig::feature_tracker_max_frames() const {
    return m_feature_tracker_max_frames;
}

// 返回特征跟踪器CLAHE裁剪限制
double YamlConfig::feature_tracker_clahe_clip_limit() const {
    return m_feature_tracker_clahe_clip_limit;
}

// 返回特征跟踪器CLAHE宽度
size_t YamlConfig::feature_tracker_clahe_width() const {
    return m_feature_tracker_clahe_width;
}

// 返回特征跟踪器CLAHE高度
size_t YamlConfig::feature_tracker_clahe_height() const {
    return m_feature_tracker_clahe_height;
}

// 返回是否预测关键点
bool YamlConfig::feature_tracker_predict_keypoints() const {
    return m_feature_tracker_predict_keypoints;
}

// 返回初始化关键帧数量
size_t YamlConfig::initializer_keyframe_num() const {
    return m_initializer_keyframe_num;
}

// 返回初始化关键帧间隔
size_t YamlConfig::initializer_keyframe_gap() const {
    return m_initializer_keyframe_gap;
}

// 返回初始化最小匹配数
size_t YamlConfig::initializer_min_matches() const {
    return m_initializer_min_matches;
}

// 返回初始化最小视差阈值
double YamlConfig::initializer_min_parallax() const {
    return m_initializer_min_parallax;
}

// 返回初始化最小三角化
size_t YamlConfig::initializer_min_triangulation() const {
    return m_initializer_min_triangulation;
}

// 返回初始化最小地图点数量
size_t YamlConfig::initializer_min_landmarks() const {
    return m_initializer_min_landmarks;
}

// 返回是否细化IMU
bool YamlConfig::initializer_refine_imu() const {
    return m_initializer_refine_imu;
}

// 返回是否启用视觉定位
bool YamlConfig::visual_localization_enable() const {
    return m_visual_localization_enable;
}

// 返回视觉定位配置IP
std::string YamlConfig::visual_localization_config_ip() const {
    return m_visual_localization_ip;
}

// 返回视觉定位配置端口
size_t YamlConfig::visual_localization_config_port() const {
    return m_visual_localization_port;
}

// 返回求解器迭代限制
size_t YamlConfig::solver_iteration_limit() const {
    return m_solver_iteration_limit;
}

// 返回求解器时间限制
double YamlConfig::solver_time_limit() const { return m_solver_time_limit; }

// 返回是否启用Parsac
bool YamlConfig::parsac_flag() const { return m_parsac_flag; }

// 返回Parsac动态概率
double YamlConfig::parsac_dynamic_probability() const {
    return m_parsac_dynamic_probability;
}

// 返回Parsac阈值
double YamlConfig::parsac_threshold() const { return m_parsac_threshold; }

// 返回Parsac归一化尺度
double YamlConfig::parsac_norm_scale() const { return m_parsac_norm_scale; }

// 返回Parsac关键帧检查大小
size_t YamlConfig::parsac_keyframe_check_size() const {
    return m_parsac_keyframe_check_size;
}

// 返回旋转不匹配阈值
double YamlConfig::rotation_misalignment_threshold() const {
    return m_rotation_misalignment_threshold;
}

// 返回旋转RANSAC阈值
double YamlConfig::rotation_ransac_threshold() const {
    return m_rotation_ransac_threshold;
}

} // namespace xrslam::extra
