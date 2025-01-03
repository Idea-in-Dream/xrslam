#include <xrslam/common.h>
#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/frontend_worker.h>
#include <xrslam/estimation/solver.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/inspection.h>
#include <xrslam/localizer/localizer.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>

namespace xrslam {

// 用于根据 IMU 数据（陀螺仪和加速度计）对设备的状态（位置、速度、姿态）进行时间传播（状态预测）
static void propagate_state(double &state_time, PoseState &state_pose,
                            MotionState &state_motion, double t,
                            const vector<3> &w, const vector<3> &a) {
    // 定义重力常量
    static const vector<3> gravity = {0, 0, -XRSLAM_GRAVITY_NOMINAL};
    // 时间步长计算，用于积分的时间间隔
    double dt = t - state_time;
    // 当前位姿
    state_pose.p =
        state_pose.p + dt * state_motion.v +
        0.5 * dt * dt * (gravity + state_pose.q * (a - state_motion.ba));
    // 当前速度    
    state_motion.v =
        state_motion.v + dt * (gravity + state_pose.q * (a - state_motion.ba));
    // 当前姿态    
    state_pose.q =
        (state_pose.q * expmap((w - state_motion.bg) * dt)).normalized();
    // 更新时间戳
    state_time = t;
}
// Detail 构造函数通过传入的 config 对象初始化了 Detail 类的成员变量
XRSLAM::Detail::Detail(std::shared_ptr<Config> config) : config(config) {
    // 调用 Solver 类的 init 静态方法，传入配置对象 config 的原始指针
    Solver::init(config.get());
    // 创建一个 FrontendWorker 对象，用于处理 SLAM 系统的前端任务
    frontend = std::make_unique<FrontendWorker>(this, config);
    // 创建一个 FeatureTracker 对象，用于处理特征点跟踪任务
    feature_tracker = std::make_unique<FeatureTracker>(this, config);

    frontend->start();
    feature_tracker->start();
}

XRSLAM::Detail::~Detail() {
    feature_tracker->stop();
    frontend->stop();
}

const Config *XRSLAM::Detail::configurations() const { return config.get(); }

// 用于处理陀螺仪数据，并利用加速度数据对姿态进行估计。
// t：当前陀螺仪数据的时间戳。x、y、z：陀螺仪数据的三轴角速度。
Pose XRSLAM::Detail::track_gyroscope(const double &t, const double &x,
                                     const double &y, const double &z) {
    // 检查是否已有加速度数据
    if (accelerometers.size() > 0) {
        // 如果陀螺仪时间戳早于最早的加速度数据,，则清空陀螺仪数据。
        if (t < accelerometers.front().t) {
            gyroscopes.clear();
        } else {
            // 同步加速度数据
            // 对每个时间戳小于等于 t 的加速度数据
            // 1. 计算加速度时间和陀螺仪数据之间的角速度插值（线性插值）

            // 当前陀螺仪时间戳数据 > 第一帧加速度计的数据
            //     得到当前加速度计的数据
            //     计算当前陀螺仪时间戳与加速度计时间戳之间的插值
            //     根据线性差值求解 acc.t所时刻下的 gyro
            //     将 时间戳，陀螺仪，加速度数据存入 track_imu
            //     移除已处理的加速度数据
            while (accelerometers.size() > 0 && t >= accelerometers.front().t) {
                const auto &acc = accelerometers.front();
                double lambda =
                    (acc.t - gyroscopes[0].t) / (t - gyroscopes[0].t);
                vector<3> w = gyroscopes[0].w +
                              lambda * (vector<3>{x, y, z} - gyroscopes[0].w);
                track_imu({acc.t, w, acc.a});
                accelerometers.pop_front();
            }
            // 移除时间戳早于当前时间 t 的陀螺仪数据
            if (accelerometers.size() > 0) {
                while (gyroscopes.size() > 0 && gyroscopes.front().t < t) {
                    gyroscopes.pop_front();
                }
            }
        }
    }
    // 存储当前陀螺仪数据
    gyroscopes.emplace_back(GyroscopeData{t, {x, y, z}});
    // 返回预测的姿态
    return predict_pose(t);
}
// 处理加速度计数据
Pose XRSLAM::Detail::track_accelerometer(const double &t, const double &x,
                                         const double &y, const double &z) {
    // 检查陀螺仪数据是否可用,前加速度数据时间戳大于或等于陀螺仪数据最早时间戳
    if (gyroscopes.size() > 0 && t >= gyroscopes.front().t) {
        // 如果加速度数据时间戳大于陀螺仪最新时间戳
        if (t > gyroscopes.back().t) {
            // 保留最近的一个陀螺仪数据
            while (gyroscopes.size() > 1) {
                gyroscopes.pop_front();
            }
            // t > gyroscopes[0].t
            // 将当前加速度数据存储到队列中
            accelerometers.emplace_back(AccelerometerData{t, {x, y, z}});
        } else if (t == gyroscopes.back().t) {
            // 如果加速度时间戳与陀螺仪最新时间戳相等
            // 1. 保留最近的一个陀螺仪数据
            while (gyroscopes.size() > 1) {
                gyroscopes.pop_front();
            }
            // 2. 调用 track_imu 直接处理当前加速度和陀螺仪数据
            track_imu({t, gyroscopes.front().w, {x, y, z}});
        } else {
            // pre-condition: gyroscopes.front().t <= t < gyroscopes.back().t
            // ==>  gyroscopes.size() >= 2
            // 如果加速度时间戳位于两个陀螺仪时间戳之间
            // 1. 移除多余的陀螺仪数据，仅保留与加速度数据相关的两个时间点。
            // 2. 对这两个时间点的数据进行插值，计算当前时间点的角速度。
            // 3. 调用 track_imu 处理插值后的角速度和加速度
            while (t >= gyroscopes[1].t) {
                gyroscopes.pop_front();
            }
            // post-condition: t < gyroscopes[1].t
            double lambda =
                (t - gyroscopes[0].t) / (gyroscopes[1].t - gyroscopes[0].t);
            vector<3> w =
                gyroscopes[0].w + lambda * (gyroscopes[1].w - gyroscopes[0].w);
            track_imu({t, w, {x, y, z}});
        }
    }
    // 返回姿态预测结果
    return predict_pose(t);
}

// 处理输入的图像数据，生成一个新的帧 (Frame)，将其配置为当前相机帧，并更新最新的姿态估计（Pose);
Pose XRSLAM::Detail::track_camera(std::shared_ptr<Image> image) {
    // 创建新帧 (Frame) 并初始化其属性
    std::unique_ptr<Frame> frame = std::make_unique<Frame>();
    // frame->K：设置为当前相机的内参（从配置文件 config 中获取）
    frame->K = config->camera_intrinsic();
    // 存储传入的图像数据
    frame->image = image;
    // 关键点噪声协方差矩阵
    // 从相机内参矩阵中提取前两行两列 (block<2, 2>(0, 0))，设置为协方差矩阵的平方根倒数。
    frame->sqrt_inv_cov = frame->K.block<2, 2>(0, 0);
    frame->sqrt_inv_cov(0, 0) /= ::sqrt(config->keypoint_noise_cov()(0, 0));
    frame->sqrt_inv_cov(1, 1) /= ::sqrt(config->keypoint_noise_cov()(1, 1));
    // 设置帧的外参和 IMU 参数
    // 相机到 机体系 的旋转和平移
    frame->camera.q_cs = config->camera_to_body_rotation();
    frame->camera.p_cs = config->camera_to_body_translation();
    // IMU 到 机体系 的旋转和平移
    frame->imu.q_cs = config->imu_to_body_rotation();
    frame->imu.p_cs = config->imu_to_body_translation();
    // 设置加速度、陀螺仪的噪声和偏置协方差
    frame->preintegration.cov_a = config->accelerometer_noise_cov();
    frame->preintegration.cov_w = config->gyroscope_noise_cov();
    frame->preintegration.cov_ba = config->accelerometer_bias_noise_cov();
    frame->preintegration.cov_bg = config->gyroscope_bias_noise_cov();
    // 将新帧添加到帧队列中
    frames.emplace_back(std::move(frame));
    // 预测当前图像帧的姿态
    Pose outpose = predict_pose(image->t);
    // 更新最新的姿态估计和对应的时间戳
    std::unique_lock<std::mutex> lk(latest_mutex_);
    if (image->t > latest_timestamp_) {
        latest_pose_ = outpose;
        latest_timestamp_ = image->t;
    }
    // 返回预测的姿态
    return outpose;
}

// 处理输入的 IMU 数据，并将其添加到 IMU 数据队列中，同时根据需要更新帧队列中的帧
void XRSLAM::Detail::track_imu(const ImuData &imu) {
    // 新接收到的 IMU 数据 imu 添加到两个队列
    // 1. frontal_imus 前置队列
    // 2. imus 主处理队列
    frontal_imus.emplace_back(imu);
    imus.emplace_back(imu);
    // 只要 IMU 数据队列和图像队列都不为空，就进入处理循环
    while (imus.size() > 0 && frames.size() > 0) {
        // 如果 IMU 数据的时间戳小于或等于当前帧的图像时间戳
        if (imus.front().t <= frames.front()->image->t) {
            // 将 IMU 数据添加到当前帧的预积分数据中
            frames.front()->preintegration.data.push_back(imus.front());
            // 从 imus 队列中移除已经使用的 IMU 数据
            imus.pop_front();
        } else {
            // 当前帧已经处理完所有所需的 IMU 数据，将其交由 feature_tracker 进行跟踪处理
            feature_tracker->track_frame(std::move(frames.front()));
            // 从帧队列中移除已处理的帧
            frames.pop_front();
        }
    }
}

// 基于 IMU 数据和最新的状态信息，预测给定时间 t 的相机姿态
Pose XRSLAM::Detail::predict_pose(const double &t) {
    // 定义返回的状态变量，用于存储预测的姿态信息（位置和方向）
    Pose output_pose;
    
    // 从 feature_tracker 中获取最新的状态信息，包括时间、姿态和运动状态
    // 如果状态存在（maybe_state 有值），则进入计算逻辑
    if (auto maybe_state = feature_tracker->get_latest_state()) {
        // 从 maybe_state 中解包出时间、姿态和运动状态
        auto [state_time, state_pose, state_motion] = maybe_state.value();
        // 计算输入输出延迟       
        inspect_debug(input_output_lag, lag) {
            lag = std::min(t - state_time, 5.0);
        }
        // std::cout << "delay: " << t - state_time << std::endl;
        // 清理 frontal_imus 队列中时间早于 state_time 的 IMU 数据，这些数据已经被状态计算使用过。
        while (!frontal_imus.empty() && frontal_imus.front().t <= state_time) {
            frontal_imus.pop_front();
        }
        // 遍历队列
        for (const auto &imu : frontal_imus) {
            // 如果 IMU 数据的时间戳小于等于当前时间 t，则进行状态传播
            if (imu.t <= t) {
                propagate_state(state_time, state_pose, state_motion, imu.t,
                                imu.w, imu.a);
            }
        }
        // 将预测的方向 state_pose.q 转换到目标坐标系
        output_pose.q = state_pose.q * config->output_to_body_rotation();
        // 使用方向旋转偏移量 output_to_body_translation 后加到位置 state_pose.p，得到最终的预测位置
        output_pose.p =
            state_pose.p + state_pose.q * config->output_to_body_translation();
    } else {
        // 如果无法获取最新状态（maybe_state 无值），将输出的姿态设置为初始值（零四元数和零位置）。
        output_pose.q.coeffs().setZero();
        output_pose.p.setZero();
    }
    // 如果启用了视觉定位，并且全局定位状态为 true，则使用 localizer 进行定位
    if (config->visual_localization_enable() &&
        frontend->global_localization_state()) {
        if (frontend->localizer.get()) {
            return frontend->localizer->transform(output_pose);
        }
    }
    // 返回预测的姿态
    return output_pose;
}

// 调用前端模块的 create_virtual_object 方法创建一个新的虚拟对象
size_t XRSLAM::Detail::create_virtual_object() {
    return frontend->create_virtual_object();
}

// 根据 ID 获取虚拟对象的姿态
OutputObject XRSLAM::Detail::get_virtual_object_pose_by_id(size_t id) {

    OutputObject object = frontend->get_virtual_object_pose_by_id(id);
    return object;
}

// 调用前端模块获取系统的当前状态。
SysState XRSLAM::Detail::get_system_state() const {
    return frontend->get_system_state();
}

// 启用全局定位
void XRSLAM::Detail::enable_global_localization() {
    //   std::cout << "VLoc. Mode" << std::endl;
    frontend->set_global_localization_state(true);
}

// 禁用全局定位
void XRSLAM::Detail::disable_global_localization() {
    //   std::cout << "SLAM Mode" << std::endl;
    frontend->set_global_localization_state(false);
}

// 触发前端模块的帧查询操作
void XRSLAM::Detail::query_frame() { frontend->query_frame(); }

// 检查全局定位是否初始化
bool XRSLAM::Detail::global_localization_initialized() {
    return frontend->localizer ? frontend->localizer->is_initialized() : false;
}

// for AR
// 返回最新的状态信息（时间戳和姿态）
std::tuple<double, Pose> XRSLAM::Detail::get_latest_state() const {
    Pose output_pose;
    double timestamp;

    // 如果存在最新状态，从 feature_tracker 提取姿态和运动状态
    if (auto maybe_state = feature_tracker->get_latest_state()) {
        auto [state_time, state_pose, state_motion] = maybe_state.value();
        output_pose.q = state_pose.q * config->output_to_body_rotation();
        output_pose.p =
            state_pose.p + state_pose.q * config->output_to_body_translation();
        timestamp = state_time;
    } else {
        // 如果无法获取最新状态，将输出的姿态设置为初始值（零四元数和零位置）
        output_pose.q.coeffs().setZero();
        output_pose.p.setZero();
        timestamp = 0.0;
    }
    // 如果启用了视觉定位并且全局定位模块处于活动状态，调用 localizer 对姿态进行进一步修正
    if (config->visual_localization_enable() &&
        frontend->global_localization_state()) {
        if (frontend->localizer.get()) {
            output_pose = frontend->localizer->transform(output_pose);
        }
    }

    // 返回一个包含时间戳和姿态（Pose）的元组。
    return {timestamp, output_pose};
}

// 获取最新的姿态（线程安全）
std::tuple<double, Pose> XRSLAM::Detail::get_latest_pose() {
    std::unique_lock<std::mutex> lk(latest_mutex_);
    return {latest_timestamp_, latest_pose_};
}

} // namespace xrslam
