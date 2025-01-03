#include <iostream>
#include <xrslam/common.h>
#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/frontend_worker.h>
#include <xrslam/core/initializer.h>
#include <xrslam/core/sliding_window_tracker.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/localizer/localizer.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>

namespace xrslam {

// 构造函数：初始化 FrontendWorker
FrontendWorker::FrontendWorker(XRSLAM::Detail *detail,
                               std::shared_ptr<Config> config)
    : detail(detail), config(config) {
    // 创建一个 Initializer 对象，用于执行 SLAM 的初始化任务
    initializer = std::make_unique<Initializer>(config);

    // 将 latest_state 设置为空初始值
    latest_state = {{}, nil(), {}, {}};
}

// 析构函数：销毁 FrontendWorker
FrontendWorker::~FrontendWorker() = default;

// 判断任务队列是否为空
bool FrontendWorker::empty() const { return pending_frame_ids.empty(); }

// 前端核心逻辑：处理任务队列中的帧
void FrontendWorker::work(std::unique_lock<std::mutex> &l) {
    // 如果 initializer 存在，说明当前处于初始化阶段
    if (initializer) {

        // 提取挂起帧队列中的第一个帧 ID（pending_frame_id）
        size_t pending_frame_id = pending_frame_ids.front();
        // 将第一个帧 ID 从挂起帧队列中移除
        pending_frame_ids.clear();
        // 解锁互斥锁
        l.unlock();
        // 将 feature_tracker->map 的关键帧同步到初始化器中，以便进行初始化
        synchronized(detail->feature_tracker->map) {
            initializer->mirror_keyframe_map(detail->feature_tracker->map.get(),
                                             pending_frame_id);
        }
        // 调用 initializer->initialize() 执行初始化，并返回一个 sliding_window_tracker 对象
        if ((sliding_window_tracker = initializer->initialize())) {
#if defined(XRSLAM_IOS)
            // 将 feature_tracker 的关键帧地图与滑动窗口跟踪器的地图同步
            synchronized(detail->feature_tracker->keymap) {
                detail->feature_tracker->synchronize_keymap(
                    sliding_window_tracker->map.get());
            }
#endif
            // 如果启用了视觉定位（visual_localization_enable）且全局定位状态有效
            if (config->visual_localization_enable() &&
                global_localization_state()) {
                // 创建一个 Localizer，并将其绑定到虚拟对象管理器    
                localizer = std::make_unique<Localizer>(config);
                sliding_window_tracker->map->create_virtual_object_manager(
                    localizer.get());
            } else {
                // 否则，直接创建虚拟对象管理器
                sliding_window_tracker->map->create_virtual_object_manager();
            }
            // 将 feature_tracker 的地图同步到滑动窗口跟踪器中
            sliding_window_tracker->feature_tracking_map =
                detail->feature_tracker->map;
            // 锁    
            std::unique_lock lk(latest_state_mutex);
            // 获取滑动窗口跟踪器的最新状态
            auto [t, pose, motion] = sliding_window_tracker->get_latest_state();
            // 更新最新状态
            latest_state = {t, pending_frame_id, pose, motion};
            // 解锁
            lk.unlock();
            // 初始化完成后，释放 initializer，标志初始化阶段结束
            initializer.reset();
        }
        }
        // 如果滑动窗口跟踪器存在，说明当前处于跟踪阶段
        else if (sliding_window_tracker) {
        // 从 pending_frame_ids 队列中取出首个待处理的帧 ID
        size_t pending_frame_id = pending_frame_ids.front();
        // 将其从队列中移除
        pending_frame_ids.pop_front();
        // 解锁锁 l
        l.unlock();
        // 确保在多线程环境中对地图的操作是安全的
        synchronized(detail->feature_tracker->map) {
            // 将 feature_tracker 的地图镜像到滑动窗口跟踪器中
            sliding_window_tracker->mirror_frame(
                detail->feature_tracker->map.get(), pending_frame_id);
        }
        // 调用滑动窗口跟踪器的 track() 函数进行跟踪
        if (sliding_window_tracker->track()) {
#if defined(XRSLAM_IOS)
            // 将 feature_tracker 的关键帧地图与滑动窗口跟踪器的地图同步
            synchronized(detail->feature_tracker->keymap) {
                detail->feature_tracker->synchronize_keymap(
                    sliding_window_tracker->map.get());
            }
#endif      
            // 获取滑动窗口跟踪器的最新状态
            std::unique_lock lk(latest_state_mutex);
            auto [t, pose, motion] = sliding_window_tracker->get_latest_state();
            // 更新最新状态
            latest_state = {t, pending_frame_id, pose, motion};
            lk.unlock();
        } else {
            // 如果跟踪失败，则重新初始化
            std::unique_lock lk(latest_state_mutex);
            latest_state = {{}, nil(), {}, {}};
            lk.unlock();
            // 重新创建初始化器
            initializer = std::make_unique<Initializer>(config);
            // 放滑动窗口跟踪器：将 sliding_window_tracker 指针重置为空，释放内存
            sliding_window_tracker.reset();
        }
    }
}

void FrontendWorker::issue_frame(Frame *frame) {
    // 调用 lock() 获取互斥锁 l，确保对共享资源的安全访问
    auto l = lock();
    // 记录帧 ID到 pending_frame_ids 队列中
    pending_frame_ids.push_back(frame->id());
    // 调用 resume() 函数，传入 l 作为参数
    resume(l);
}

// 获取最新状态
std::tuple<double, size_t, PoseState, MotionState>
FrontendWorker::get_latest_state() const {
    std::unique_lock lk(latest_state_mutex);
    return latest_state;
}

// 创造虚拟对象
size_t FrontendWorker::create_virtual_object() {
    auto l = lock();
    // 如果滑动窗口跟踪器存在，则调用其 map 的 create_virtual_object() 函数创建虚拟对象
    if (sliding_window_tracker) {
        return sliding_window_tracker->map->create_virtual_object();
    } else {
        return nil();
    }
    l.unlock();
}

// 根据提供的 id 获取虚拟对象的姿态
OutputObject FrontendWorker::get_virtual_object_pose_by_id(size_t id) {
    auto l = lock();
    // 如果滑动窗口跟踪器存在，则调用其 map 的 get_virtual_object_pose_by_id() 函数获取虚拟对象的姿态
    if (sliding_window_tracker) {
        return sliding_window_tracker->map->get_virtual_object_pose_by_id(id);
    } else {
        // 如果 sliding_window_tracker 无效，返回默认值，其中姿态为单位四元数，位置是非常大的值（如 {1000.0, 1000.0, 1000.0}）
        return {{0.0, 0.0, 0.0, 1.0}, {1000.0, 1000.0, 1000.0}, 1};
    }
    l.unlock();
}
// 返回当前系统状态
SysState FrontendWorker::get_system_state() const {
    // 如果初始化器存在，说明当前处于初始化阶段
    if (initializer) {
        return SysState::SYS_INITIALIZING;
        // sliding_window_tracker 存在，则系统正在跟踪
    } else if (sliding_window_tracker) {
        return SysState::SYS_TRACKING;
    }
    // 否则，系统处于未知状态
    return SysState::SYS_UNKNOWN;
}

// 调用 localizer 的 query_frame 方法。
void FrontendWorker::query_frame() {
    if (localizer)
        localizer->query_frame();
}

// 获取当前是否处于全局定位状态
bool FrontendWorker::global_localization_state() const {
    return global_localization_flag;
}

// 设置全局定位的状态标志。
void FrontendWorker::set_global_localization_state(bool state) {
    global_localization_flag = state;
}

} // namespace xrslam
