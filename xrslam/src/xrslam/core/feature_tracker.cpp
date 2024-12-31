#include <xrslam/common.h>
#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/frontend_worker.h>
#include <xrslam/estimation/solver.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/inspection.h>
#include <xrslam/localizer/localizer.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>
#include <xrslam/utility/unique_timer.h>
namespace xrslam {

// FeatureTracker 特征跟踪 构造函数 实现 
FeatureTracker::FeatureTracker(XRSLAM::Detail *detail,
                               std::shared_ptr<Config> config)
    : detail(detail), config(config) { // 目的: 避免先默认初始化 detail 成员变量，再在构造函数体内赋值，提升效率
    // 初始化 map 和 keymap (全局地图和关键帧地图)
    map = std::make_unique<Map>();
    keymap = std::make_unique<Map>();
}

// FeatureTracker 特征跟踪 析构函数 实现
FeatureTracker::~FeatureTracker() = default;

//
void FeatureTracker::work(std::unique_lock<std::mutex> &l) {
    // make_timer 定义了一个定时器 ft_timer，用于测量特征跟踪的时间开销，并计算平均耗时
    auto ft_timer = make_timer([](double t) {
        inspect(feature_tracker_time, time) {
            static double avg_time = 0;
            static double avg_count = 0;
            avg_time = (avg_time * avg_count + t) / (avg_count + 1);
            avg_count += 1.0;
            time = avg_time;
        }
    });

    // 移动语义将队首的帧赋值给局部变量 frame，避免深拷贝，提升效率。
    std::unique_ptr<Frame> frame = std::move(frames.front());
    // 移除队首的帧
    frames.pop_front();
    // frames 解锁 (解锁后，其他线程可以继续向 frames 中添加帧)
    l.unlock();

    // 预处理，包括直方图均衡化和建立金字塔
    frame->image->preprocess(config->feature_tracker_clahe_clip_limit(),
                             config->feature_tracker_clahe_width(),
                             config->feature_tracker_clahe_height());
    // 获取最新的时间戳、最新的帧 ID、最新的位姿和最新的运动
    auto [latest_optimized_time, latest_optimized_frame_id,
          latest_optimized_pose, latest_optimized_motion] =
        detail->frontend->get_latest_state();
    // 系统是否已经完成初始化   
    // 如果 latest_optimized_frame_id 不等于 nil()，说明系统已经初始化完成。
    bool is_initialized = latest_optimized_frame_id != nil();
    // 表示当前帧是否应该被滑动窗口跟踪算法标记。
    // 如果系统尚未初始化，则直接标记当前帧。
    // 如果系统已经初始化，检查当前帧的 ID 是否满足滑动窗口跟踪的频率条件。
    bool slidind_window_frame_tag =
        !is_initialized ||
        frame->id() % config->sliding_window_tracker_frequent() == 0;

    // synchronized(map) 是一种 线程同步 机制，用于确保在多线程环境中对 map 对象的访问是 线程安全 的。
    // 它的作用是使当前代码块成为 互斥区域，在同一时间内只能有一个线程进入和操作 map 对象，从而避免 竞态条件（Race Condition）和数据不一致的问题。
    synchronized(map) {
        // 如果地图中已经存在当前帧，进行操作
        if (map->frame_num() > 0) {
            // 系统是否已经初始化
            if (is_initialized) {
                // 获取最新的优化帧的索引
                size_t latest_optimized_frame_index =
                    map->frame_index_by_id(latest_optimized_frame_id);
                // 如果最新的优化帧存在 
                if (latest_optimized_frame_index != nil()) {
                    // 如果找到，更新最新优化帧的 pose 和 motion 状态。
                    Frame *latest_optimized_frame =
                        map->get_frame(latest_optimized_frame_index);
                    // 最新的优化帧的位姿
                    latest_optimized_frame->pose = latest_optimized_pose;
                    // 最新优化帧的速度，陀螺仪偏置，加速度计偏置
                    latest_optimized_frame->motion = latest_optimized_motion;

                    // 并对从该帧之后的所有帧，重新进行 IMU 预积分（integrate）和运动预测（predict）
                    for (size_t j = latest_optimized_frame_index + 1;
                         j < map->frame_num(); ++j) {
                        Frame *frame_i = map->get_frame(j - 1);
                        Frame *frame_j = map->get_frame(j);
                        // 重新预积分
                        frame_j->preintegration.integrate(
                            frame_j->image->t, frame_i->motion.bg,
                            frame_i->motion.ba, false, false);
                        // 预积分算速度，旋转，位置    
                        frame_j->preintegration.predict(frame_i, frame_j);
                    }
                } else {
                    // TODO: unfortunately the frame has slided out, which means
                    // we are lost...
                    // 如果未找到（帧已滑出窗口），记录警告日志，
                    // 重置 latest_state，表明滑动窗口未能及时追踪最新优化结果。
                    log_warning("SWT cannot catch up.");
                    std::unique_lock lk(latest_pose_mutex);
                    // 预积分状态重置
                    latest_state.reset();
                }
            }
            
            // 调用 map->get_frame() 获取存储在地图对象中的最后一帧。
            Frame *last_frame = map->get_frame(map->frame_num() - 1);
            // 如果上一帧的预积分数据中有 IMU 数据，则继续处理。
            if (!last_frame->preintegration.data.empty()) {
                // 如果当前帧的预积分数据为空，或者 IMU 数据的时间戳与上一帧图像时间戳的差距过大，则需要补充数据。
                if (frame->preintegration.data.empty() ||
                    (frame->preintegration.data.front().t -
                         last_frame->image->t >
                     1.0e-5)) {
                    // 从上一帧的预积分数据中获取最新的 IMU 数据    
                    ImuData imu = last_frame->preintegration.data.back();
                    // 将 IMU 数据的时间戳设置为上一帧图像的时间戳
                    imu.t = last_frame->image->t;
                    // 插入到当前帧的预积分数据的开头
                    frame->preintegration.data.insert(
                        frame->preintegration.data.begin(), imu);
                }
            }
            // 调用 integrate 方法，使用上一帧的运动偏置值 bg（陀螺仪偏置）和 ba（加速度计偏置）对当前帧进行 IMU 预积分。
            frame->preintegration.integrate(
                frame->image->t, last_frame->motion.bg, last_frame->motion.ba,
                false, false);
            // 使用上一帧作为参考帧，在当前帧上进行特征点跟踪
            last_frame->track_keypoints(frame.get(), config.get());
            if (is_initialized) {
                frame->preintegration.predict(last_frame, frame.get());
#if defined(XRSLAM_IOS)
                synchronized(keymap) {
                    attach_latest_frame(frame.get());
                    solve_pnp();
                    Frame *latest_frame =
                        keymap->get_frame(keymap->frame_num() - 1);
                    std::unique_lock lk(latest_pose_mutex);
                    latest_state = {latest_frame->image->t, latest_frame->pose,
                                    latest_frame->motion};
                    lk.unlock();
                    keymap->erase_frame(keymap->frame_num() - 1);
                    if (config->visual_localization_enable() &&
                        detail->frontend->global_localization_state()) {
                        detail->frontend->localizer->query_localization(
                            latest_frame->image, latest_frame->pose);
                        // detail->frontend->localizer->send_pose_message(frame->image->t);
                    }
                }
#else
                std::unique_lock lk(latest_pose_mutex);
                latest_state = {frame->image->t, frame->pose, frame->motion};
                if (config->visual_localization_enable() &&
                    detail->frontend->global_localization_state()) {
                    detail->frontend->localizer->query_localization(
                        frame->image, frame->pose);
                    // detail->frontend->localizer->send_pose_message(frame->image->t);
                }
                lk.unlock();
#endif
            }
            last_frame->image->release_image_buffer();
        }

        if (slidind_window_frame_tag)
            frame->detect_keypoints(config.get());
        map->attach_frame(std::move(frame));

        while (map->frame_num() >
                   (is_initialized
                        ? config->feature_tracker_max_frames()
                        : config->feature_tracker_max_init_frames()) &&
               map->get_frame(0)->id() < latest_optimized_frame_id) {
            map->erase_frame(0);
        }

        inspect_debug(feature_tracker_painter, p) {
            if (p.has_value()) {
                auto painter = std::any_cast<InspectPainter *>(p);
                auto frame = map->get_frame(map->frame_num() - 1);
                painter->set_image(frame->image.get());
                for (size_t i = 0; i < frame->keypoint_num(); ++i) {
                    if (Track *track = frame->get_track(i)) {
                        color3b c = {0, 255, 0};
                        painter->point(apply_k(frame->get_keypoint(i), frame->K)
                                           .cast<int>(),
                                       c, 5);
                    } else {
                        color3b c = {255, 0, 255};
                        painter->point(apply_k(frame->get_keypoint(i), frame->K)
                                           .cast<int>(),
                                       c, 5, 1);
                    }
                }
            }
        }
    }
    if (slidind_window_frame_tag)
        detail->frontend->issue_frame(map->get_frame(map->frame_num() - 1));
}
// 将一帧数据加入到特征跟踪器的工作队列
void FeatureTracker::track_frame(std::unique_ptr<Frame> frame) {
    // 获取互斥锁
    auto l = lock();
    // 将传入的 frame 移动到 frames 队列末尾
    frames.emplace_back(std::move(frame));
    // 唤醒特征跟踪器的工作线程
    resume(l);
}

std::optional<std::tuple<double, PoseState, MotionState>>
FeatureTracker::get_latest_state() const {
    std::unique_lock lk(latest_pose_mutex);
    return latest_state;
}

void FeatureTracker::synchronize_keymap(Map *sliding_window_tracker_map) {

    // clean keymap
    while (keymap->frame_num()) {
        keymap->erase_frame(0);
    }

    // mirror the latest SWT map to keymap
    mirror_map(sliding_window_tracker_map);

    // add last frame (include subframe) in swt map to keymap for track
    // associating
    mirror_lastframe(sliding_window_tracker_map);
}

void FeatureTracker::mirror_map(Map *sliding_window_tracker_map) {

    for (size_t index = 0; index < sliding_window_tracker_map->frame_num();
         ++index) {
        keymap->attach_frame(
            sliding_window_tracker_map->get_frame(index)->clone());
    }

    for (size_t j = 1; j < keymap->frame_num(); ++j) {
        Frame *old_frame_i = sliding_window_tracker_map->get_frame(j - 1);
        Frame *old_frame_j = sliding_window_tracker_map->get_frame(j);
        Frame *new_frame_i = keymap->get_frame(j - 1);
        Frame *new_frame_j = keymap->get_frame(j);
        for (size_t ki = 0; ki < old_frame_i->keypoint_num(); ++ki) {
            if (Track *track = old_frame_i->get_track(ki)) {
                if (size_t kj = track->get_keypoint_index(old_frame_j);
                    kj != nil()) {
                    Track *new_track = new_frame_i->get_track(ki, keymap.get());
                    new_track->add_keypoint(new_frame_j, kj);
                    new_track->landmark = track->landmark;
                    new_track->tag(TT_VALID) = track->tag(TT_VALID);
                    new_track->tag(TT_TRIANGULATED) =
                        track->tag(TT_TRIANGULATED);
                    new_track->tag(TT_FIX_INVD) = true;
                }
            }
        }
    }

    for (size_t index = 0; index < keymap->frame_num(); ++index) {
        Frame *keyframe = keymap->get_frame(index);
        keyframe->tag(FT_KEYFRAME) = true;
        keyframe->tag(FT_FIX_POSE) = true;
        keyframe->tag(FT_FIX_MOTION) = true;
    }
}

void FeatureTracker::mirror_lastframe(Map *sliding_window_tracker_map) {

    Frame *last_keyframe_i = keymap->get_frame(keymap->frame_num() - 1);
    Frame *last_keyframe_j = sliding_window_tracker_map->get_frame(
        sliding_window_tracker_map->frame_num() - 1);

    if (last_keyframe_j->subframes.empty()) // no subframes means this keyframe
                                            // has been existed in FT map
        return;

    Frame *last_subframe = last_keyframe_j->subframes.back().get();

    keymap->attach_frame(last_subframe->clone());

    Frame *new_keyframe = keymap->get_frame(keymap->frame_num() - 1);

    for (size_t ki = 0; ki < last_keyframe_j->keypoint_num(); ++ki) {
        if (Track *track = last_keyframe_j->get_track(ki)) {
            if (size_t kj = track->get_keypoint_index(last_subframe);
                kj != nil()) {
                last_keyframe_i->get_track(ki, keymap.get())
                    ->add_keypoint(new_keyframe, kj);
            }
        }
    }

    new_keyframe->tag(FT_KEYFRAME) = false;
    new_keyframe->tag(FT_FIX_POSE) = false;
    new_keyframe->tag(FT_FIX_MOTION) = false;
}

void FeatureTracker::attach_latest_frame(Frame *frame) {

    Frame *new_last_frame_i = keymap->get_frame(keymap->frame_num() - 1);
    size_t last_frame_index = map->frame_index_by_id(new_last_frame_i->id());

    size_t frame_index_i = map->frame_index_by_id(
        keymap->get_frame(keymap->frame_num() - 1)->id());
    size_t frame_index_j = map->frame_num() - 1;

    keymap->attach_frame(frame->clone());
    Frame *new_last_frame_j = keymap->get_frame(keymap->frame_num() - 1);

    if (last_frame_index != nil()) {
        Frame *old_last_frame_i = map->get_frame(last_frame_index);
        Frame *old_last_frame_j = frame;
        for (size_t ki = 0; ki < old_last_frame_i->keypoint_num(); ++ki) {
            if (Track *track = old_last_frame_i->get_track(ki)) {
                if (size_t kj = track->get_keypoint_index(old_last_frame_j);
                    kj != nil()) {
                    Track *track =
                        new_last_frame_i->get_track(ki, keymap.get());
                    track->add_keypoint(new_last_frame_j, kj);
                }
            }
        }
        new_last_frame_j->tag(FT_KEYFRAME) = false;
        new_last_frame_j->tag(FT_FIX_POSE) = false;
    } else {
        std::cout << "error: cannot find last frame id in FT map" << std::endl;
    }
}

void FeatureTracker::solve_pnp() {

    Frame *latest_frame = keymap->get_frame(keymap->frame_num() - 1);

    auto solver = Solver::create();

    solver->add_frame_states(latest_frame);

    for (size_t j = 0; j < latest_frame->keypoint_num(); ++j) {
        if (Track *track = latest_frame->get_track(j)) {
            if (track->all_tagged(TT_VALID, TT_TRIANGULATED)) {
                solver->put_factor(Solver::create_reprojection_prior_factor(
                    latest_frame, track));
            }
        }
    }
}

} // namespace xrslam
