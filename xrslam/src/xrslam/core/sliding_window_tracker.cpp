#include <xrslam/common.h>
#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/frontend_worker.h>
#include <xrslam/core/sliding_window_tracker.h>
#include <xrslam/estimation/solver.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/geometry/pnp.h>
#include <xrslam/inspection.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>
#include <xrslam/utility/unique_timer.h>

namespace xrslam {

// 构造函数，初始化滑动窗口跟踪器
SlidingWindowTracker::SlidingWindowTracker(std::unique_ptr<Map> keyframe_map,
                                           std::shared_ptr<Config> config)
    : map(std::move(keyframe_map)), config(config) {
    // 对所有相邻帧进行 IMU 预积分
    for (size_t j = 1; j < map->frame_num(); ++j) {
        Frame *frame_i = map->get_frame(j - 1);
        Frame *frame_j = map->get_frame(j);
        frame_j->preintegration.integrate(frame_j->image->t, frame_i->motion.bg,
                                          frame_i->motion.ba, true, true);
    }
}

SlidingWindowTracker::~SlidingWindowTracker() = default;

// 特定帧从 特征跟踪地图（feature_tracking_map） 同步到 滑动窗口跟踪器（SlidingWindowTracker） 的功能
void SlidingWindowTracker::mirror_frame(Map *feature_tracking_map,
                                        size_t frame_id) {
    //  获取当前滑动窗口跟踪器中最后一个关键帧 keyframe                                       
    Frame *keyframe = map->get_frame(map->frame_num() - 1);
    //  如果最后一个关键帧有子帧，则获取最后一个子帧 new_frame_i
    Frame *new_frame_i = keyframe;
    if (!keyframe->subframes.empty()) {
        new_frame_i = keyframe->subframes.back().get();
    }

    // 获取 new_frame_i 和 frame_id 对应帧在 feature_tracking_map 中的索引
    size_t frame_index_i =
        feature_tracking_map->frame_index_by_id(new_frame_i->id());
    size_t frame_index_j = feature_tracking_map->frame_index_by_id(frame_id);

    // 如果索引无效（nil()），则直接返回，表示帧无法找到
    if (frame_index_i == nil() || frame_index_j == nil())
        return;
    // 从特征跟踪帧得到 old_frame_i 和 old_frame_j
    // old_frame_i 滑动窗口跟踪器中最后一个关键帧
    // old_frame_j 特征跟踪地图中 frame_id 对应的帧
    Frame *old_frame_i = feature_tracking_map->get_frame(frame_index_i);
    Frame *old_frame_j = feature_tracking_map->get_frame(frame_index_j);
    // 克隆目标帧 old_frame_j
    std::unique_ptr<Frame> curr_frame = std::move(old_frame_j->clone());
    // 特征跟踪地图预积分
    std::vector<ImuData> &new_data = curr_frame->preintegration.data;
    // 遍历 frame_index_i 和 frame_index_j 之间的帧，将它们的 IMU 数据插入到 curr_frame 的预积分数据
    for (size_t index = frame_index_j - 1; index > frame_index_i; --index) {
        std::vector<ImuData> old_data =
            feature_tracking_map->get_frame(index)->preintegration.data;
        new_data.insert(new_data.begin(), old_data.begin(), old_data.end());
    }
    // 将 curr_frame 插入到滑动窗口跟踪器中
    map->attach_frame(curr_frame->clone());
    // 获取新附加帧的指针 new_frame_j
    Frame *new_frame_j = map->get_frame(map->frame_num() - 1);
    // 遍历 old_frame_i (滑动窗口跟踪器中最后一个关键帧) 的所有关键点
    for (size_t ki = 0; ki < old_frame_i->keypoint_num(); ++ki) {
        // 检查每个关键点是否有轨迹
        if (Track *track = old_frame_i->get_track(ki)) {
            // 如果有轨迹且在 old_frame_j 中存在对应的关键点索引 kj
            if (size_t kj = track->get_keypoint_index(old_frame_j);
                kj != nil()) {
                // 获取 new_frame_i(滑动窗口跟踪器最后一个关键帧) 的对应轨迹 new_track    
                Track *new_track = new_frame_i->get_track(ki, map.get());
                // 将 new_frame_j (特征跟踪地图中 frame_id 对应的帧) 的关键点 kj 添加到 new_track 轨迹中
                new_track->add_keypoint(new_frame_j, kj);
                // 更新 TT_TRASH 标记：如果新轨迹不为静态轨迹（TT_STATIC），则标记为垃圾轨迹。
                track->tag(TT_TRASH) =
                    new_track->tag(TT_TRASH) && !new_track->tag(TT_STATIC);
            }
        }
    }
    // 清理地图中标记为垃圾且非静态的轨迹
    map->prune_tracks([](const Track *track) {
        return track->tag(TT_TRASH) && !track->tag(TT_STATIC);
    });
    // 更新新帧的运动状态和预积分
    new_frame_j->preintegration.integrate(new_frame_j->image->t,
                                          new_frame_i->motion.bg,
                                          new_frame_i->motion.ba, true, true);
    new_frame_j->preintegration.predict(new_frame_i, new_frame_j);
}

// 滑动窗口跟踪核心函数 如状态更新、局部化新帧、关键帧管理、地标跟踪、窗口优化、以及调试信息输出等
bool SlidingWindowTracker::track() {
    // 检查配置 config 中是否启用了 PARSAC 标志
    if (config->parsac_flag()) {
        // 判断当前状态是否有效
        if (judge_track_status()) {
            // 更新当前状态
            update_track_status();
        }
    }
    // 此函数处理当前帧的局部化操作，将当前帧映射到已知的地图中
    localize_newframe();
    // 检查是否需要管理关键帧,如果需要，则执行关键帧管理操作
    if (manage_keyframe()) {
        // 在关键帧中跟踪地标
        track_landmark();
        // 对滑动窗口中的帧进行优化
        refine_window();
        // 执行窗口滑动操作，移除窗口中较旧的帧，保持窗口大小适当
        slide_window();
    } else {
        // 对当前的子窗口进行优化操作
        refine_subwindow();
    }
    // 输出调试信息
    inspect_debug(sliding_window_landmarks, landmarks) {
        // 获取地图中的所有轨迹点，并存储在 landmarks 向量中
        std::vector<Landmark> points;
        points.reserve(map->track_num());
        // 遍历地图中的所有点的轨迹
        for (size_t i = 0; i < map->track_num(); ++i) {
            if (Track *track = map->get_track(i)) {
                // 检查轨迹点是否有效
                if (track->tag(TT_VALID)) {
                    // 将轨迹点添加到 points 向量中
                    Landmark point;
                    // 获取轨迹点的三维位置
                    point.p = track->get_landmark_point();
                    // 检查轨迹点是否已被三角化
                    point.triangulated = track->tag(TT_TRIANGULATED);
                    // 检查轨迹点是否为静态点
                    points.push_back(point);
                }
            }
        }
        // 将 points 向量移动到 landmarks 变量中
        landmarks = std::move(points);
    }
    // 陀螺因偏差（bg） 
    inspect_debug(sliding_window_current_bg, bg) {
        bg = std::get<2>(get_latest_state()).bg;
    }
    // 加速度偏差（ba）
    inspect_debug(sliding_window_current_ba, ba) {
        ba = std::get<2>(get_latest_state()).ba;
    }

    return true;
}

// 滑动窗口跟踪器中对新加入帧进行局部定位的功能
void SlidingWindowTracker::localize_newframe() {
    // 创建solver求解器
    auto solver = Solver::create();
    // frame_i: 滑动窗口中的倒数第二帧
    Frame *frame_i = map->get_frame(map->frame_num() - 2);
    // 如果帧 frame_i 包含子帧（subframes），则使用最新的子帧
    if (!frame_i->subframes.empty()) {
        frame_i = frame_i->subframes.back().get();
    }
    // frame_j: 滑动窗口中的最后一帧
    Frame *frame_j = map->get_frame(map->frame_num() - 1);
    // 将新帧 frame_j 的状态（位姿、速度、IMU 偏置等）添加到求解器中作为优化变量
    solver->add_frame_states(frame_j);
    // 将新帧frame_i 和frame_j 之间的预积分信息添加到求解器中
    solver->put_factor(Solver::create_preintegration_prior_factor(
        frame_i, frame_j, frame_j->preintegration));
    // 遍历新帧中的关键点：
    // TT_VALID: 轨迹点有效。
    // TT_TRIANGULATED: 轨迹点已被三角化，拥有三维位置。
    // TT_STATIC: 轨迹点被认为是静态的，排除动态物体的影响。
    for (size_t k = 0; k < frame_j->keypoint_num(); ++k) {
        if (Track *track = frame_j->get_track(k)) {
            if (track->all_tagged(TT_VALID, TT_TRIANGULATED, TT_STATIC)) {
                
                // 调用 create_reprojection_prior_factor 创建重投影因子，将关键点的图像观测与地图点的三维位置约束加入求解器。
                solver->put_factor(
                    Solver::create_reprojection_prior_factor(frame_j, track));
            }
        }
    }
    // 调用求解器，执行非线性优化
    solver->solve();
}

// 滑动窗口跟踪器中对关键帧进行管理
bool SlidingWindowTracker::manage_keyframe() {
    // 获取滑动窗口中的倒数第二帧和最后一帧
    Frame *keyframe_i = map->get_frame(map->frame_num() - 2);
    Frame *newframe_j = map->get_frame(map->frame_num() - 1);
    // keyframe_i(倒数第二帧)的子帧不为空
    if (!keyframe_i->subframes.empty()) {
        // 倒数第二帧的子帧最后一个是无平移帧
        if (keyframe_i->subframes.back()->tag(FT_NO_TRANSLATION)) {
            // 如果最新帧也是无平移帧，无需任何操作
            if (newframe_j->tag(FT_NO_TRANSLATION)) {
                // [T]...........<-[R]
                //  +-[R]-[R]-[R]
                // ==>
                // [T]
                //  +-[R-R]-[R]-[R]
            } else {
                // [T]...........<-[T]
                //  +-[R]-[R]-[R]
                // ==>
                // [T]........[R]-[T]
                //  +-[R]-[R]
                // 如果最新帧不是无平移帧，将子帧升级为关键帧
                keyframe_i->subframes.back()->tag(FT_KEYFRAME) = true;
                // 插入滑动窗口
                map->attach_frame(std::move(keyframe_i->subframes.back()),
                                  map->frame_num() - 1);
                // 子帧移除队列                  
                keyframe_i->subframes.pop_back();
                // 新帧标记为关键帧
                newframe_j->tag(FT_KEYFRAME) = true;
                return true;
            }
        } else { // 倒数第二帧的子帧最后一个是平移帧
            // 如果最新帧是无平移帧
            if (newframe_j->tag(FT_NO_TRANSLATION)) {
                // [T]...........<-[R]
                //  +-[T]-[T]-[T]
                // ==>
                // [T]........[T]
                //  +-[T]-[T]  +-[R]
                // 倒数第二帧的子帧的最后一帧 frame_lifted
                std::unique_ptr<Frame> frame_lifted =
                    std::move(keyframe_i->subframes.back());
                // 移除 倒数第二帧的子帧的最后一帧   
                keyframe_i->subframes.pop_back();
                // 将该帧设为关键帧
                frame_lifted->tag(FT_KEYFRAME) = true;
                // 新帧作为它的子帧
                frame_lifted->subframes.emplace_back(
                    map->detach_frame(map->frame_num() - 1));
                // 将新关键帧插入到滑动窗口    
                map->attach_frame(std::move(frame_lifted));
                return true;
            }
            // 如果最新帧是平移帧
            else {
                // 如果子帧数量超过配置的阈值
                if (keyframe_i->subframes.size() >=
                    config->sliding_window_subframe_size()) {
                    // [T]...........<-[T]
                    //  +-[T]-[T]-[T]
                    // ==>
                    // [T]............[T]
                    //  +-[T]-[T]-[T]
                    // 将新帧直接标记为关键帧 
                    newframe_j->tag(FT_KEYFRAME) = true;
                    return true;
                }
            }
        }
    }
    // 统计新地图点的数量
    size_t mapped_landmark_count = 0;
    // 统计新帧中所有被标记为有效、三角化、静态的地图点的数量
    for (size_t k = 0; k < newframe_j->keypoint_num(); ++k) {
        if (Track *track = newframe_j->get_track(k)) {
            if (track->all_tagged(TT_VALID, TT_TRIANGULATED, TT_STATIC)) {
                mapped_landmark_count++;
            }
        }
    }
    // 映射的有效地图点数量少于配置的阈值，则将新帧标记为关键帧
    bool is_keyframe = mapped_landmark_count <
                       config->sliding_window_force_keyframe_landmarks();
    
#if defined(XRSLAM_IOS)
    // 新帧不是无平移帧
    is_keyframe = is_keyframe || !newframe_j->tag(FT_NO_TRANSLATION);
#endif
    // 如果是关键帧，标记 newframe_j 为关键帧，返回 true。
    if (is_keyframe) {
        newframe_j->tag(FT_KEYFRAME) = true;
        return true;
    }
    // 将 newframe_j 作为 keyframe_i 的子帧，返回 false 
    else {
        keyframe_i->subframes.emplace_back(
            map->detach_frame(map->frame_num() - 1));
        return false;
    }
}

// 对地图点进行三角化
void SlidingWindowTracker::track_landmark() {
    // 获取最新帧
    Frame *newframe_j = map->get_frame(map->frame_num() - 1);
    // 遍历关键点
    for (size_t k = 0; k < newframe_j->keypoint_num(); ++k) {
        // 获取关键点对应的轨迹
        if (Track *track = newframe_j->get_track(k)) {
            // 如果该点还没有被三角化
            if (!track->tag(TT_TRIANGULATED)) {
                // 使用 track->triangulate() 进行三角化
                if (auto p = track->triangulate()) {
                    // 如果成功，返回一个三维点 p，将其设置为该轨迹的地图点位置
                    track->set_landmark_point(p.value());
                    // 标记该轨迹为已三角化、有效、静态
                    track->tag(TT_TRIANGULATED) = true;
                    track->tag(TT_VALID) = true;
                    track->tag(TT_STATIC) = true;
                } else {
                    // outlier
                    // 三角化失败（外点）
                    // 将轨迹的反深度（inv_depth）设置为 -1.0，表示无效
                    track->landmark.inv_depth = -1.0;
                    // 标记 TT_TRIANGULATED 为 false，表示三角化失败
                    track->tag(TT_TRIANGULATED) = false;
                    // 标记 TT_VALID 为 false，表示该轨迹无效
                    track->tag(TT_VALID) = false;
                }
            }
        }
    }
}

// 滑动窗口中的窗口细化
void SlidingWindowTracker::refine_window() {
    // 获取滑动窗口中的倒数第二帧（keyframe_i）和最后一帧（keyframe_j）
    Frame *keyframe_i = map->get_frame(map->frame_num() - 2);
    Frame *keyframe_j = map->get_frame(map->frame_num() - 1);
    // 创建一个求解器
    auto solver = Solver::create();
    // 如果当前地图中没有边缘化因子，则创建一个
    if (!map->marginalization_factor) {
        map->marginalization_factor =
            Solver::create_marginalization_factor(map.get());
    }
    // 将所有滑动窗口中的帧状态（包括位置、姿态等）加入优化器
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        solver->add_frame_states(frame);
    }

    // 添加轨迹状态
    // 1.已添加过的跟踪
    std::unordered_set<Track *> visited_tracks;
    // 遍历所有帧的关键点，得到关键点的跟踪
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            Track *track = frame->get_track(j);
            // 如果 点被跟踪到，且没有被添加过，且是有效的，且是静态的，则添加
            if (!track)
                continue;
            if (visited_tracks.count(track) > 0)
                continue;
            visited_tracks.insert(track);
            if (!track->tag(TT_VALID))
                continue;
            if (!track->tag(TT_STATIC))
                continue;
            // 点跟踪的第一个帧是关键帧    
            if (!track->first_frame()->tag(FT_KEYFRAME))
                continue;
            // 点跟踪添加进优化器
            solver->add_track_states(track);
        }
    }
    // 将边缘化因子加入优化器，用于约束窗口内的关键帧状态
    solver->add_factor(map->marginalization_factor.get());
    // 遍历所有帧的关键点，得到关键点的跟踪
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            Track *track = frame->get_track(j);
            // 如果 点被跟踪到，且没有被添加过，且是有效的，且是静态的，则添加
            if (!track)
                continue;
            if (!track->all_tagged(TT_VALID, TT_TRIANGULATED, TT_STATIC))
                continue;
            // 点跟踪的第一个帧是关键帧    
            if (!track->first_frame()->tag(FT_KEYFRAME))
                continue;
            // 当前帧不是轨迹的起始帧    
            if (frame == track->first_frame())
                continue;
            // 添加重投影误差因子    
            solver->add_factor(frame->reprojection_error_factors[j].get());
        }
    }
    // 遍历所有帧，得到IMU预积分因子
    for (size_t j = 1; j < map->frame_num(); ++j) {
        // 获取滑动窗口中的两帧 frame_i（前一帧）和 frame_j（当前帧）
        Frame *frame_i = map->get_frame(j - 1);
        Frame *frame_j = map->get_frame(j);
        // 将当前帧的预积分数据 preintegration 复制到 keyframe_preintegration    
        frame_j->keyframe_preintegration = frame_j->preintegration;
        // 如果当前帧有子帧
        if (!frame_i->subframes.empty()) {
            // 收集这些子帧的 IMU 数据
            std::vector<ImuData> imu_data;
            for (size_t k = 0; k < frame_i->subframes.size(); ++k) {
                // 获取子帧的 IMU 数据
                auto &sub_imu_data = frame_i->subframes[k]->preintegration.data;
                // 将子帧的 IMU 数据添加到当前帧的 IMU 数据中
                imu_data.insert(imu_data.end(), sub_imu_data.begin(),
                                sub_imu_data.end());
            }
            // 收集到的 IMU 数据整合到 frame_j->keyframe_preintegration.data 中
            frame_j->keyframe_preintegration.data.insert(
                frame_j->keyframe_preintegration.data.begin(), imu_data.begin(),
                imu_data.end());
        }
        // 使用 IMU 数据进行预积分计算
        if (frame_j->keyframe_preintegration.integrate(
                frame_j->image->t, frame_i->motion.bg, frame_i->motion.ba, true,
                true)) {
            // 将关键帧之间的预积分结果作为误差因子加入优化求解器（solver)        
            solver->put_factor(Solver::create_preintegration_error_factor(
                frame_i, frame_j, frame_j->keyframe_preintegration));
        }
    }
    // 运行优化器，求解当前窗口内帧状态和轨迹
    solver->solve();
    // 遍历所有的点跟踪
    for (size_t k = 0; k < map->track_num(); ++k) {
        Track *track = map->get_track(k);
        // 如果点轨迹被三角化
        if (track->tag(TT_TRIANGULATED)) {
            // is_valid：表示当前轨迹点是否有效
            bool is_valid = true;
            // x：三角化得到的点跟踪的三维位置
            auto x = track->get_landmark_point();
            // rpe 和 rpe_count：分别记录重投影误差的累计值和关键帧数量
            double rpe = 0.0;
            double rpe_count = 0.0;
            // 遍历关键帧并计算重投影误差
            for (const auto &[frame, keypoint_index] : track->keypoint_map()) {
                // 如果当前帧不是关键帧（FT_KEYFRAME），跳过
                if (!frame->tag(FT_KEYFRAME))
                    continue;
                //  相机轨迹    
                PoseState pose = frame->get_pose(frame->camera);
                // 轨迹点 x 在当前关键帧相机坐标系下的三维位置
                vector<3> y = pose.q.conjugate() * (x - pose.p);
                
                // 如果 y.z()（轨迹点的深度）小于一个非常小的值（1.0e-3）或者深度过大（50），认为该点无效
                if (y.z() <= 1.0e-3 || y.z() > 50) { // todo
                    is_valid = false;
                    break;
                }
                // 将轨迹点 y 投影到像素坐标系，并与实际关键点的像素坐标比较，计算重投影误差
                // 累积误差 rpe 和观测数量 rpe_count
                rpe += (apply_k(y, frame->K) -
                        apply_k(frame->get_keypoint(keypoint_index), frame->K))
                           .norm();
                rpe_count += 1.0;
            }
            // 深度和轨迹点的位置满足约束 且 平均重投影误差小于阈值  3.0
            is_valid = is_valid && (rpe / std::max(rpe_count, 1.0) < 3.0);
            // 点跟踪标记
            track->tag(TT_VALID) = is_valid;
        } else {
            // 如果未三角化，跳过验证。
            track->landmark.inv_depth = -1.0;
        }
    }
    // 遍历所有点跟踪，检查其是否为无效点
    for (size_t k = 0; k < map->track_num(); ++k) {
        Track *track = map->get_track(k);
        // 如果无效，将其标记为垃圾点
        if (!track->tag(TT_VALID))
            track->tag(TT_TRASH) = true;
    }
}

// 窗口滑动函数
void SlidingWindowTracker::slide_window() {
    // 检查帧数量是否超过最大窗口大小
    while (map->frame_num() > config->sliding_window_size()) {
        // 获取窗口中的第一帧
        Frame *frame = map->get_frame(0);
        // 遍历当前帧的所有子帧（subframes）
        for (size_t i = 0; i < frame->subframes.size(); ++i) {
            // 将子帧从地图中移除
            map->untrack_frame(frame->subframes[i].get());
        }
        // 边缘化当前帧
        map->marginalize_frame(0);
    }
}

// 主要用于对滑动窗口中的子帧进行优化调整
void SlidingWindowTracker::refine_subwindow() {
    // 获取当前地图中的最后一帧
    Frame *frame = map->get_frame(map->frame_num() - 1);
    // 如果帧中不包含子帧，直接返回
    if (frame->subframes.empty())
        return;
    //  如果当前帧的第一个子帧标记为 FT_NO_TRANSLATION，意味着该帧没有进行平移   
    if (frame->subframes[0]->tag(FT_NO_TRANSLATION)) {
        // 当子帧的数量大于等于 9 时，进入合并 IMU 数据的步骤
        if (frame->subframes.size() >= 9) {
            // 通过遍历子帧
            for (size_t i = frame->subframes.size() / 3; i > 0; --i) {
                // 获取目标子帧 目标帧 (tgt_frame)：tgt_frame 是当前循环中需要合并 IMU 数据的目标子帧
                Frame *tgt_frame = frame->subframes[i * 3 - 1].get();
                // 用于存储将要合并的 IMU 数据
                std::vector<ImuData> imu_data;
                // 前 i 值对应的 3 个子帧（(i - 1) * 3 到 i * 3 - 1 之间的子帧）
                for (size_t j = i * 3 - 1; j > (i - 1) * 3; --j) {
                    // 每一个源子帧（src_frame）
                    Frame *src_frame = frame->subframes[j - 1].get();
                    // 将其 preintegration.data（IMU数据）插入到 imu_data 的开头,插入到开头是为了保持数据的时间顺序。
                    imu_data.insert(imu_data.begin(),
                                    src_frame->preintegration.data.begin(),
                                    src_frame->preintegration.data.end());
                    //  从地图中将该子帧移除               
                    map->untrack_frame(src_frame);
                    // 从当前帧的子帧列表中移除该子帧
                    frame->subframes.erase(frame->subframes.begin() + (j - 1));
                }
                // 将 imu_data（已经合并的IMU数据）插入到目标子帧（tgt_frame）的 preintegration.data 的开头，
                // 合并后的IMU数据将更新目标子帧的预积分数据
                tgt_frame->preintegration.data.insert(
                    tgt_frame->preintegration.data.begin(), imu_data.begin(),
                    imu_data.end());
            }
        }
        // 创建优化求解器并设置固定标签
        auto solver = Solver::create();
        // 设置当前帧的标签，表示它的姿态和运动在优化过程中是固定的，不参与优化。
        frame->tag(FT_FIX_POSE) = true;
        frame->tag(FT_FIX_MOTION) = true;
        // 将当前帧添加到优化求解器中
        solver->add_frame_states(frame);
        // 遍历当前帧的所有子帧
        for (size_t i = 0; i < frame->subframes.size(); ++i) {
            // 获取当前子帧
            Frame *subframe = frame->subframes[i].get();
            // 将子帧添加到优化求解器中
            solver->add_frame_states(subframe);
            // 获取前一个子帧
            Frame *prev_frame =
                (i == 0 ? frame : frame->subframes[i - 1].get());
            // 进行 IMU 数据的预积分    
            subframe->preintegration.integrate(
                subframe->image->t, prev_frame->motion.bg,
                prev_frame->motion.ba, true, true);
            // 将子帧添加到求解器中，加入预积分误差因子    
            solver->put_factor(Solver::create_preintegration_error_factor(
                prev_frame, subframe, subframe->preintegration));
        }
        // 处理最后一个子帧的关键点约束
        // 得到最后一个子帧
        Frame *last_subframe = frame->subframes.back().get();
        // 遍历最后一个子帧中的所有关键点
        for (size_t k = 0; k < last_subframe->keypoint_num(); ++k) {
            // 如果该关键点对应的轨迹存在，并且该轨迹被标记为有效
            if (Track *track = last_subframe->get_track(k)) {
                // 该关键点对应的轨迹, 如果该轨迹被标记为三角化且静态
                if (track->tag(TT_VALID)) {
                    if (track->tag(TT_TRIANGULATED)) {
                        if (track->tag(TT_STATIC))
                            // 添加一个重投影先验因子
                            solver->put_factor(
                                Solver::create_reprojection_prior_factor(
                                    last_subframe, track));
                    } else {
                        // 添加一个旋转先验因子
                        solver->put_factor(Solver::create_rotation_prior_factor(
                            last_subframe, track));
                    }
                }
            }
        }
        // 进行优化求解
        solver->solve();
        // 优化完成后，取消固定标签，允许这些帧在后续的优化过程中自由变化
        frame->tag(FT_FIX_POSE) = false;
        frame->tag(FT_FIX_MOTION) = false;
    } else {
        // 如果该帧正在进行平移
        // 1. 创建求解器
        auto solver = Solver::create();
        // 2. 设置当前帧的 FT_FIX_POSE 和 FT_FIX_MOTION 标签为 true
        frame->tag(FT_FIX_POSE) = true;
        frame->tag(FT_FIX_MOTION) = true;
        // 3. 将当前帧添加到求解器中
        solver->add_frame_states(frame);
        // 4. 遍历当前帧的所有子帧
        for (size_t i = 0; i < frame->subframes.size(); ++i) {
            // 获取当前帧的子帧
            Frame *subframe = frame->subframes[i].get();
            // 将子帧添加到求解器中
            solver->add_frame_states(subframe);
            // 对每个子帧，先确定前一个帧（prev_frame），如果是第一个子帧，则使用当前帧作为前一帧。
            Frame *prev_frame =
                (i == 0 ? frame : frame->subframes[i - 1].get());
            // 对 IMU 数据进行积分    
            subframe->preintegration.integrate(
                subframe->image->t, prev_frame->motion.bg,
                prev_frame->motion.ba, true, true);
            // 将前一帧和当前子帧的预积分误差添加到求解器中，作为优化的约束条件
            solver->put_factor(Solver::create_preintegration_error_factor(
                prev_frame, subframe, subframe->preintegration));
            // 遍历当前子帧中的所有关键点
            for (size_t k = 0; k < subframe->keypoint_num(); ++k) {
                // 该关键点对应的轨迹存在
                if (Track *track = subframe->get_track(k)) {
                    // 对于每个关键点，检查它是否有效且已三角化，并且是否为静态点（TT_VALID, TT_TRIANGULATED, TT_STATIC）
                    if (track->all_tagged(TT_VALID, TT_TRIANGULATED,
                                          TT_STATIC)) {
                        // 如果关键点是来自一个关键帧（FT_KEYFRAME）                    
                        if (track->first_frame()->tag(FT_KEYFRAME)) {
                            // 向求解器中添加重投影先验约束
                            solver->put_factor(
                                Solver::create_reprojection_prior_factor(
                                    subframe, track));
                        // 如果该关键点来自非关键帧，且该关键点的第一个帧 ID 大于当前帧的 ID            
                        } else if (track->first_frame()->id() > frame->id()) {
                            // 将该关键点的重投影误差因素添加到求解器中（reprojection_error_factors）
                            solver->add_factor(
                                frame->reprojection_error_factors[k].get());
                        }
                    }
                }
            }
        }
        // 求解优化问题
        solver->solve();
        // 恢复当前帧的标签，表示当前帧的姿态和运动不再被固定
        frame->tag(FT_FIX_POSE) = false;
        frame->tag(FT_FIX_MOTION) = false;
    }
} // namespace xrslam

// 滑动窗口获取最新状态
std::tuple<double, PoseState, MotionState>
SlidingWindowTracker::get_latest_state() const {
    const Frame *frame = map->get_frame(map->frame_num() - 1);
    // 如果该帧有子帧，则使用最后一个子帧
    if (!frame->subframes.empty()) {
        frame = frame->subframes.back().get();
    }
    return {frame->image->t, frame->pose, frame->motion};
}

// 计算本质矩阵
matrix<3> compute_essential_matrix(matrix<3> &R, vector<3> &t) {
    matrix<3> t_ = matrix<3>::Zero();

    t_(0, 1) = -t(2);
    t_(0, 2) = t(1);
    t_(1, 0) = t(2);
    t_(1, 2) = -t(0);
    t_(2, 0) = -t(1);
    t_(2, 1) = t(0);

    matrix<3> E = t_ * R;
    return E;
}

// 计算对极几何距离 
// F：3x3 的基础矩阵 pt1, pt2：两帧图像中的 2D 点坐标
double compute_epipolar_dist(matrix<3> F, vector<2> &pt1, vector<2> &pt2) {
    // pt1.homogeneous() 将 2D 点 (x, y) 转成同余坐标 (x, y, 1)
    // l = F * pt1.homogeneous() 计算 pt1 在另一帧图像中的极线 l
    vector<3> l = F * pt1.homogeneous();
    // pt2.homogeneous().transpose() * l 计算点 pt2 到极线 l 的代数距离
    // l.segment<2>(0).norm() 表示极线的 (a, b) 部分的范数，用于归一化距离
    // dist 是点到极线的几何距离。若 dist 很小，说明点 pt2 符合由 pt1 确定的对极几何约束。
    double dist =
        std::abs(pt2.homogeneous().transpose() * l) / l.segment<2>(0).norm();
    return dist;
}

// 对于一个特征点 Track，在多关键帧下进行投影误差统计，如果深度或重投影残差不合理，则认为这个点无效
// Track *track：包含一个特征点（Track）在若干帧中的对应关系
// vector<3> &x：3D 空间中某点的位置估计（通常在当前参考坐标系下）
bool SlidingWindowTracker::check_frames_rpe(Track *track, const vector<3> &x) {
    std::vector<matrix<3, 4>> Ps;
    std::vector<vector<3>> ps;

    bool is_valid = true;
    double rpe = 0.0;
    double rpe_count = 0.0;
    // 遍历每个点跟踪中的每个关键帧和索引对
    for (const auto &[frame, keypoint_index] : track->keypoint_map()) {
        // 如果该帧不是关键帧，则跳过
        if (!frame->tag(FT_KEYFRAME))
            continue;
        // 从 frame 中获取相机姿态 pose    
        PoseState pose = frame->get_pose(frame->camera);
        // 将 3D 点 x 转换到 frame 的坐标系下
        vector<3> y = pose.q.conjugate() * (x - pose.p);
        // 如果 y 的 z 分量小于 1e-3 或大于 50，则认为该点无效
        if (y.z() <= 1.0e-3 || y.z() > 50) { // todo
            is_valid = false;
            break;
        }
        // apply_k(y, frame->K) 将 3D 点 y 投影并转换到像素坐标
        // 二者之差的范数表示重投影误差
        rpe += (apply_k(y, frame->K) -
                apply_k(frame->get_keypoint(keypoint_index), frame->K))
                   .norm();
        rpe_count += 1.0;
    }
    // 如果平均误差大于阈值（例如 3.0 像素），或者深度检查不通过，则 is_valid 设置为 false
    is_valid = is_valid && (rpe / std::max(rpe_count, 1.0) < 3.0);

    return is_valid;
}

// 两个帧 frame_i 和 frame_j 之间寻找足够多的 2D-2D 对应点（特征点匹配），
// 并通过 parsac（类似于 RANSAC 的方法）估计本质矩阵 E，同时输出内外点掩码 mask
bool SlidingWindowTracker::filter_parsac_2d2d(
    Frame *frame_i, Frame *frame_j, std::vector<char> &mask,
    std::vector<size_t> &pts_to_index) {

    std::vector<vector<2>> pts1, pts2;
    // 遍历 frame_i 中的所有特征点
    for (size_t ki = 0; ki < frame_i->keypoint_num(); ++ki) {
        // 如果该特征点有对应的 Track
        if (Track *track = frame_i->get_track(ki)) {
            // 如果 Track 中有 frame_j 的对应点
            if (size_t kj = track->get_keypoint_index(frame_j)) {
                if (kj != nil()) {
                    // 将 frame_i 和 frame_j 中对应点的归一化坐标添加到 pts1 和 pts2
                    pts1.push_back(frame_i->get_keypoint(ki).hnormalized());
                    pts2.push_back(frame_j->get_keypoint(kj).hnormalized());
                    // 将 kj 存入 pts_to_index
                    pts_to_index.push_back(kj);
                }
            }
        }
    }
    // 如果匹配点对数量小于 10，则无法计算本质矩阵，返回 false
    if (pts1.size() < 10)
        return false;
    // 调用 find_essential_matrix_parsac 函数计算本质矩阵 E
    matrix<3> E =
        find_essential_matrix_parsac(pts1, pts2, mask, m_th / frame_i->K(0, 0));

    return true;
}

// 根据已有的世界坐标系下两帧（frame_i、frame_j）的位姿、相机外参、IMU 外参，
// 推断这两帧在相机坐标系下的相对旋转 R 和平移 t
void SlidingWindowTracker::predict_RT(Frame *frame_i, Frame *frame_j,
                                      matrix<3> &R, vector<3> &t) {

    auto camera = frame_i->camera;
    auto imu = frame_i->imu;

    matrix<4> Pwc = matrix<4>::Identity();
    matrix<4> PwI = matrix<4>::Identity();
    matrix<4> Pwi = matrix<4>::Identity();
    matrix<4> Pwj = matrix<4>::Identity();
    // Pwc 和 PwI 分别为相机坐标系和 IMU 坐标系在世界坐标系下的位姿
    Pwc.block<3, 3>(0, 0) = camera.q_cs.toRotationMatrix();
    Pwc.block<3, 1>(0, 3) = camera.p_cs;
    PwI.block<3, 3>(0, 0) = imu.q_cs.toRotationMatrix();
    PwI.block<3, 1>(0, 3) = imu.p_cs;
    // Pwi 和 Pwj 分别为 frame_i 和 frame_j 在世界坐标系下的位姿
    Pwi.block<3, 3>(0, 0) = frame_i->pose.q.toRotationMatrix();
    Pwi.block<3, 1>(0, 3) = frame_i->pose.p;
    Pwj.block<3, 3>(0, 0) = frame_j->pose.q.toRotationMatrix();
    Pwj.block<3, 1>(0, 3) = frame_j->pose.p;

    // 帧 j 到帧 i 的世界坐标系变换
    // 计算两帧在 IMU+相机坐标系下的相对变换
    matrix<4> Pji = Pwj.inverse() * Pwi;
    // 计算两帧在相机坐标系下的相对变换
    matrix<4> P = (Pwc.inverse() * PwI * Pji * PwI.inverse() * Pwc);
    // 提取旋转和平移
    R = P.block<3, 3>(0, 0);
    t = P.block(0, 3, 3, 1);
}

// 对当前帧与关键帧之间的特征点进行筛选与评估
bool SlidingWindowTracker::judge_track_status() {

    // 获取当前帧和前一个关键帧
    Frame *curr_frame = map->get_frame(map->frame_num() - 1);
    Frame *keyframe = map->get_frame(map->frame_num() - 2);
    // 将前一个关键帧的子帧（如果有）设为 last_frame
    Frame *last_frame = keyframe;
    if (!keyframe->subframes.empty()) {
        last_frame = keyframe->subframes.back().get();
    }
    // 对当前帧 curr_frame 与关键帧 last_frame 之间的 IMU 数据做预积分
    curr_frame->preintegration.integrate(curr_frame->image->t,
                                         last_frame->motion.bg,
                                         last_frame->motion.ba, true, true);
    // 根据上一帧的偏置（陀螺仪、加速度计）进行状态预测，赋给新帧                                    
    curr_frame->preintegration.predict(last_frame, curr_frame);

    m_P2D.clear();
    m_P3D.clear();
    m_lens.clear();
    // 地图点索引表
    m_indices_map = std::vector<int>(curr_frame->keypoint_num(), -1);
    // 遍历当前帧中的所有特征点
    for (size_t k = 0; k < curr_frame->keypoint_num(); ++k) {
        // 如果该特征点有对应的 Track
        if (Track *track = curr_frame->get_track(k)) {
            // 如果该特征点已被三角化（标签 TT_VALID 与 TT_TRIANGULATED）
            if (track->all_tagged(TT_VALID, TT_TRIANGULATED)) {
                // 获取特征点的归一化坐标和地图点坐标
                const vector<3> &bearing = curr_frame->get_keypoint(k);
                const vector<3> &landmark = track->get_landmark_point();
                // 将当前帧特征点的归一化坐标（bearing.hnormalized()）存入 m_P2D
                m_P2D.push_back(bearing.hnormalized());
                // 将对应的 3D Landmark（世界坐标）存入 m_P3D
                m_P3D.push_back(landmark);
                // 将地图点的生命周期存入 m_lens
                m_lens.push_back(std::max(track->m_life, size_t(0)));
                // 记录特征点 k 对应的 m_P3D 索引
                m_indices_map[k] = m_P3D.size() - 1;
            }
        }
    }
    // 如果收集到的特征点少于 20，则直接返回 false，表示无法进行可靠计算
    if (m_P2D.size() < 20)
        return false;
    // 获取当前帧的位姿
    const PoseState &pose = curr_frame->get_pose(curr_frame->camera);

    // mask：和常见 RANSAC 一样，输出用于区分内点/外点的布尔向量
    std::vector<char> mask;
    // 调用自定义的 find_pnp_matrix_parsac_imu()，基于 Parsac
    // 进行 2D-3D PnP 计算，得到一个相机 pose 矩阵 T_IMU
    matrix<3> Rcw = pose.q.inverse().toRotationMatrix();
    vector<3> tcw = pose.q.inverse() * pose.p * (-1.0);
    matrix<4> T_IMU =
        find_pnp_matrix_parsac_imu(m_P3D, m_P2D, m_lens, Rcw, tcw, 0.20, 1.0,
                                   mask, 1.0 / curr_frame->K(0, 0));

    matrix<3> R;
    vector<3> t;
    // 调用 predict_RT 函数，根据关键帧和当前帧已有的姿态信息，预测二者的相对旋转 R 与平移 t。
    predict_RT(keyframe, curr_frame, R, t);

    // check rpe 
    {
        // 内外点分离
        std::vector<vector<2>> P2D_inliers, P2D_outliers;
        std::vector<vector<3>> P3D_inliers, P3D_outliers;
        // 使用mask 用来分离内点与外点
        for (int i = 0; i < m_P2D.size(); ++i) {
            if (mask[i]) {
                P2D_inliers.push_back(m_P2D[i]);
                P3D_inliers.push_back(m_P3D[i]);
            } else {
                P2D_outliers.push_back(m_P2D[i]);
                P3D_outliers.push_back(m_P3D[i]);
            }
        }
        // 计算内点与外点的投影误差
        std::vector<double> inlier_errs, outlier_errs;
        double inlier_errs_sum = 0, outlier_errs_sum = 0;
        // 计算内点误差
        for (int i = 0; i < P2D_inliers.size(); i++) {
            vector<3> p = pose.q.conjugate() * (P3D_inliers[i] - pose.p);
            double proj_err =
                (apply_k(p, curr_frame->K) -
                 apply_k(P2D_inliers[i].homogeneous(), curr_frame->K))
                    .norm();
            inlier_errs.push_back(proj_err);
            inlier_errs_sum += proj_err;
        }
        // 计算外点误差
        for (int i = 0; i < P2D_outliers.size(); i++) {
            vector<3> p = pose.q.conjugate() * (P3D_outliers[i] - pose.p);
            double proj_err =
                (apply_k(p, curr_frame->K) -
                 apply_k(P2D_outliers[i].homogeneous(), curr_frame->K))
                    .norm();
            outlier_errs.push_back(proj_err);
            outlier_errs_sum += proj_err;
        }
    }
    // 由预测相对位姿 R, t 生成本质矩阵 E = [t]x R
    matrix<3> E = compute_essential_matrix(R, t);
    // 计算基础矩阵 F = K^(-1).E.K^(-1)
    matrix<3> F =
        keyframe->K.transpose().inverse() * E * curr_frame->K.inverse();
    // 构建 inlier 和 outlier 集合
    std::vector<vector<2>> inlier_set1, inlier_set2;
    std::vector<vector<2>> outlier_set1, outlier_set2;
    // 遍历当前帧的所有特征点
    for (size_t i = 0; i < curr_frame->keypoint_num(); ++i) {
        // 这个特征点已关联到某个3D点
        if (m_indices_map[i] != -1) {
            // 说明在关键帧中也能找到该特征点 (帧 j 的索引)
            if (size_t j =
                    curr_frame->get_track(i)->get_keypoint_index(keyframe);
                j != nil()) {
                // // 如果 mask[m_indices_map[i]] == true，表示内点
                if (mask[m_indices_map[i]]) {
                    // 将内点添加到 inlier 集合中
                    inlier_set1.push_back(
                        apply_k(keyframe->get_keypoint(j), keyframe->K));
                    inlier_set2.push_back(
                        apply_k(curr_frame->get_keypoint(i), curr_frame->K));
                } else {
                    // 否则将外点添加到 outlier 集合中
                    outlier_set1.push_back(
                        apply_k(keyframe->get_keypoint(j), keyframe->K));
                    outlier_set2.push_back(
                        apply_k(curr_frame->get_keypoint(i), curr_frame->K));
                }
            }
        }
    }
    // 计算对极几何距离
    std::vector<double> inliers_dist, outliers_dist;

    // 计算内点对极几何距离
    for (int i = 0; i < inlier_set1.size(); i++) {
        vector<2> &p1 = inlier_set1[i];
        vector<2> &p2 = inlier_set2[i];
        // compute_epipolar_dist(F, p1, p2) 点 p1 相对于 p2 的对极几何距离计算
        // compute_epipolar_dist(F.transpose(), p2, p1) 点 p2 相对于 p1 的对极几何距离计算
        double err = compute_epipolar_dist(F, p1, p2) +
                     compute_epipolar_dist(F.transpose(), p2, p1);
        // 将内点的距离存于 inliers_dist
        inliers_dist.push_back(err);
    }
    // 计算外点对极几何距离
    for (int i = 0; i < outlier_set1.size(); i++) {
        vector<2> &p1 = outlier_set1[i];
        vector<2> &p2 = outlier_set2[i];
        double err = compute_epipolar_dist(F, p1, p2) +
                     compute_epipolar_dist(F.transpose(), p2, p1);
        // 外点的距离存于 outliers_dist             
        outliers_dist.push_back(err);
    }

    // 如果内点或外点数量小于 20，则无法计算阈值
    size_t min_num = 20;
    if (inliers_dist.size() < min_num || outliers_dist.size() < min_num)
        return false;

    // 对内点和外点的距离进行排序
    std::sort(inliers_dist.begin(), inliers_dist.end());
    std::sort(outliers_dist.begin(), outliers_dist.end());

    // 计算内外点误差的中位数
    double th1 = inliers_dist[size_t(inliers_dist.size() * 0.5)];
    double th2 = outliers_dist[size_t(outliers_dist.size() * 0.5)];

    // 判断是否存在歧义
    // 如果外点距离的中位数 th2 并没有显著大于内点距离的中位数 th1，
    // 例如 th2 < th1 * 2，说明内外点的几何距离分布过于接近（即歧义较大）。
    if (th2 < th1 * 2) // mean there is ambiguity
        return false;

    // 动态设置阈值
    // 内外点的中位距离 th1 和 th2 进行平均，得到一个新的阈值 m_th
    m_th = (th1 + th2) / 2;

    // 更新当前帧中每个特征点的状态
    // 遍历当前帧的所有特征点
    for (size_t k = 0; k < curr_frame->keypoint_num(); ++k) {
        // 特征点已被跟踪到
        if (Track *track = curr_frame->get_track(k)) {
            // track->tag(TT_STATIC) = true;
            // 这个特征点已关联到某个3D点
            if (m_indices_map[k] != -1) {
                // 如果是内点
                if (mask[m_indices_map[k]]) {
                    curr_frame->get_track(k)->tag(TT_OUTLIER) = false;
                    // TT_STATIC：表示该特征被认为是静态、可信赖的点
                    curr_frame->get_track(k)->tag(TT_STATIC) = true;
                } else {
                    // TT_OUTLIER：表示该特征在本次几何估计中被认为是外点
                    curr_frame->get_track(k)->tag(TT_OUTLIER) = true;
                    curr_frame->get_track(k)->tag(TT_STATIC) = false;
                }
            }
        }
    }

    return true;
}

// 更新当前帧中每个特征点的状态
void SlidingWindowTracker::update_track_status() {
    // 获取当前帧
    Frame *curr_frame = map->get_frame(map->frame_num() - 1);
    // 获取当前帧在特征跟踪地图中的索引
    size_t frame_id = feature_tracking_map->frame_index_by_id(curr_frame->id());
    // 如果当前帧在特征跟踪地图中不存在，则返回
    if (frame_id == nil())
        return;

    // 在 feature_tracking_map 中找到与 curr_frame 相同 ID 的 “旧帧”
    Frame *old_frame = feature_tracking_map->get_frame(frame_id);

    // outlier_cnts[i]：记录第 i 个特征点被判定为“外点”的次数
    std::vector<size_t> outlier_cnts(curr_frame->keypoint_num(), 0);
    // matches_cnts[i]：记录第 i 个特征点被匹配到的次数
    std::vector<size_t> matches_cnts(curr_frame->keypoint_num(), 0);

    // 需要要检查的帧的id
    size_t start_idx = std::min(
        map->frame_num() - 1,
        std::max(map->frame_num() - 1 - config->parsac_keyframe_check_size(),
                 size_t(0)));
    // 遍历要坚持的帧id             
    for (size_t i = start_idx; i < map->frame_num() - 1; i++) {
        std::vector<char> mask;
        std::vector<size_t> pts_to_index;
        // 进行2D-2D特征匹配，返回 mask（内点 / 外点）和 pts_to_index（指示当前帧特征点索引）
        if (filter_parsac_2d2d(map->get_frame(i), curr_frame, mask,
                               pts_to_index)) {
            // 遍历 mask，统计外点和匹配次数
            for (size_t j = 0; j < mask.size(); j++) {
                if (!mask[j]) {
                    outlier_cnts[pts_to_index[j]] += 1;
                }
                matches_cnts[pts_to_index[j]] += 1;
            }
        }
    }
    // 遍历当前帧的所有特征点
    for (size_t i = 0; i < curr_frame->keypoint_num(); i++) {
        // 如果当前帧的特征点被跟踪到
        if (Track *curr_track = curr_frame->get_track(i)) {
            // 如果当前帧的特征点在特征跟踪帧中也被跟踪到
            if (size_t j = curr_track->get_keypoint_index(old_frame)) {
                if (j != nil()) {
                    // 获取 old_frame 中的对应 Track（即特征点 j）
                    Track *old_track = old_frame->get_track(j);
                    // 定义一个外点阈值 outlier_th，用总帧数 / 2 来衡量
                    size_t outlier_th = map->frame_num() / 2;
                    
                    // 如果当前特征点在统计中，外点出现次数大于 outlier_th / 2
                    // 且外点出现次数大于匹配次数的 80%，则标记为非静态
                    if (outlier_cnts[i] > outlier_th / 2 &&
                        outlier_cnts[i] > 0.8 * matches_cnts[i]) {
                        curr_track->tag(TT_STATIC) = false;
                    }
                    //  // 如果 old_track 或 current track 任意一方不再是静态，那么都标记为非静态
                    if (!old_track->tag(TT_STATIC) ||
                        !curr_track->tag(TT_STATIC)) {
                        curr_track->tag(TT_STATIC) = false;
                        old_track->tag(TT_STATIC) = false;
                    }
                }
            }
        }
    }
}

} // namespace xrslam
