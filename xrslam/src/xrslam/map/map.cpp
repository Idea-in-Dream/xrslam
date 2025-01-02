#include <xrslam/ar/virtual_object_manager.h>
#include <xrslam/estimation/marginalization_factor.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>
#include <xrslam/utility/debug.h>

namespace xrslam {

struct Map::construct_by_map_t {};

Map::Map() = default;

Map::~Map() = default;

void Map::clear() {
    frames.clear();
    tracks.clear();
}

// 用于将一个帧（Frame）对象添加到 Map 的帧列表（frames）中，可以在末尾追加，也可以插入到指定位置
void Map::attach_frame(std::unique_ptr<Frame> frame, size_t position) {
    // 将传入的帧对象的 map 指针设置为当前的 Map 对象。
    frame->map = this;
    if (position == nil()) {
        // 将传入的 std::unique_ptr<Frame> 移动到 frames，避免复制。
        // std::move(frame) 将传入的 std::unique_ptr<Frame> 移动到 frames，避免复制。
        frames.emplace_back(std::move(frame));
    } else {
        // 将传入的 std::unique_ptr<Frame> 移动到 frames 的指定位置
        frames.emplace(frames.begin() + position, std::move(frame));
    }
}
// 从 Map 中分离指定索引的帧，但不清除帧的内容
std::unique_ptr<Frame> Map::detach_frame(size_t index) {
    // 获取 frames[index] 中的帧指针（不释放资源）
    std::unique_ptr<Frame> frame = std::move(frames[index]);
    // 将索引 index 处的帧从 frames 中移出
    frames.erase(frames.begin() + index);
    // 将移出的帧的 map 指针设置为 nullptr，表示该帧不再属于任何 Map
    frame->map = nullptr;
    // 返回被移除的帧，供外部操作
    return frame;
}
// 取消指定帧中所有关键点与轨迹（Track）的关联
void Map::untrack_frame(Frame *frame) {
    // 遍历帧中的所有关键点
    for (size_t i = 0; i < frame->keypoint_num(); ++i) {
        // 如果关键点有对应的轨迹
        if (Track *track = frame->get_track(i)) {
            // 将该关键点从轨迹中移除
            track->remove_keypoint(frame);
        }
    }
}

void Map::erase_frame(size_t index) {
    // 获取 frames[index] 中的帧指针（不释放资源）
    Frame *frame = frames[index].get();
    // 取消该帧的所有关键点与轨迹的关联
    untrack_frame(frame);
    // 将帧从 frames 中移除，同时将其 map 指针置为 nullptr
    detach_frame(index);
}

// 该函数的目的是对指定的帧（frame）进行边缘化（marginalization）
void Map::marginalize_frame(size_t index) {
    // 该实现仅允许边缘化索引为 0 的帧。也就是说，只能对第一个帧进行边缘化
    runtime_assert(index == 0, "currently only index == 0 is allowed");
    // 如果 marginalization_factor（边缘化因子）没有初始化，程序会报错
    runtime_assert(marginalization_factor,
                   "marginalization_factor is not initialized yet");
    //  执行边缘化             
    marginalization_factor->marginalize(index);
    // 得到边缘化的帧
    Frame *frame = frames[index].get();
    // 遍历帧中的所有关键点
    for (size_t i = 0; i < frame->keypoint_num(); ++i) {
        // 对于每一个关键点，检查它是否关联到某个 Track 中
        if (Track *track = frame->get_track(i)) {
            // 调用 remove_keypoint 方法移除这个关键点与帧的关联
            track->remove_keypoint(frame);
        }
    }
    // 删除 frames 中的这帧数据。通过 erase 移除索引为 index 的帧
    frames.erase(frames.begin() + index);
}

// 函数接受一个帧 ID (id) 作为参数，在 frames 容器中查找对应帧的索引，并返回该索引。
size_t Map::frame_index_by_id(size_t id) const {
    // 内部辅助类 FrameID 用于比较帧 ID
    struct FrameID {
        // 提取帧指针中的 ID
        FrameID(const std::unique_ptr<Frame> &frame) : id(frame->id()) {}
        // 直接用给定的 ID 初始化
        FrameID(size_t id) : id(id) {}
        // 允许比较两个 FrameID 对象，实际上比较的是它们的 id
        bool operator<(const FrameID &fi) const { return id < fi.id; }
        size_t id;
    };
    // 使用 std::lower_bound 在 frames 容器中查找第一个大于等于给定 ID 的帧
    auto it = std::lower_bound(frames.begin(), frames.end(), id,
                               std::less<FrameID>());
    // 如果 it == frames.end()，表示未找到任何帧
    if (it == frames.end())
        return nil();
    // 如果找到的帧的 id 大于 输入 id，也表示没有匹配帧
    if (id < (*it)->id())
        return nil();
    // 如果找到帧，计算迭代器 it 到 frames.begin() 的距离，返回帧的索引。
    return std::distance(frames.begin(), it);
}

Track *Map::create_track() {
    std::unique_ptr<Track> track =
        std::make_unique<Track>(construct_by_map_t());
    track->map_index = tracks.size();
    track->map = this;
    track_id_map[track->id()] = track.get();
    return tracks.emplace_back(std::move(track)).get();
}

void Map::erase_track(Track *track) {
    while (track->keypoint_num() > 0) {
        track->remove_keypoint(track->keypoint_map().begin()->first, false);
    }
    recycle_track(track);
}

void Map::prune_tracks(const std::function<bool(const Track *)> &condition) {
    std::vector<Track *> tracks_to_prune;
    for (size_t i = 0; i < track_num(); ++i) {
        if (Track *track = get_track(i); condition(track)) {
            tracks_to_prune.push_back(track);
        }
    }
    for (const auto &track : tracks_to_prune) {
        erase_track(track);
    }
}

Track *Map::get_track_by_id(size_t id) const {
    if (track_id_map.count(id)) {
        return track_id_map.at(id);
    } else {
        return nullptr;
    }
}

void Map::recycle_track(Track *track) {
    if (track->map_index != tracks.back()->map_index) {
        tracks[track->map_index].swap(tracks.back());
        tracks[track->map_index]->map_index = track->map_index;
    }
    track_id_map.erase(track->id());
    tracks.pop_back();
}

void Map::create_virtual_object_manager(Localizer *localizer) {
    virtual_object_manager =
        std::make_unique<VirtualObjectManager>(this, localizer);
}

void Map::create_virtual_object_manager() {
    virtual_object_manager = std::make_unique<VirtualObjectManager>(this);
}

size_t Map::create_virtual_object() {
    return virtual_object_manager->create_virtual_object();
}

void Map::update_virtual_objects() {
    virtual_object_manager->update_virtual_objects();
}

OutputObject Map::get_virtual_object_pose_by_id(size_t id) const {
    return virtual_object_manager->get_virtual_object_pose_by_id(id);
}

OutputObject Map::get_virtual_object_pose(size_t index) const {
    return virtual_object_manager->get_virtual_object_pose(index);
}

size_t Map::virtual_object_num() const {
    return virtual_object_manager->virtual_object_num();
}

} // namespace xrslam
