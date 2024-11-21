#include <xrslam/utility/worker.h>

namespace xrslam {

// 一个循环的工作函数，用于在工作线程中不断处理任务
void Worker::worker_loop() {
    // 循环在 worker_running 为 true 时运行，当外部设置 worker_running = false 时，循环结束。
    while (worker_running) {
        // 调用 lock() 获取一个 unique_lock 对象 l
        auto l = lock();
#if defined(XRSLAM_ENABLE_THREADING)
        // 如果 worker_running 为 true 且任务队列为空，则等待 worker_cv 被通知
        if (worker_running && empty()) {
            // 等待 worker_cv 被通知，直到 worker_running 为 false 或任务队列不为空
            worker_cv.wait(l, [this] { return !worker_running || !empty(); });
        }
#else
        // 直接检查任务队列是否为空
        if (empty())
            break;
#endif
        // 如果被设置为 false，则直接退出循环
        if (!worker_running)
            break;
        // 虚函数 work(l) 来执行实际任务，work() 方法由派生类实现，并持有锁 l
        work(l);
    }
}

} // namespace xrslam
