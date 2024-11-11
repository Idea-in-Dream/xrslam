#ifndef XRSLAM_WORKER_H
#define XRSLAM_WORKER_H

#include <xrslam/common.h>

namespace xrslam {
// 定义了一个可启动、暂停和停止的工作线程框架,其中包含了线程和条件变量等机制，用于多线程任务管理。
// Worker 是一个抽象基类，它需要派生类实现 empty() 和 work() 方法来处理实际任务。
class Worker {
  public:
    // 构造函数 Worker()
    Worker() {}
    // 析构函数 ~Worker()
    virtual ~Worker() { stop(); }
    // 启动工作线程,将 worker_running 设置为 true，表示线程正在运行
    void start() {
        worker_running = true;
#if defined(XRSLAM_ENABLE_THREADING)
        worker_thread = std::thread(&Worker::worker_loop, this);
#endif
    }
    // 检查 worker_running 是否为 true，如果是，则将其设置为 false 来指示线程应停止
    void stop() {
        if (worker_running) {
            worker_running = false;
#if defined(XRSLAM_ENABLE_THREADING)
            worker_cv.notify_all();
            worker_thread.join();
#endif
        }
    }
    // 锁定 worker_mutex
    std::unique_lock<std::mutex> lock() const {
        return std::unique_lock(worker_mutex);
    }
    // 解锁传入的锁 l，解除对共享数据的保护
    void resume(std::unique_lock<std::mutex> &l) {
        l.unlock();
#if defined(XRSLAM_ENABLE_THREADING)
        worker_cv.notify_all();
#else
    // 如果不支持多线程，直接调用 worker_loop() 在当前线程上执行任务
        worker_loop();
#endif
    }
    // 检查是否还有待处理的任务，派生类需要实现这个方法
    virtual bool empty() const = 0;
    // 执行实际任务，派生类需要实现这个方法
    virtual void work(std::unique_lock<std::mutex> &l) = 0;

  protected:
    // 表示线程的运行状态 
    std::atomic<bool> worker_running;

  private:
    // 用于同步访问共享数据的互斥锁
    void worker_loop();

#if defined(XRSLAM_ENABLE_THREADING)
    std::thread worker_thread;
    std::condition_variable worker_cv;
#endif
    // 
    mutable std::mutex worker_mutex;
};

} // namespace xrslam

#endif // XRSLAM_WORKER_H
