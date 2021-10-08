#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <vector>
#include <queue>
#include <atomic>
#include <future>
#include <condition_variable>
#include <thread>
#include <functional>
#include <stdexcept>

class threadpool
{
	using Task = std::function<void()>;	//定义类型
	std::vector<std::thread> _pool;     //线程池
	std::queue<Task> _tasks;            //任务队列
	std::mutex _lock;                   //同步
	std::condition_variable _task_cv;   //条件阻塞
	std::atomic<bool> _stop{ false };     //线程池是否停止

public:
	inline threadpool(unsigned short size = 4) { addThread(size); }
	inline ~threadpool()
	{
		_stop.store(true);
		_task_cv.notify_all(); 
		for (std::thread& thread : _pool) {
			if (thread.joinable())
				thread.join(); 
		}
	}

public:
	// 提交一个任务
	// 调用.get()获取返回值会等待任务执行完,获取返回值
	// 有两种方法可以实现调用类成员，
	// 一种是使用   bind： .commit(std::bind(&Dog::sayHello, &dog));
	// 一种是用   mem_fn： .commit(std::mem_fn(&Dog::sayHello), this)
	template<class F, class... Args>
	auto commit(F&& f, Args&&... args) ->std::future<decltype(f(args...))>
	{
		if (_stop.load())
			throw std::runtime_error("commit on ThreadPool is stopped.");

		using RetType = decltype(f(args...)); // typename std::result_of<F(Args...)>::type, 函数 f 的返回值类型
		auto task = std::make_shared<std::packaged_task<RetType()>>(
			std::bind(std::forward<F>(f), std::forward<Args>(args)...)
			);

		std::future<RetType> future = task->get_future();
		{
			std::lock_guard<std::mutex> lock{ _lock };
			_tasks.emplace([task]() { 
				(*task)();
				});
		}

		_task_cv.notify_one(); 

		return future;
	}

private:
	void addThread(unsigned short size)
	{
		const int THREADPOOL_MAX_NUM = std::thread::hardware_concurrency();
		for (; _pool.size() < THREADPOOL_MAX_NUM && size > 0; --size)
		{
			_pool.emplace_back([this] 
			{ 
				while (!_stop.load())
				{
					Task task; 

					{
						std::unique_lock<std::mutex> lock{ this->_lock };
						this->_task_cv.wait(lock, [this] {
							return this->_stop.load() || !this->_tasks.empty();
							}); 
						if (this->_stop.load() && this->_tasks.empty())
							return;
						task = move(this->_tasks.front()); 
						_tasks.pop();
					}
					task();
				}
			});
		}
	}
};

#endif // !THREADPOOL_H

