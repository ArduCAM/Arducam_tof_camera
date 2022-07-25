#ifndef __SEMAPHORE_H
#define __SEMAPHORE_H

#include <mutex>
#include <condition_variable>
#include <cstdint>

class Semaphore
{
public:
    explicit Semaphore(int count = 0, int max_count = 3) : count_(count), max_count_(max_count) {}

    inline void Signal()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (count_ < max_count_)
        {
            ++count_;
            cv_.notify_one();
        }
    }

    inline bool Wait(int16_t ms = 0)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (ms == 0)
        {
            return true;
        }
        else if (ms < 0)
        {
            cv_.wait(lock, [=]
                     { return count_ > 0; });
        }
        else
        {
            if (cv_.wait_for(lock, std::chrono::milliseconds(ms), [=]
                             { return count_ > 0; }) == false)
                return false;
        }
        --count_;
        return true;
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    int count_;
    int max_count_;
};
#endif