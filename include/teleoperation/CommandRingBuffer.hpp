#pragma once

#include "teleoperation/CommandTypes.hpp"

#include <array>
#include <atomic>
#include <cstddef>

namespace teleoperation
{
    // Fixed-size interpolation-window buffer. Logical positions are monotonically
    // increasing unsigned counters; physical positions wrap modulo Capacity.
    // This class supports exactly one producer and one consumer.
    template <std::size_t Capacity, std::size_t LookbackWindow> class CommandRingBuffer
    {
      public:
        bool tryPush(const TargetCommand &command)
        {
            const std::size_t write = write_position_.load(std::memory_order_relaxed);
            const std::size_t read_head = read_head_position_.load(std::memory_order_acquire);

            // Acquire observes a safely published read head. A stale head may
            // cause a conservative false return, but never an unsafe overwrite.
            const std::size_t oldest_retained = read_head - LookbackWindow + 1;
            const std::size_t occupied = write - oldest_retained;

            if (occupied >= Capacity)
            {
                return false;
            }

            storage_[write % Capacity] = command;

            write_position_.store(write + 1, std::memory_order_release);

            return true;
        }

        bool hasInterpolationWindow() const
        {
            const std::size_t write = write_position_.load(std::memory_order_acquire);
            const std::size_t read_head = read_head_position_.load(std::memory_order_relaxed);

            const std::size_t occupied = write - (read_head - LookbackWindow + 1);

            return occupied >= LookbackWindow;
        }

        bool tryGetInterpolationSet(std::array<TargetCommand, LookbackWindow> &commands) const
        {
            const std::size_t read_head = read_head_position_.load(std::memory_order_relaxed);
            const std::size_t write = write_position_.load(std::memory_order_acquire);

            const std::size_t oldest_retained = read_head - LookbackWindow + 1;
            const std::size_t occupied = write - oldest_retained;

            if (occupied < LookbackWindow)
            {
                return false;
            }

            for (std::size_t i = 0; i < LookbackWindow; ++i)
            {
                commands[i] = storage_[(oldest_retained + i) % Capacity];
            }

            return true;
        }

        bool tryAdvanceInterpolationSet()
        {
            const std::size_t read_head = read_head_position_.load(std::memory_order_relaxed);
            const std::size_t write = write_position_.load(std::memory_order_acquire);

            const std::size_t occupied = write - (read_head - LookbackWindow + 1);

            if (occupied < LookbackWindow + 1)
            {
                return false;
            }

            read_head_position_.store(read_head + 1, std::memory_order_release);

            return true;
        }

      private:
        static_assert(Capacity > LookbackWindow,
                      "Capacity must provide more slots than the LookbackWindow demands.");

        static_assert((LookbackWindow >= 2) && (LookbackWindow <= 5),
                      "LookbackWindow must range between 2 and 5.");

        static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");

        static_assert(std::atomic<std::size_t>::is_always_lock_free,
                      "The real-time command buffer requires lock-free position atomics");

        std::array<TargetCommand, Capacity> storage_{};

        std::atomic<std::size_t> write_position_{0};
        std::atomic<std::size_t> read_head_position_{LookbackWindow - 1};
    };
}
