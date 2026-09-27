#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace teleoperation
{
    // Fixed-capacity queue for transferring telemetry from one high-frequency
    // producer to one batch-oriented consumer. This class is process-local;
    // a separate logging process must receive drained batches through ROS2 or
    // another explicitly selected interprocess transport.
    template <typename Sample, std::size_t Capacity> class TelemetryRingBuffer
    {
      public:
        bool tryPush(const Sample &sample)
        {
            const std::size_t write = write_position_.load(std::memory_order_relaxed);

            const std::size_t read = read_position_.load(std::memory_order_acquire);

            if (write - read >= Capacity)
            {
                dropped_sample_count_.fetch_add(1, std::memory_order_relaxed);

                return false;
            }

            storage_[write % Capacity] = sample;

            write_position_.store(write + 1, std::memory_order_release);

            return true;
        }

        // Removes up to requested_count samples and places them in chronological
        // order at the beginning of samples. The returned count says how many
        // entries in samples are valid. BatchCapacity is the caller's maximum
        // batch storage; requested_count can be tuned at runtime.
        template <std::size_t BatchCapacity>
        std::size_t tryPopBatch(std::array<Sample, BatchCapacity> &samples,
                                std::size_t requested_count = BatchCapacity)
        {
            static_assert(BatchCapacity > 0, "BatchCapacity must be greater than zero");

            const std::size_t read = read_position_.load(std::memory_order_relaxed);

            const std::size_t write = write_position_.load(std::memory_order_acquire);

            const std::size_t available_count = write - read;

            std::size_t count = requested_count;

            if (count > BatchCapacity)
            {
                count = BatchCapacity;
            }

            if (count > available_count)
            {
                count = available_count;
            }

            for (std::size_t i = 0; i < count; ++i)
            {
                samples[i] = storage_[(read + i) % Capacity];
            }

            if (count > 0)
            {
                read_position_.store(read + count, std::memory_order_release);
            }

            return count;
        }

        std::size_t available() const noexcept
        {
            const std::size_t write = write_position_.load(std::memory_order_acquire);

            const std::size_t read = read_position_.load(std::memory_order_relaxed);

            return write - read;
        }

        std::size_t droppedSampleCount() const noexcept
        {
            return dropped_sample_count_.load(std::memory_order_relaxed);
        }

      private:
        static_assert(Capacity > 1, "Capacity must be greater than one");

        static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");

        static_assert(std::atomic<std::size_t>::is_always_lock_free,
                      "The real-time telemetry buffer requires lock-free position atomics");

        std::array<Sample, Capacity> storage_{};

        std::atomic<std::size_t> write_position_{0};
        std::atomic<std::size_t> read_position_{0};
        std::atomic<std::size_t> dropped_sample_count_{0};
    };
}
