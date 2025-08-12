//
// Created by tobias on 1/30/25.
//

#pragma once
#include <coroutine>
#include <exception>

namespace libstp::async
{
    template <typename T>
    class AsyncAlgorithm
    {
    public:
        struct promise_type
        {
            bool initialized = false;

            T current_value{};

            AsyncAlgorithm get_return_object()
            {
                return AsyncAlgorithm(std::coroutine_handle<promise_type>::from_promise(*this));
            }

            static std::suspend_always initial_suspend() noexcept
            {
                return {};
            }

            static std::suspend_always final_suspend() noexcept
            {
                return {};
            }

            void return_value(const T& value)
            {
                current_value = value;
            }

            static void unhandled_exception() { std::terminate(); }

            std::suspend_always yield_value(const T& value)
            {
                current_value = value;
                return {};
            }
        };

        using handle_type = std::coroutine_handle<promise_type>;

        explicit AsyncAlgorithm(handle_type h) : coro(h)
        {
        }

        AsyncAlgorithm(const AsyncAlgorithm&) = delete;

        AsyncAlgorithm(AsyncAlgorithm&& other) noexcept
            : coro(other.coro)
        {
            other.coro = nullptr;
        }

        ~AsyncAlgorithm()
        {
            if (coro)
            {
                coro.destroy();
            }
        }

        bool advance()
        {
            if (!coro || coro.done())
                return false;

            auto& p = coro.promise();

            if (!p.initialized)
            {
                p.initialized = true;
            }

            coro.resume();

            if (coro.done())
                return false;

            return true;
        }

        T current() const
        {
            return coro.promise().current_value;
        }

        [[nodiscard]] bool await_ready() const noexcept
        {
            // Ready if the coroutine has already produced a value.
            return coro.done();
        }

        void await_suspend(const std::coroutine_handle<> awaiting_coro) noexcept
        {
            // Resume the current coroutine and then resume the awaiting coroutine.
            if (coro && !coro.done())
            {
                coro.resume();
            }
            awaiting_coro.resume();
        }

        T await_resume() noexcept
        {
            return current();
        }

    private:
        handle_type coro;
    };
}
