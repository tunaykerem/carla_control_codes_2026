#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// profiling.hpp — Lightweight chrono-based profiling macros for pipeline debug
//
// Usage in reader side:
//   PROF_READ_BEGIN("OUSTER");     // at callback entry
//   ... payload prep ...
//   PROF_READ_T1();                // after data prep
//   ... tcp send ...
//   PROF_READ_END("OUSTER");      // after send()
//
// Usage in publisher side:
//   PROF_PUB_BEGIN("OUSTER");     // at tcp receive
//   ... processing ...
//   PROF_PUB_T4();                // after processing
//   ... publish ...
//   PROF_PUB_END("OUSTER");      // after publish()
// ─────────────────────────────────────────────────────────────────────────────

#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
#include <atomic>

namespace carla_sensor_bridge {
namespace profiling {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

inline long long us_since(TimePoint start) {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now() - start).count();
}

inline double epoch_sec() {
    auto now = std::chrono::system_clock::now();
    auto epoch = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(epoch).count() / 1e6;
}

} // namespace profiling
} // namespace carla_sensor_bridge

// ═══════════════════════════════════════════════════════════════════════════
// Compile with -DENABLE_PROFILING to re-enable verbose timing output.
// ═══════════════════════════════════════════════════════════════════════════

#ifdef ENABLE_PROFILING

// ── Reader-side macros (T0, T1, T2) ─────────────────────────────────────

#define PROF_READ_BEGIN(tag)                                                    \
    static auto _prof_prev_cb =                                                 \
        carla_sensor_bridge::profiling::Clock::now();                            \
    auto _prof_t0 = carla_sensor_bridge::profiling::Clock::now();               \
    long long _prof_dt_cb =                                                     \
        std::chrono::duration_cast<std::chrono::microseconds>(                  \
            _prof_t0 - _prof_prev_cb).count();                                  \
    _prof_prev_cb = _prof_t0;                                                   \
    double _prof_epoch = carla_sensor_bridge::profiling::epoch_sec();

#define PROF_READ_T1()                                                          \
    auto _prof_t1 = carla_sensor_bridge::profiling::Clock::now();

#define PROF_READ_END(tag)                                                      \
    auto _prof_t2 = carla_sensor_bridge::profiling::Clock::now();               \
    long long _dt1 = std::chrono::duration_cast<std::chrono::microseconds>(     \
        _prof_t1 - _prof_t0).count();                                           \
    long long _dt2 = std::chrono::duration_cast<std::chrono::microseconds>(     \
        _prof_t2 - _prof_t1).count();                                           \
    static int _prof_read_cnt = 0;                                              \
    if (++_prof_read_cnt % 20 == 0) {                                           \
        std::cout << "[PROF][" << tag << "_READ] T0="                           \
                  << std::fixed << std::setprecision(6) << _prof_epoch          \
                  << " dt_cb=" << _prof_dt_cb << "us"                           \
                  << " T1=+" << _dt1 << "us"                                    \
                  << " T2=+" << _dt2 << "us" << std::endl;                      \
    }

// ── Publisher-side macros (T3, T4, T5 + Hz counter) ─────────────────────

#define PROF_PUB_BEGIN(tag)                                                     \
    auto _prof_t3 = carla_sensor_bridge::profiling::Clock::now();

#define PROF_PUB_T4()                                                           \
    auto _prof_t4 = carla_sensor_bridge::profiling::Clock::now();

#define PROF_PUB_END(tag)                                                       \
    auto _prof_t5 = carla_sensor_bridge::profiling::Clock::now();               \
    long long _dt4 = std::chrono::duration_cast<std::chrono::microseconds>(     \
        _prof_t4 - _prof_t3).count();                                           \
    long long _dt5 = std::chrono::duration_cast<std::chrono::microseconds>(     \
        _prof_t5 - _prof_t4).count();                                           \
    long long _dt_total = _dt4 + _dt5;                                          \
    /* Hz counter: report every second */                                       \
    static int _prof_pub_frame_cnt = 0;                                         \
    static auto _prof_pub_hz_start =                                            \
        carla_sensor_bridge::profiling::Clock::now();                            \
    _prof_pub_frame_cnt++;                                                       \
    long long _prof_hz_elapsed =                                                \
        std::chrono::duration_cast<std::chrono::microseconds>(                  \
            _prof_t5 - _prof_pub_hz_start).count();                             \
    if (_prof_hz_elapsed >= 1000000) {                                          \
        double _hz = (_prof_pub_frame_cnt * 1e6) / _prof_hz_elapsed;            \
        std::cout << "[PROF][" << tag << "_PUB] process=" << _dt4 << "us"       \
                  << " publish=" << _dt5 << "us"                                \
                  << " total=" << _dt_total << "us"                             \
                  << " Hz=" << std::fixed << std::setprecision(1) << _hz        \
                  << " frames=" << _prof_pub_frame_cnt << std::endl;            \
        _prof_pub_frame_cnt = 0;                                                \
        _prof_pub_hz_start = _prof_t5;                                          \
    }

// ── TCP Sender queue depth macro ────────────────────────────────────────

#define PROF_TCP_QUEUE(queue_size, payload_bytes)                                \
    static int _prof_tcp_cnt = 0;                                               \
    if (++_prof_tcp_cnt % 40 == 0) {                                            \
        std::cout << "[PROF][TCP_QUEUE] depth=" << queue_size                   \
                  << " payload=" << payload_bytes << "B" << std::endl;          \
    }

#else  // !ENABLE_PROFILING — all macros are silent no-ops

#define PROF_READ_BEGIN(tag)             do {} while(0)
#define PROF_READ_T1()                   do {} while(0)
#define PROF_READ_END(tag)               do {} while(0)
#define PROF_PUB_BEGIN(tag)              do {} while(0)
#define PROF_PUB_T4()                    do {} while(0)
#define PROF_PUB_END(tag)                do {} while(0)
#define PROF_TCP_QUEUE(q, p)             do {} while(0)

#endif // ENABLE_PROFILING
