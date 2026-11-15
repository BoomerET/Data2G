// The station's transmit thread: bursts the receive (engine) thread hands
// over, each keyed (rig::Keyer), played into the playback FIFO on the sound
// card's own clock and released. The rig's PTT calls, the drain and the off
// delay all block here, never on the receive side; real-time scheduling,
// where granted (audio::thread_init("tx")), keeps the feeding on time under
// load. host.py's serve() does the same inline in its loop.
//
// Threads: submit() and abort() come from the engine thread; active() is
// read there every block (the radio's receive audio is muted while it is
// set); ptt() is polled by the owner; on_ptt is called on the transmit
// thread as the rig is keyed and released.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rig/keyer.hpp"

namespace data2g::audio {
class PlaybackFifo;
}

namespace data2g::app {

class Transmitter {
public:
    // `rate`: the playback FIFO's (the sound card's) sample rate; bursts come
    // at config::FS and are interpolated up to it. `gain`: linear, applied
    // before clipping to +-1. `ptt`, `must_release`, `off_delay_s`: the
    // Keyer's. Starts the thread.
    Transmitter(audio::PlaybackFifo& play, int rate, double gain, rig::Keyer::Ptt ptt, rig::Keyer::MustRelease must_release,
                double off_delay_s, std::function<void(bool on)> on_ptt, std::function<void(double s)> on_latency = {});
    // stop()s; then PTT off, if the rig was ever keyed (~Keyer).
    ~Transmitter();
    Transmitter(const Transmitter&) = delete;
    Transmitter& operator=(const Transmitter&) = delete;

    // Queued behind what is playing. `behind_s`: how far the engine's clock (its burst's first sample) is
    // behind real time now.
    void submit(std::shared_ptr<const std::vector<double>> burst, double behind_s = 0.0);
    // ABORT, or the engine dying: what is playing stops (the FIFO is flushed, PTT drops) and what is queued is dropped.
    void abort();
    // The thread joined; PTT is off when it returns. Idempotent.
    void stop();

    // From keying until our audio has drained (not the PTT off): the receive side mutes.
    bool active() const { return active_; }
    bool ptt() const { return ptt_; }
    // `on_latency`, on the transmit thread once per burst as its first block is fed: seconds from where the
    // engine's clock put the burst's first sample to where it leaves the card (keying, the lead, the
    // queue, the output latency, and submit()'s `behind_s`).
    // Linear, as the constructor's; from the next block on.
    void set_gain(double gain) { gain_ = gain; }
    void set_off_delay(double s) { keyer_.set_off_delay(s); }  // the Keyer's, from the next unkey

private:
    void run();

    struct Job {
        std::shared_ptr<const std::vector<double>> audio;
        unsigned generation;  // an abort moves the generation on: older jobs are dropped, a playing one stops
        std::chrono::steady_clock::time_point engine_t;  // real time of the burst's first sample by the engine's clock
    };

    audio::PlaybackFifo& play_;
    const int rate_;
    std::atomic<double> gain_;
    std::function<void(bool)> on_ptt_;
    std::function<void(double)> on_latency_;
    rig::Keyer keyer_;
    std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Job> q_;
    std::atomic<unsigned> generation_{0};
    std::atomic<bool> stop_{false}, active_{false}, ptt_{false};
    std::thread thread_;  // last: everything above is ready
};

}  // namespace data2g::app
