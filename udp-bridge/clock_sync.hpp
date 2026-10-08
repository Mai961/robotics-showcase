// Simplified from talos-udp: include/talos/sync.hpp (offsetOf, rttOf, SyncEstimator, SyncClient).
// Removed: socket setup, threading, locking and stale-reply checks. Illustrative, not compilable;
// the full implementation is not public.
//
// Goal: know robot_clock - coprocessor_clock, so timestamps from the two machines can be compared.
// The coprocessor pings the robot's subscriber session (2 Hz by default) on the reserved
// "__sync" topic; the robot answers automatically (udp_session.hpp: answer_clock_ping).

// NTP-style arithmetic. t0 = ping sent (client clock), t1 = ping received (server clock),
// t2 = reply sent (server clock), t3 = reply received (client clock). All in seconds.
double clock_offset(double t0, double t1, double t2, double t3) { return ((t1 - t0) + (t2 - t3)) * 0.5; }
double round_trip(double t0, double t1, double t2, double t3) { return (t3 - t0) - (t2 - t1); }

// Keep the last `window` samples (16 by default) and trust the one with the SMALLEST round trip:
// it waited least in queues, so its two directions were the most symmetric.
struct ClockSyncEstimate { double offset_s, rtt_s, age_s; bool valid; };

ClockSyncEstimate estimate(const std::deque<Sample>& last_samples, double now)
{
    if (last_samples.empty())
        return {0.0, 0.0, 0.0, false};
    const Sample& best = *std::min_element(
        last_samples.begin(), last_samples.end(),
        [](const Sample& a, const Sample& b) { return a.rtt < b.rtt; });
    return {best.offset, best.rtt, now - best.received_at, true};
}

// One ping/reply cycle, run periodically on its own socket.
void ping_once(SyncClient& client)
{
    const double t0 = monotonic_now();
    send_f64_array(client.socket, "__sync", {t0});
    const auto reply = wait_for_reply(client.socket, /*timeout=*/std::min(client.period_s * 0.5, 0.25));
    if (!reply)
        return;
    const double t3 = monotonic_now();
    const auto [echoed_t0, t1, t2] = *reply;
    client.add_sample(clock_offset(t0, t1, t2, t3), round_trip(t0, t1, t2, t3), /*received_at=*/t3);
}
