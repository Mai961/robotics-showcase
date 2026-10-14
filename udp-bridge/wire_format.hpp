// Simplified from talos-udp: include/talos/wire.hpp and include/talos/codec.hpp.
// Removed: the reader class, bounds checks, and the codecs for bool/int/String (they follow the
// same pattern). Illustrative, not compilable; the full implementation is not public.
//
// Wire frame (one UDP datagram = one message), verbatim from wire.hpp:
//   ┌──────────┬─────────────────┬───────────┬───────────────────────────┐
//   │ topicLen │ topic (UTF-8)   │ typeHash  │ payload                   │
//   │  u8      │  topicLen bytes │  u32 LE   │  rest of datagram (LE)    │
//   └──────────┴─────────────────┴───────────┴───────────────────────────┘
// Every field is written byte by byte in little-endian order, so the bytes are the same on the
// roboRIO (32-bit ARM) and on an x86-64 coprocessor without relying on either CPU's layout.

// Write the frame header in front of every payload.
void write_header(std::vector<uint8_t>& frame, const std::string& topic, uint32_t type_hash)
{
    frame.clear();  // the buffer is reused for every message: no allocation in steady state
    put_u8(frame, topic.size());
    put_bytes(frame, topic);
    put_u32_le(frame, type_hash);
}

// The type hash is FNV-1a over a type name. A subscriber that registered a double array drops a
// datagram tagged "struct:Pose3d" instead of misreading its bytes.
constexpr uint32_t fnv1a(const char* name)
{
    uint32_t h = 2166136261u;
    while (*name) { h ^= static_cast<uint8_t>(*name++); h *= 16777619u; }
    return h;
}

// double[]: u16 count, then count x f64. Used for joint references, joint measurements, status.
struct DoubleArrayCodec {
    static constexpr uint32_t kTypeHash = fnv1a("vector") ^ (fnv1a("double") * 16777619u);
    static void encode(std::vector<uint8_t>& frame, const std::vector<double>& values)
    {
        put_u16_le(frame, values.size());
        for (double v : values) put_f64_le(frame, v);
    }
};

// Pose3d: mirrors wpi::Struct<frc::Pose3d>, 7 x f64 = 56 B (x, y, z, qw, qx, qy, qz), metres.
// The Java side packs and unpacks with WPILib's own Pose3d.struct, so these bytes must match it.
struct Pose3dCodec {
    static constexpr uint32_t kTypeHash = fnv1a("struct:Pose3d");
    static void encode(std::vector<uint8_t>& frame, const Pose3d& p)
    {
        for (double v : {p.x, p.y, p.z, p.qw, p.qx, p.qy, p.qz}) put_f64_le(frame, v);
    }
};
