// Simplified from talos-udp: include/talos/session.hpp (Session::poll, handleReserved, Publisher).
// Removed: socket setup and teardown, move semantics, error handling and drop logging.
// Illustrative, not compilable; the full implementation is not public.
//
// A session is one UDP socket. A publisher session sends to one fixed (ip, port). A subscriber
// session binds one port and hands each datagram to the handler registered for its topic.
// Two-way traffic uses two sessions on two ports, one per direction. There is no broker, no
// discovery and no "latest value" cache: a datagram arrives, its handler runs, done.

// Encode one typed message and send it.
template <class Codec, class T>
void publish(Session& session, const std::string& topic, const T& value)
{
    write_header(session.tx_buffer, topic, Codec::kTypeHash);
    Codec::encode(session.tx_buffer, value);
    sendto(session.socket, session.tx_buffer, session.destination);
}

// Receive and dispatch ONE datagram; the caller runs this in a loop on a receive thread.
bool receive_one(Session& session)
{
    uint8_t datagram[1500];
    sockaddr_in sender;
    const ssize_t n = recvfrom(session.socket, datagram, sizeof(datagram), &sender);
    if (n <= 0)
        return false;  // receive timeout
    const double received_at = monotonic_now();

    WireReader reader(datagram, n);
    const std::string topic = reader.read_string(reader.read_u8());
    const uint32_t type_hash = reader.read_u32();

    // Topics starting with "__" belong to the transport itself and never reach user code.
    if (topic == "__sync")
        return answer_clock_ping(session, reader, sender, received_at);

    const auto handler = session.handlers.find(topic);
    if (handler == session.handlers.end() || handler->second.type_hash != type_hash) {
        ++session.dropped;  // unknown topic or wrong type: count it, never misread it
        return false;
    }
    handler->second.decode_and_call(reader);  // the reader now points at the payload
    return true;
}

// Every subscriber session is also a clock server. A ping carries [t0], the sender's clock at
// sending; the reply carries [t0, t1 = when the ping arrived here, t2 = just before replying],
// sent straight back to the ping's source address.
bool answer_clock_ping(Session& session, WireReader& ping, const sockaddr_in& sender,
                       double received_at)
{
    const double t0 = ping.read_f64_array().front();
    std::vector<double> reply = {t0, received_at, monotonic_now()};
    write_header(session.sync_buffer, "__sync", DoubleArrayCodec::kTypeHash);
    DoubleArrayCodec::encode(session.sync_buffer, reply);
    sendto(session.socket, session.sync_buffer, sender);
    return true;
}
