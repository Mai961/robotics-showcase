// Simplified from talos-udp: java/.../talos/udp/Session.java (rxLoop, answerSyncPing) and
// NativeUdp.java. Removed: thread start/stop, the drop counters and the exception handling.
// Illustrative, not compilable; the full implementation is not public.
//
// On the roboRIO the socket calls are C++ behind JNI (NativeUdp: openPub, openSub, send, recv,
// replyLast, closeHandle). Java owns the receive thread and the decoding; native code never calls
// back into the JVM. Buffers are direct ByteBuffers, so the native side reads them without a copy.

/** Receive thread: one datagram per iteration, decoded and dispatched before the next recv. */
private void receiveLoop() {
    ByteBuffer datagram = ByteBuffer.allocateDirect(1500).order(ByteOrder.LITTLE_ENDIAN);
    while (running) {
        int length = NativeUdp.recv(subscriberHandle, datagram, datagram.capacity());
        if (length <= 0) continue;  // receive timeout

        WireReader reader = new WireReader(datagram.position(0).limit(length));
        String topic = reader.str(reader.u8());
        int typeHash = reader.u32();

        // "__" topics belong to the transport: answered here, never shown to robot code.
        if (topic.equals("__sync")) {
            answerClockPing(reader);
            continue;
        }
        Subscription subscription = subscriptions.get(topic);
        if (subscription == null || subscription.typeHash() != typeHash) continue;  // dropped
        subscription.handler().accept(reader);
    }
}

/**
 * Reply [t0, t1, t2] to a ping [t0], sent back to the ping's source address. The robot's clock
 * is WPILib's FPGA timestamp when available, so the coprocessor learns the offset to the clock
 * the robot program actually stamps with.
 */
private void answerClockPing(WireReader ping) {
    double t1 = ROBOT_CLOCK.getAsDouble();
    ping.u16();  // element count
    double t0 = ping.f64();
    syncReply.reset();
    syncReply.header("__sync", SYNC_TYPE_HASH);
    syncReply.u16(3);
    syncReply.f64(t0);
    syncReply.f64(t1);
    syncReply.f64(ROBOT_CLOCK.getAsDouble());  // t2: immediately before sending
    NativeUdp.replyLast(subscriberHandle, syncReply.buffer(), syncReply.size());
}
