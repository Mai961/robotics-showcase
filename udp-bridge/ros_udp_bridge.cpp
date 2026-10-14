// Simplified from talos_udp_bridge: src/talos_udp_bridge_node.cpp (build_ros_to_udp_factories,
// discover_ros_to_udp, setup_udp_to_ros). Removed: parameter declaration, per-topic rate limiting,
// the factories for the scalar and array types (same pattern), logging and the clock-sync
// publisher. Illustrative, not compilable; the full implementation is not public.
//
// The node is the coprocessor's end of the wire. Robot-facing ROS nodes stay plain ROS; this node
// is the only one that speaks UDP.

// ROS -> robot. The config lists topic NAMES only. Once a second, each listed topic that has
// appeared is subscribed with the converter registered for its message type.
void discover_topics_to_forward()
{
    for (const std::string& ros_topic : topics_waiting_for_a_publisher) {
        const std::string type = type_in_ros_graph(ros_topic);  // e.g. "geometry_msgs/msg/PoseStamped"
        if (type.empty())
            continue;  // not advertised yet; try again next second
        // The wire name is the ROS name without the leading slash: "odin_tree/field_base".
        const std::string wire_topic = strip_leading_slash(ros_topic);
        subscriptions.push_back(converters.at(type)(ros_topic, wire_topic));
    }
}

// PoseStamped -> Pose3d (56 B). Header and frame_id are not carried.
converters["geometry_msgs/msg/PoseStamped"] = [](auto ros_topic, auto wire_topic) {
    return subscribe<geometry_msgs::msg::PoseStamped>(ros_topic, [wire_topic](const auto& msg) {
        const auto& p = msg.pose.position;
        const auto& q = msg.pose.orientation;
        publish<Pose3dCodec>(to_robot, wire_topic, Pose3d{p.x, p.y, p.z, q.w, q.x, q.y, q.z});
    });
};

// JointState -> double[] in a FIXED joint order (joint_names, e.g. [q1, d, q3]), looked up by name.
// A message missing one of those joints is dropped: there is no positional fallback, so a
// reordered message can never put the wrong joint's value in a slot.
converters["sensor_msgs/msg/JointState"] = [](auto ros_topic, auto wire_topic) {
    return subscribe<sensor_msgs::msg::JointState>(ros_topic, [wire_topic](const auto& msg) {
        std::vector<double> ordered;
        for (const std::string& joint : joint_names) {
            const auto i = index_of(msg.name, joint);
            if (!i) return;  // missing joint: drop the whole message
            ordered.push_back(msg.position[*i]);
        }
        publish<DoubleArrayCodec>(to_robot, wire_topic, ordered);
    });
};

// Robot -> ROS. Each declared channel gives a wire topic, a ROS topic and a message type.
// Example: wire "wholebody/measured" (double[]) becomes /wholebody/measured (JointState).
void republish_joint_state(const std::string& wire_topic, const std::string& ros_topic)
{
    auto ros_publisher = create_publisher<sensor_msgs::msg::JointState>(ros_topic);
    subscribe_wire<DoubleArrayCodec>(from_robot, wire_topic, [=](const std::vector<double>& q) {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        msg.name = joint_names;  // re-attach the names the wire dropped
        msg.position = q;
        ros_publisher->publish(msg);
    });
}
