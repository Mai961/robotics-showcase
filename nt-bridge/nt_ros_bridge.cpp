// Simplified from talos_bridge: src/talos_bridge_node.cpp (constructor, build_ros_to_nt_factories,
// setup_ros_to_nt, setup_nt_to_ros, poll_nt_to_ros). Removed: parameter handling, per-topic rate
// limiting, most of the type converters (same pattern), logging and the parameter bridge.
// Illustrative, not compilable; the full implementation is not public.
// Written by Zimeng Chai (GitHub KeseterG).
//
// NetworkTables 4 (NT4) is WPILib's publish/subscribe system: the robot program runs the server,
// and dashboards and coprocessors connect as clients. This node is such a client, and mirrors ROS
// topics into NT and NT entries back into ROS.

void connect_to_robot()
{
    nt.StartClient4(client_identity);  // e.g. "talos_coproc"
    if (is_sim)
        nt.SetServer(sim_server_address, 5810);  // a local NT4 server when simulating
    else
        nt.SetServerTeam(team_number, 5810);     // the real robot is found by its team number
}

// ROS -> NT. Like the UDP bridge, the config lists topic NAMES only; each topic's type is read
// from the ROS graph once a second, and the topic is mirrored to "<nt_topic_root>/<topic>",
// e.g. /odin_tree/field_base -> /Talos/Topics/odin_tree/field_base.
void discover_topics_to_mirror()
{
    for (const std::string& ros_topic : topics_waiting_for_a_publisher) {
        const std::string type = type_in_ros_graph(ros_topic);
        if (type.empty() || !converters.contains(type))
            continue;  // not advertised yet, or a type with no NT equivalent
        const std::string nt_key = nt_topic_root + "/" + strip_leading_slash(ros_topic);
        subscriptions.push_back(converters.at(type)(ros_topic, nt_key));
    }
}

// Geometry goes out as WPILib structs, so robot code can read it directly with
// StructSubscriber<Pose3d>. The ROS header (stamp, frame_id) is not carried: NT stamps each value
// itself when it is set.
converters["geometry_msgs/msg/PoseStamped"] = [](auto ros_topic, auto nt_key) {
    auto nt_publisher = nt.GetStructTopic<frc::Pose3d>(nt_key).Publish();
    return subscribe<geometry_msgs::msg::PoseStamped>(ros_topic, [=](const auto& msg) {
        const auto& p = msg.pose.position;
        const auto& q = msg.pose.orientation;
        nt_publisher.Set(frc::Pose3d(frc::Translation3d(p.x, p.y, p.z),
                                     frc::Rotation3d(frc::Quaternion(q.w, q.x, q.y, q.z))));
    });
};

// A JointState has no single NT type, so it becomes a subtable of parallel arrays:
// <key>/names, <key>/position, <key>/velocity, <key>/effort.
converters["sensor_msgs/msg/JointState"] = [](auto ros_topic, auto nt_key) {
    auto names = nt.GetStringArrayTopic(nt_key + "/names").Publish();
    auto position = nt.GetDoubleArrayTopic(nt_key + "/position").Publish();
    return subscribe<sensor_msgs::msg::JointState>(ros_topic, [=](const auto& msg) {
        names.Set(msg.name);
        if (!msg.position.empty()) position.Set(msg.position);
        // velocity and effort follow the same pattern
    });
};

// NT -> ROS. Each declared channel names an NT key, a ROS topic and a ROS message type.
// A 10 ms timer polls every channel; of the values queued since the last poll, only the
// newest is republished.
auto mirror_double_array(const std::string& nt_key, const std::string& ros_topic)
{
    auto nt_subscriber = nt.GetDoubleArrayTopic(nt_key).Subscribe({});
    auto ros_publisher = create_publisher<std_msgs::msg::Float64MultiArray>(ros_topic);
    return [=] {
        const auto queued = nt_subscriber.ReadQueue();
        if (queued.empty())
            return;
        std_msgs::msg::Float64MultiArray msg;
        msg.data = queued.back().value;
        ros_publisher->publish(msg);
    };
}
