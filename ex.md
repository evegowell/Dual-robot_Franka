ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState "{
  name: ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7','panda_finger_joint1','panda_finger_joint2'],
  position: [0.0, -0.8, 0.0, -2.2, 0.0, 2.0, 0.8, 0.04, 0.04]
}"


ros2 topic pub -1 /isaac_joint_commands sensor_msgs/msg/JointState "{
  name: ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7','panda_finger_joint1','panda_finger_joint2'],
  position: [0.5, 0.8, 1.0, -2.2, 1.0, -2.0, 0.8, 0.04, 0.04]}"
