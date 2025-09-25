# Recording and replaying data with ROS 2

To keep a record of the experiments conducted, and to save the system's topics in a database, you can use the rosbag2 tool, which is integrated into ROS 2 by default. You need to place yourself in a directory of your choice, a `bag_files` folder for example, and run the command: ``ros2 bag record topic_name``. If you don't specify the topic name(s), all the topics in the system will be recorded. Of course, it's also possible to record system frames, using the `/tf` topic.

```{important}
When registering topics, if one of the topics is registered with a custom message type, for example, and the message format is subsequently modified, it will not be possible to listen to the topics, as the format will not allow correct subscription to the corresponding topics.
```

To replay the data, always from the directory containing the records, you can replay the data by typing the command followed by the name of the bag file: ``ros2 bag play bag_file_name``.

```{tip}
For bag file naming, ROS 2 defaults to ``rosbag2``, followed by the full date and time. We recommend keeping this part of the name and adding the name of the experiment as a suffix. For example: ``rosbag2_2022_11_08-17_23_08_turn_at_left``.
```