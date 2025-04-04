set(MQTT_HOST 10.0.0.6)

add_definitions(-DROBOT_TOPIC_CMD_VEL="@new mqtt @coder msgpack @host ${MQTT_HOST} @topic cmd_vel")
add_definitions(-DROBOT_TOPIC_SCAN="@new mqtt @coder msgpack @host ${MQTT_HOST} @topic scan")
add_definitions(-DROBOT_TOPIC_ODOM="@new mqtt @coder msgpack @host ${MQTT_HOST} @topic odom")

add_definitions(-DROBOT_TOPIC_CAMERA_RGB="@new mqtt @coder msgpack @host ${MQTT_HOST} @topic camera_rgb")
add_definitions(-DROBOT_TOPIC_CAMERA_DEPTH="@new mqtt @coder msgpack @host ${MQTT_HOST} @topic camera_depth")

add_definitions(-DROBOT_TOPIC_TTS="@new mqtt @coder msgpack @host ${MQTT_HOST} @topic tts")
add_definitions(-DROBOT_TOPIC_STT="@new mqtt @coder msgpack @host ${MQTT_HOST} @topic stt")