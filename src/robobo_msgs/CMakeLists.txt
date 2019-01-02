cmake_minimum_required(VERSION 2.8.3)
project(robobo_msgs)

find_package(catkin REQUIRED COMPONENTS
	roscpp 
	rospy 
	std_msgs
	sensor_msgs
	message_generation
)

add_message_files(
	FILES
	Wheels.msg
	IRs.msg
	Tap.msg
	Fling.msg
	Led.msg
)

add_service_files(
	FILES
	MovePanTilt.srv
	MoveWheels.srv
	PlaySound.srv
	ResetWheels.srv
	SetCamera.srv
	SetEmotion.srv
	SetLed.srv
	SetSensorFrequency.srv
	Talk.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)
