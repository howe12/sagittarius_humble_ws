#!/usr/bin/env bash
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH

#=================================================
#	System Required: Ubuntu 20.04
#        litian.zhuang@nxrobo.com
#	Site: http://www.nxrobo.com/
#    技术讨论与反馈QQ群：6646169  8346256
#=================================================

sh_ver="2.0"
filepath=$(cd "$(dirname "$0")"; pwd)
Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Yellow_background_prefix="\033[43;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
Error="${Red_font_prefix}[错误]${Font_color_suffix}"
Warn="${Yellow_font_prefix}[警告]${Font_color_suffix}"
Tip="${Green_font_prefix}[注意]${Font_color_suffix}"
Separator_1="——————————————————————————————"
Version=$(lsb_release -r --short)
Codename=$(lsb_release -c --short)
OSDescription=$(lsb_release -d --short)
OSArch=$(uname -m)
ARM_TYPE="sgr532"

#检查系统要求
check_sys(){
		if [[ "${Version}" == "22.04" ]]; then
                ROS_Ver="humble"
        else
                echo -e "${Error} sagittarius暂不支持当前系统 ${OSDescription} !" && exit 1
        fi
}

#检查设备连接
# check_dev(){
# 	#检查机械臂
# 	if [ ! -n "$(lsusb -d 2e88:4603)" ]; then
# 		echo -e "${Error} 机械臂usb没有正确连接，请确认正确连接！！"
# 	fi
# }

check_dev(){
    # 检查机械臂设备节点
    if [ ! -e "/dev/sagittarius_0" ]; then
        echo -e "${Error} 机械臂设备节点 /dev/sagittarius 未找到，请确认正确连接且进行了软链接！！"
    fi
}


#检测是否需要安装完整版
check_install_ros_full(){
	ROSVER=${ROS_DISTRO}
	if [ "${ROS_DISTRO}" == "humble" ]; then
		echo -e "${Tip} 检测到当前系统已安装了ROS的${ROS_DISTRO}版本!" 
	
	else
		echo -e "${Tip} 检测到未安装任何ROS版本!请先安装ROS"
	fi 
}

#安装sagittarius依赖库
install_sagittarius_require(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	echo -e "${Info} 准备安装sagittarius相关驱动……"
	${PROJECTPATH}/src/sagittarius_arm_ros/install.sh
	echo -e "${Info} 完成依赖库安装……"
}


#编译sagittarius
compile_sagittarius(){
	source /opt/ros/${ROS_Ver}/setup.bash
	colcon build
}




#远程设置
master_uri_setup(){
	eth_ip=`/sbin/ifconfig eth0|grep 'inet '|awk '{print $2}'`
	wlp1s_ip=`/sbin/ifconfig wlp1s0|grep 'inet '|awk '{print $2}'`
	wlp2s_ip=`/sbin/ifconfig wlp2s0|grep 'inet '|awk '{print $2}'`
	wlan_ip=`/sbin/ifconfig wlan0|grep 'inet '|awk '{print $2}'`
        enp3s_ip=`/sbin/ifconfig enp3s0|grep 'inet '|awk '{print $2}'`
	if [ $eth_ip ]; then
		echo -e "${Info}使用有线网络eth0" 
		local_ip=$eth_ip
	elif [ $wlp1s_ip ]; then
		echo -e "${Info}使用无线网络wlp1s0" 
	  	local_ip=$wlp1s_ip
	elif [ $wlp2s_ip ]; then
		echo -e "${Info}使用无线网络wlp2s0" 
	  	local_ip=$wlp2s_ip
	elif [ $wlan_ip ]; then
		echo -e "${Info}使用无线网络wlan0" 
	  	local_ip=$wlan_ip
        elif [ $enp3s_ip ]; then
                echo -e "${Info}使用无线网络enp3s0" 
                local_ip=$enp3s_ip	
	fi
	export ROS_HOSTNAME=$local_ip
	export ROS_MASTER_URI="http://${local_ip}:11311"
	echo -e "${Info}Using ROS MASTER at ${Red_font_prefix}$ROS_MASTER_URI${Font_color_suffix} from ${Red_font_prefix}$ROS_HOSTNAME${Font_color_suffix}"
}

print_command()
{
	echo -e "${Yellow_background_prefix}${Red_font_prefix}${1}${Font_color_suffix}"
}

start_sagittarius_rviz()
{

	echo -e "${Info}" 
	echo -e "${Info} 启动ROS RVIZ仿真界面，请选择：" 
	echo -e "${Info} 1、通过joint_state_publisher界面控制虚拟机械臂" 
	echo -e "${Info} 2、通过joint_state_publisher界面控制实体机械臂" 
	echo -e "${Info} 退出请输入：Ctrl + c        "
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo && stty erase ^? && read -p "请输入数字 [1或者2]：" number && stty erase ^?
	case "$number" in
		1)
		start_sagittarius_rviz_simulation
		;;
		2)
		start_sagittarius_rviz_actual
		;;
		*)
		echo -e "${Error} 非在选范围内，退出"
		return
		;;
	esac
}

#start_sagittarius_rviz_simulation
start_sagittarius_rviz_simulation()
{
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	print_command "ros2 launch sagittarius_descriptions sagittarius_description.launch.py robot_model:=${ARM_TYPE} use_joint_pub_gui:=true"
	ros2 launch sagittarius_descriptions sagittarius_description.launch.py robot_model:=${ARM_TYPE} use_joint_pub_gui:=true
}
#start_sagittarius_rviz_actual
start_sagittarius_rviz_actual()
{
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	print_command "ros2 launch sdk_sagittarius_arm sagittarius_description_true.launch.py robot_model:=${ARM_TYPE} use_joint_pub_gui:=true"
	ros2 launch sdk_sagittarius_arm sagittarius_description_true.launch.py robot_model:=${ARM_TYPE} use_joint_pub_gui:=true
}

# 软连接机械臂
symbolic_link_sag()
{
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	print_command "sudo python3 ${PROJECTPATH}/src/Sag_Bringup/sdk_sagittarius_arm/scripts/sym_link_sag.py"
	sudo python3 ${PROJECTPATH}/src/Sag_Bringup/sdk_sagittarius_arm/scripts/sym_link_sag.py
}

# 软连接USB相机
symbolic_link_usb_cam()
{
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	print_command "sudo python3 ${PROJECTPATH}/src/Sag_Vision/usb_cam/scripts/sym_link_usbcam.py"
	sudo python3 ${PROJECTPATH}/src/Sag_Vision/usb_cam/scripts/sym_link_usbcam.py
}

# 相机标定
cam_calibration(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	# 设置等待回车开始
	echo -e "${Info} 执行前请先打印好相机标定板"
	echo && stty erase ^? && read -p "输入回车执行任务："
	# 启动第一个终端 - 运行 usb_cam
	gnome-terminal --title=USB相机终端 -- bash -c "source ${PROJECTPATH}/install/setup.bash; ros2 launch usb_cam camera.launch.py; exec bash"

	# 启动第二个终端 - 运行 camera_calibration
	gnome-terminal --title=标定程序终端 -- bash -c "source ${PROJECTPATH}/install/setup.bash; ros2 run camera_calibration cameracalibrator --ros-args --remap image:=/camera1/image_raw --remap camera:=/usb_cam; exec bash"
}

# 生成apriltag码
generate_apriltag(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info} 前往https://chev.me/arucogen/生成图片"
	echo -e "${Info} 请确定好生成的图片大小，长宽比，码的大小，码的类型" 
	echo -e "${Info} 请确定好生成的图片保存路径"
	echo -e "${Info} 生成好的图片请进行打印，粘贴在机械臂的上"
	echo && stty erase ^? && read -p "输入回车前往网址："
	# 自动登陆网址
	gnome-terminal -- bash -c "xdg-open 'https://chev.me/arucogen/'; exec bash"
}

# 眼在手上标定
eye_in_hand_calibration(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info} 请确保机械臂与USB相机已进行软链接"
	echo -e "${Info} 请将打印好标定使用的aruco标定板" 
	echo -e "${Info} 标定板需平整地贴在墙面上" 
	echo -e "${Info} 标定时点按机械臂的向下键位释放力矩" 
	echo -e "${Info} 标定时通过拖动机械臂进行标定，确保图像中包含标定板" 
	echo -e "${Info} 标定过程中需要至少20组标定数据"
	echo && stty erase ^? && read -p "输入回车执行任务："
	print_command "ros2 launch sagittarius_cam_calibration sag_rs_camera_calibration.launch.py "
	ros2 launch sagittarius_cam_calibration sag_rs_camera_calibration.launch.py 
}

# 眼在手外标定
eye_out_hand_calibration(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info} 请确保生成好的apirltag图片粘贴在机械臂的上"
	echo -e "${Info} 请确定已连接好RGBD相机" 
	echo -e "${Info} 标定过程中需要至少20组标定数据"
	echo -e "${Info} 生成好的图片请进行打印，粘贴在机械臂的上"
	echo && stty erase ^? && read -p "输入回车执行任务："
	# 启动第一个终端 - 运行 rgbd相机
	print_command “ ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true ”
	gnome-terminal --title=rgbd相机终端 -- bash -c "source ${PROJECTPATH}/install/setup.bash; ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true ; exec bash"

	print_command "ros2 launch sagittarius_cam_calibration sag_apriltag_camera_calibration.launch.py robot_model:=${ARM_TYPE} "
	ros2 launch sagittarius_cam_calibration sag_apriltag_camera_calibration.launch.py robot_model:=${ARM_TYPE}
	# gnome-terminal --title=标定程序 -- bash -c "source ${PROJECTPATH}/install/setup.bash; ros2 launch sagittarius_cam_calibration sag_apriltag_camera_calibration.launch.py robot_model:=${ARM_TYPE}; exec bash"
}

# 颜色标定
color_calibration(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info} 1、眼在手上的颜色标定" 
	echo -e "${Info} 2、眼在手外的颜色标定" 
	echo -e "${Info} 退出请输入：Ctrl + c        "
	echo && stty erase ^? && read -p "请输入数字 [1或者2]：" number
	case "$number" in
		1)
		print_command "ros2 launch sagittarius_py_demo hsv_roi.launch.py camera_type:=usb_cam"
		ros2 launch sagittarius_py_demo hsv_roi.launch.py camera_type:=usb_cam
		;;
		2)
		print_command "ros2 launch sagittarius_py_demo hsv_roi.launch.py camera_type:=d435"
		ros2 launch sagittarius_py_demo hsv_roi.launch.py camera_type:=d435
		;;
		*)
		echo -e "${Error} 非在选范围内，退出"
		return
		;;
	esac
}

preparation(){
	echo -e "${Info}" 
	echo -e "${Info} 请选择需要执行的准备操作：" 
	echo -e "${Info} 1、软连接机械臂" 
	echo -e "${Info} 2、软连接USB相机" 
	echo -e "${Info} 3、USB相机标定" 
	echo -e "${Info} 4、apriltag码生成" 
	echo -e "${Info} 5、眼在手上标定" 
	echo -e "${Info} 6、眼在手外标定" 
	echo -e "${Info} 7、颜色标定" 
	echo -e "${Info} 退出请输入：Ctrl + c        "
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo && stty erase ^? && read -p "请输入对应功能数字：" number
	case "$number" in
		1)
		symbolic_link_sag
		;;
		2)
		symbolic_link_usb_cam
		;;
		3)
		cam_calibration
		;;
		4)
		generate_apriltag
		;;
		5)
		eye_in_hand_calibration
		;;
		6)
		eye_out_hand_calibration
		;;
		7)
		color_calibration
		;;
		*)
		return
		;;
	esac
}


start_sagittarius_MoveIt()
{

	echo -e "${Info}" 
	echo -e "${Info} 启动ROS MoveIt仿真界面，请选择：" 
	echo -e "${Info} 1、通过MoveIt控制虚拟机械臂" 
	echo -e "${Info} 2、通过MoveIt控制实体机械臂" 
	echo -e "${Info} 退出请输入：Ctrl + c        "
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo && stty erase ^? && read -p "请输入数字 [1或者2]：" number
	case "$number" in
		1)
		start_sagittarius_MoveIt_simulation
		;;
		2)
		start_sagittarius_MoveIt_actual
		;;
		*)
		echo -e "${Error} 非在选范围内，退出"
		return
		;;
	esac
}

#start_sagittarius_MoveIt_actual
start_sagittarius_MoveIt_actual()
{
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	print_command "ros2 launch sagittarius_moveit demo_true.launch.py robot_model:=${ARM_TYPE} device_type:=actual"
	ros2 launch sagittarius_moveit demo_true.launch.py robot_model:=${ARM_TYPE} device_type:=actual
}

#start_sagittarius_MoveIt_simulation
start_sagittarius_MoveIt_simulation()
{
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	print_command "ros2 launch sagittarius_humble_moveit demo.launch.py"
	ros2 launch sagittarius_humble_moveit demo.launch.py
}


#Forward_kinematics
Forward_kinematics(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}正运动学示例" 
	echo -e "${Info}" 
	echo -e "${Info}请选择相关选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 只查看源代码
	  ${Green_font_prefix}2.${Font_color_suffix} 只运行示例
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-3]：" asrnum
	case "$asrnum" in
		1)
		cat  ${PROJECTPATH}/src/Sag_Demo/sagittarius_py_demo/sagittarius_py_demo/simple_fk_planning.py
		;;
		2)
		print_command "ros2 launch sagittarius_py_demo simple_fk_planning.launch.py "
		ros2 launch sagittarius_py_demo simple_fk_planning.launch.py 
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}

#Inverse_kinematics
Inverse_kinematics(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}逆运动学示例" 
	echo -e "${Info}" 
	echo -e "${Info}请选择相关选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 只查看源代码
	  ${Green_font_prefix}2.${Font_color_suffix} 只运行示例
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-3]：" asrnum
	case "$asrnum" in
		1)
		cat  ${PROJECTPATH}/src/Sag_Demo/sagittarius_py_demo/sagittarius_py_demo/simple_ik_planning.py
		;;
		2)
		print_command "ros2 launch sagittarius_py_demo simple_ik_planning.launch.py "
		ros2 launch sagittarius_py_demo simple_ik_planning.launch.py 
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}

#hsv_catch
hsv_catch(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}颜色识别抓取示例" 
	echo -e "${Info}" 
	echo -e "${Info}请选择相关选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 颜色标定（如果还没有进行颜色标定的情况）
	  ${Green_font_prefix}2.${Font_color_suffix} 眼在手上标定（如果需要执行眼在手上抓取的情况）
	  ${Green_font_prefix}3.${Font_color_suffix} 眼在手外标定（如果需要执行眼在手外抓取的情况）
	  ${Green_font_prefix}4.${Font_color_suffix} 眼在手上的识别抓取
	  ${Green_font_prefix}5.${Font_color_suffix} 眼在手外的识别抓取
	  ${Green_font_prefix}6.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-3]：" asrnum
	case "$asrnum" in
		1)
		color_calibration
		;;
		2)
		eye_in_hand_calibration
		;;
		3)
		eye_out_hand_calibration
		;;
		4)
		print_commandq "ros2 launch sagittarius_py_demo hsv_catch.launch.py"
		ros2 launch sagittarius_py_demo hsv_catch.launch.py 
		;;
		5)
		# 启动第一个终端 - 运行 rgbd相机
		print_command “ ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true ”
		gnome-terminal --title=rgbd相机终端 -- bash -c "source ${PROJECTPATH}/install/setup.bash; ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true ; exec bash"

		print_command "ros2 launch sagittarius_py_demo eye_out_hsv_catch.launch.py  robot_model:=${ARM_TYPE} "
		ros2 launch sagittarius_py_demo eye_out_hsv_catch.launch.py 	
		# gnome-terminal --title=标定程序 -- bash -c "ros2 launch sagittarius_py_demo eye_out_hsv_catch.launch.py ; exec bash"
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}

yolo_catch(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}"
	echo -e "${Info}yolo识别抓取示例"
	echo -e "${Info}"
	echo -e "${Info}请选择相关选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 眼在手外标定
	  ${Green_font_prefix}2.${Font_color_suffix} 物体识别抓取
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c"
	echo && stty erase ^? && read -p "请输入数字 [1-3]：" asrnum
	case "$asrnum" in
		1)
		eye_out_hand_calibration
		;;
		2)
		# 启动第一个终端 - 运行 rgbd相机
		print_command “ ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true ”
		gnome-terminal --title=rgbd相机终端 -- bash -c "source ${PROJECTPATH}/install/setup.bash; ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true ; exec bash"

		print_command "ros2 launch sagittarius_py_demo yolo_catch.launch.py  robot_model:=${ARM_TYPE} "
		ros2 launch sagittarius_py_demo yolo_catch.launch.py
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}

#rosbag record and play
rosbag_record_play(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}rosbag录制与播放" 
	echo -e "${Info}" 
	echo -e "${Info}请选择相关选择：
	  ${Green_font_prefix}1.${Font_color_suffix} rosbag录制机械臂动作
	  ${Green_font_prefix}2.${Font_color_suffix} rosbag播放录制的机械臂动作
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-3]：" asrnum
	case "$asrnum" in
		1)
		rosbag_record
		;;
		2)
		rosbag_play
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}
#rosbag record 
rosbag_record(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}rosbag录制机械臂动作" 
	echo -e "${Info}请用手扶位机械臂至初始位置，" 
	echo -e "${Info}启动后移动机械臂各个关节，" 
	echo -e "${Info}完成录制后,按Ctrl + c退出,自动保存，" 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "输入设置的包名,按回车键(Enter)继续,可直接回车运行,默认包名为test:" bagname
	bagname=${bagname:-test}
	print_command "ros2 launch sagittarius_puppet_control puppet_control_record.launch.py "
	ros2 launch sagittarius_puppet_control puppet_control_record.launch.py bag_name:=${bagname}

}
#rosbag_play
rosbag_play(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}rosbag播放录制的机械臂动作" 
	echo -e "${Info}请注意安全" 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "输入播放的包名,按回车键(Enter)继续,可直接回车运行,默认包名为test:" bagname
	bagname=${bagname:-test}
	print_command "ros2 launch sagittarius_puppet_control puppet_control_play.launch.py "
	ros2 launch sagittarius_puppet_control puppet_control_play.launch.py bag_name:=${bagname}

}

#puppet_control
puppet_control(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}多机械臂动作实时复现" 
	echo -e "${Info}请注意保持各个机械臂的空间距离" 
	echo -e "${Info}请确保主机械臂的USB线最先与主机相连" 	
	echo -e "${Info}然后执行软链接，按照上述操作，/dev/ttyACM0为主机械臂" 
	echo -e "${Info}" 	
	echo -e "${Info}请选择相关选择：
	${Green_font_prefix}1.${Font_color_suffix} 机械臂软链接
	${Green_font_prefix}2.${Font_color_suffix} 多机械臂动作实时复现" 
	echo && stty erase ^? && read -p "请执行1进行软链接后，再执行2进行动作控制：" asrnum
	case "$asrnum" in
		1)
		symbolic_link_sag
		;;
		2)
		print_command "ros2 launch sagittarius_puppet_control puppet_control.launch.py "
		ros2 launch sagittarius_puppet_control puppet_control.launch.py  
		;;
		*)
		echo -e "${Error} 错误，请输入正确的命令"
		;;
	esac
}

drawstar_and_display(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}" 
	echo -e "${Info} 机械臂画五角星同步仿真显示" 
	echo -e "${Info} 程序启动后会新开一个终端，需要开始运行时在新终端按回车键" 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）继续：" 
	print_command "ros2 launch cobot_draw draw_five_stars_true.launch.py"
	ros2 launch cobot_draw draw_five_stars_true.launch.py
}

draw(){
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}" 
	echo -e "${Info}绘画功能示例" 
	echo -e "${Info}" 
	echo -e "${Info}请选择相关选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 机械臂画正方形同步仿真显示
	  ${Green_font_prefix}2.${Font_color_suffix} 机械臂画圆同步仿真显示
	  ${Green_font_prefix}3.${Font_color_suffix} 机械臂画五角星同步仿真显示
	  ${Green_font_prefix}4.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-3]：" asrnum
	case "$asrnum" in
		1)
		print_command "ros2 launch cobot_draw draw_square_true.launch.py"
		ros2 launch cobot_draw draw_square_true.launch.py
		;;
		2)
		print_command "ros2 launch cobot_draw draw_circle_true.launch.py"
		ros2 launch cobot_draw draw_circle_true.launch.py
		;;
		3)
		print_command "ros2 launch cobot_draw draw_five_stars_true.launch.py"
		ros2 launch cobot_draw draw_five_stars_true.launch.py
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}

qrcode_transfer_files(){
	eno1_ip=`/sbin/ifconfig eno1|grep 'inet '|awk '{print $2}'`
	eth_ip=`/sbin/ifconfig eth0|grep 'inet '|awk '{print $2}'`
	wlp1s_ip=`/sbin/ifconfig wlp1s0|grep 'inet '|awk '{print $2}'`
	wlp2s_ip=`/sbin/ifconfig wlp2s0|grep 'inet '|awk '{print $2}'`
	wlan_ip=`/sbin/ifconfig wlan0|grep 'inet '|awk '{print $2}'`
	enp3s_ip=`/sbin/ifconfig enp3s0|grep 'inet '|awk '{print $2}'`
	wlp0s_ip=`/sbin/ifconfig wlp0s20f3|grep 'inet '|awk '{print $2}'`
	wlo1_ip=`/sbin/ifconfig wlo1|grep 'inet '|awk '{print $2}'`
	if [ $wlp1s_ip ]; then
		echo -e "${Info}使用无线网络wlp1s0" 
	  	net_interface="wlp1s0"
	elif [ $wlo1_ip ]; then
		echo -e "${Info}使用无线网络wlo1" 
	  	net_interface="wlo1"	  	
	elif [ $wlp2s_ip ]; then
		echo -e "${Info}使用无线网络wlp2s0" 
	  	net_interface="wlp2s0"
	elif [ $wlan_ip ]; then
		echo -e "${Info}使用无线网络wlan0" 
	  	net_interface="wlan0"
	elif [ $enp3s_ip ]; then
		echo -e "${Info}使用无线网络enp3s0" 
		net_interface="enp3s0"
	elif [ $wlp0s_ip ]; then
		echo -e "${Info}使用无线网络wlp0s20f3" 
		net_interface="wlp0s20f3"		
	elif [ $eth_ip ]; then
		echo -e "${Info}使用无线网络eth0" 
		net_interface="eth0"
	elif [ $eno1_ip ]; then
		echo -e "${Info}使用无线网络eno1" 
		net_interface="eno1"
	fi	
	
	echo -e "${Info}" 
	echo -e "${Info}通过局域网收发文件" 
	echo -e "${Info}" 
	echo -e "${Info}请选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 发送文件（文件名，带上文件绝对路径）
	  ${Green_font_prefix}2.${Font_color_suffix} 接收文件（默认存放在~/Downloads路径中）
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" cnum
	case "$cnum" in
		1)
		echo -e "${Info}请输入文件名，带上文件绝对路径，如 /home/${USER}/a.jpg：
		 退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入要发送的文件：" s_file
		if [ -f "$s_file" ]; then
			echo -e "${Info}本机即将发送文件：${Green_font_prefix}"$s_file"${Font_color_suffix}，请接收端扫码或者直接输入下面的网址接收文件"
		else 
			echo -e "${Info}请输入带绝对路径的文件名"
			exit
		fi
		
		qrcp send  -i $net_interface $s_file
		;;
		2)
		echo -e "${Info}请输入接收到的文件存放的路径，默认为 /home/${USER}/Downloads：
		退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入文件存放的文件夹路径：" s_file
		if [ -d "$s_file" ]; then
			echo ""
		else 
			echo -e "${Info}${Red_font_prefix}文件夹不存在，将存放在默认文件夹/home/${USER}/Downloads中${Font_color_suffix}"
			s_file="/home/${USER}/Downloads"
		fi
		echo -e "${Info}接收的文件将存放在：${Green_font_prefix}"$s_file"${Font_color_suffix}，目录下，请发送端扫码或者直接输入下面的网址选择文件发送"
		qrcp  -i $net_interface receive --output=$s_file
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}

coming_soon(){
	echo -e "${Tip} coming_soon!" 
}


#printf
menu_status(){
	echo -e "${Tip} 当前系统版本 ${OSDescription} !" 
	ROSVER=$ROS_DISTRO
	if [ $ROSVER ]; then
		echo -e "${Tip} 当前ROS版本 ${ROSVER} !"
		return
	fi 
	echo -e "${Error} 未检测到ROS版本，请先安装ROS！可以选择102直接安装。" 
}

tell_us(){
	echo -e ""
	echo -e "${Tip} --------------分隔线----------------" 
	echo -e "${Tip} 网址：www.nxrobo.com" 
	echo -e "${Tip} ROS技术讨论与反馈QQ群：一群 8346256(已满)；二群 6646169" 
	echo -e "${Tip} ---------QQ扫描加入我们--------------"
	echo 'https://jq.qq.com/?_wv=1027&k=1JV8oyB8'|qrencode -o - -t UTF8
	echo -e ""
}

qrcode_picture()
{
	echo 'www.NXROBO.com'|qrencode -o - -t UTF8
}
check_sys
echo -e "————————————
  Sagittarius 一键管理脚本"
qrcode_picture

echo -e "  
  请根据右侧的功能说明选择相应的序号。
  注意：101与103为相关环境的安装与设置，如果已执行过，不要再重复执行。
  
  ${Green_font_prefix}  0.${Font_color_suffix} 单独编译
————————————
  ${Green_font_prefix}  1.${Font_color_suffix} 通过RVIZ界面控制
  ${Green_font_prefix}  2.${Font_color_suffix} 通过MOVEIT界面控制
  ${Green_font_prefix}  3.${Font_color_suffix} 正运动学示例
  ${Green_font_prefix}  4.${Font_color_suffix} 逆运动学示例
  ${Green_font_prefix}  5.${Font_color_suffix} rosbag录制与播放机械臂动作
  ${Green_font_prefix}  6.${Font_color_suffix} 多机械臂动作实时复现
  ${Green_font_prefix}  7.${Font_color_suffix} 基于HSV的颜色识别抓取
  ${Green_font_prefix}  8.${Font_color_suffix} 基于yolo的物体识别抓取
  ${Green_font_prefix}  9.${Font_color_suffix} 机械臂绘画同步仿真显示\c"


echo -e "
————————————

  ${Green_font_prefix}100.${Font_color_suffix} 问题反馈
  ${Green_font_prefix}101.${Font_color_suffix} 准备工作
  ${Green_font_prefix}103.${Font_color_suffix} 安装依赖库
  ${Green_font_prefix}104.${Font_color_suffix} 文件传输
 "
menu_status
check_dev
echo && stty erase ^? && read -p "请输入数字：" num && stty erase ^?
case "$num" in
	0)
	compile_sagittarius	
	;;
	1)
	start_sagittarius_rviz
	;;
	2)
	start_sagittarius_MoveIt
	;;
	3)
	Forward_kinematics
	;;
	4)
	Inverse_kinematics
	;;
	5)
	rosbag_record_play
	;;
	6)
	puppet_control
	;;
	7)
	hsv_catch
	;;
	8)
	yolo_catch
	;;
	9)
	draw
	;;
	10)
	3d_perception_detection_and_grab
	;;
	20)
	PROJECTPATH=$(cd `dirname $0`; pwd)
	${PROJECTPATH}/src/sagittarius_arm_ros/3rd_app/expand/expand.sh ${PROJECTPATH}
	;;
	100)
	tell_us
	;;
	101)
	preparation
	;;
	103)
	install_sagittarius_require
	;;
	104)
	qrcode_transfer_files
	;;	
	*)
	echo -e "${Error} 请输入正确的数字 "
	;;
esac

