// 独立桥接节点：订阅 /EEPose_cmd_rc，将增量指令通过 Airbot SDK 内置 IK 转成关节目标
// 适用于复用到其他项目：最小依赖、无 RL policy、直接走 SDK 轨迹规划/IK

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include "airbot/command/command_base.hpp"
#include "airbot/command/command_types.hpp"

#include <kdl/frames.hpp>
#include <memory>
#include <string>

using arm::Robot;

class ArmEePoseBridge {
public:
	ArmEePoseBridge(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh) {
		// 参数：URDF、CAN、末端类型、是否用规划、规划速度
		pnh_.param<std::string>("urdf_path", urdf_path_, std::string(""));
		pnh_.param<std::string>("can_interface", can_interface_, std::string("can0"));
		pnh_.param<std::string>("direction", direction_, std::string("down"));
		pnh_.param<std::string>("end_mode", end_mode_, std::string("gripper"));
		pnh_.param<std::string>("bigarm_type", bigarm_type_, std::string("OD"));
		pnh_.param<std::string>("forearm_type", forearm_type_, std::string("DM"));
		pnh_.param("joint_vel_limit", joint_vel_limit_, 3.14);
		pnh_.param("use_planning", use_planning_, true);
		pnh_.param("plan_vel", plan_vel_, 0.5);
		pnh_.param("blocking", blocking_, false);

		// rpy 映射：是否应用与现有控制器一致的(r,p,y)重排（默认关闭，按 [roll,pitch,yaw]）
		pnh_.param("swap_rpy_like_controller", swap_rpy_like_controller_, false);

		// 初始化 Airbot SDK 机器人
		try {
			arm_.reset(new Robot<6>(urdf_path_, can_interface_, direction_, joint_vel_limit_, end_mode_,
															bigarm_type_, forearm_type_, false));
			ROS_INFO("ArmEePoseBridge: Airbot Robot<6> initialized");
			arm_->online_mode();
		} catch (const std::exception &e) {
			ROS_FATAL("ArmEePoseBridge: failed to init Airbot Robot: %s", e.what());
			throw;
		}

		sub_ = nh_.subscribe("/EEPose_cmd_rc", 10, &ArmEePoseBridge::rcCallback, this);
	}

private:
	void rcCallback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
		if (!arm_)
			return;
		if (msg->data.size() < 7)
			return; // 期望至少 [dx,dy,dz, dR,dP,dY, gripper, (opt stop)]

		const double dx = msg->data[0];
		const double dy = msg->data[1];
		const double dz = msg->data[2];
		double dR = msg->data[3];
		double dP = msg->data[4];
		double dY = msg->data[5];
		const double gripper = msg->data[6]; // 0~1 建议

		// 可选：与现有控制器一致的重排（controller里做了 [R,P,Y] <- [msg[4], msg[5], msg[3]] 的等效效果）
		if (swap_rpy_like_controller_) {
			double R2 = dP, P2 = dY, Y2 = dR; // 参考 computeObservation 中 ee_rpy 拼接逻辑
			dR = R2;
			dP = P2;
			dY = Y2;
		}

		// 位姿增量 → SDK 内置 IK（相对当前规划目标叠加）
		Translation dT{dx, dy, dz};

		// rpy→四元数（相对旋转）
		KDL::Rotation R = KDL::Rotation::RPY(dR, dP, dY);
		double qx, qy, qz, qw;
		R.GetQuaternion(qx, qy, qz, qw);
		Rotation dQ{qx, qy, qz, qw};

		bool ok_t = true, ok_r = true, ok_g = true;

		if (dx != 0.0 || dy != 0.0 || dz != 0.0) {
			ok_t = arm_->add_target_relative_translation(dT, use_planning_, plan_vel_, blocking_);
			if (!ok_t)
				ROS_WARN_THROTTLE(1.0, "add_target_relative_translation failed (limit or unreachable)");
		}

		if (dR != 0.0 || dP != 0.0 || dY != 0.0) {
			ok_r = arm_->add_target_relative_rotation(dQ, use_planning_, plan_vel_, blocking_);
			if (!ok_r)
				ROS_WARN_THROTTLE(1.0, "add_target_relative_rotation failed (limit or unreachable)");
		}

		// 夹爪：0~1，直接设置
		ok_g = arm_->set_target_end(gripper, false);
		(void)ok_g;

		// 可选：处理停止指令（约定 data[7]==-1 则无操作或用户自定义）
		if (msg->data.size() >= 8 && msg->data[7] == -1.0) {
			ROS_INFO_THROTTLE(2.0, "ArmEePoseBridge: stop flag received (ignored by bridge)");
		}
	}

	ros::NodeHandle nh_, pnh_;
	ros::Subscriber sub_;
	std::unique_ptr<Robot<6>> arm_;

	// 参数
	std::string urdf_path_;
	std::string can_interface_;
	std::string direction_;
	std::string end_mode_;
	std::string bigarm_type_;
	std::string forearm_type_;
	double joint_vel_limit_ = 3.14;
	bool use_planning_ = true;
	double plan_vel_ = 0.5;
	bool blocking_ = false;
	bool swap_rpy_like_controller_ = false;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "arm_eepose_bridge");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	try {
		ArmEePoseBridge bridge(nh, pnh);
		ros::spin();
	} catch (...) {
		return 1;
	}

	return 0;
}
