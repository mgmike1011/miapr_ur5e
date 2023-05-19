#include "rclcpp/rclcpp.hpp"
#include "miapr_ur5e_interfaces/srv/cartesian_trajectory_interface.hpp"
#include "miapr_ur5e_interfaces/srv/joint_trajectory_interface.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/utils/moveit_error_code.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


using moveit::planning_interface::MoveGroupInterface;
    
class TrajectoryControlServer : public rclcpp::Node 
{
public:
    TrajectoryControlServer() : Node("trajectory_control_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) 
    {
        RCLCPP_INFO(this->get_logger(),"### Trajectory Control Server ###");

        this->cartesian_trajectory_service_ = this->create_service<miapr_ur5e_interfaces::srv::CartesianTrajectoryInterface>("cartesian_trajectory_service", 
        std::bind(&TrajectoryControlServer::callbackCartesianTrajectoryService, this, std::placeholders::_1, std::placeholders::_2));

        this->cartesian_linear_trajectory_service_ = this->create_service<miapr_ur5e_interfaces::srv::CartesianTrajectoryInterface>("cartesian_linear_trajectory_service", 
        std::bind(&TrajectoryControlServer::callbackCartesianLinearTrajectoryService, this, std::placeholders::_1, std::placeholders::_2));
        
        this->joint_trajectory_service_ = this->create_service<miapr_ur5e_interfaces::srv::JointTrajectoryInterface>("joint_trajectory_service", 
        std::bind(&TrajectoryControlServer::callbackJointTrajectoryService, this, std::placeholders::_1, std::placeholders::_2));
    }
     
private:
    // 
    // Members
    // 
    rclcpp::Service<miapr_ur5e_interfaces::srv::CartesianTrajectoryInterface>::SharedPtr cartesian_trajectory_service_;
    rclcpp::Service<miapr_ur5e_interfaces::srv::CartesianTrajectoryInterface>::SharedPtr cartesian_linear_trajectory_service_;
    rclcpp::Service<miapr_ur5e_interfaces::srv::JointTrajectoryInterface>::SharedPtr joint_trajectory_service_;
    // 
    // Methods
    // 
    /** 
     * \brief Cartesian linear trajectory service callback.
     * \param[in] request CartesianTrajectoryInterface request
     * \param[in] response CartesianTrajectoryInterface response
     * \return none. 
    */
    void callbackCartesianLinearTrajectoryService(const miapr_ur5e_interfaces::srv::CartesianTrajectoryInterface::Request::SharedPtr request,
                                                  const miapr_ur5e_interfaces::srv::CartesianTrajectoryInterface::Response::SharedPtr response){
        RCLCPP_INFO(this->get_logger(),"### Cartesian Linear Trajectory Serice ###");
        
        auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(move_group_node);
        std::thread([&executor]() { 
            executor.spin(); 
        }).detach();

        auto move_group_interface = MoveGroupInterface(move_group_node, "ur_manipulator");

        auto const target_pose = [=]
        {
            geometry_msgs::msg::Pose msg;
            msg.orientation.w = request->qw;
            msg.orientation.x = request->qx;
            msg.orientation.y = request->qy;
            msg.orientation.z = request->qz;
            msg.position.x = request->x;
            msg.position.y = request->y;
            msg.position.z = request->z;
            return msg;
        }();
        move_group_interface.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // Compute Cartesian path
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);
        // Define the intermediate waypoints for the Cartesian path
        geometry_msgs::msg::Pose middle_pose = target_pose;
        middle_pose.position.x += 0.1;
        middle_pose.position.y -= 0.1;
        middle_pose.position.z += 0.1;
        waypoints.push_back(middle_pose);

        move_group_interface.setNumPlanningAttempts(5);
        switch(request->controller){
            case 0:
                move_group_interface.setPlannerId("SBLkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "SBLkConfigDefault planner selected");
                break;
            case 1:
                move_group_interface.setPlannerId("ESTkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "ESTkConfigDefault planner selected");
                break;
            case 2:
                move_group_interface.setPlannerId("LBKPIECEkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "LBKPIECEkConfigDefault planner selected");
                break;
            case 3:
                move_group_interface.setPlannerId("BKPIECEkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "BKPIECEkConfigDefault planner selected");
                break;
            case 4:
                move_group_interface.setPlannerId("KPIECEkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "KPIECEkConfigDefault planner selected");
                break;
            case 5:
                move_group_interface.setPlannerId("RRTkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "RRTkConfigDefault planner selected");
                break;
            case 6:
                move_group_interface.setPlannerId("RRTConnectkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "RRTConnectkConfigDefault planner selected");
                break;
            case 7:
                move_group_interface.setPlannerId("RRTstarkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "RRTstarkConfigDefault planner selected");
                break;
            case 8:
                move_group_interface.setPlannerId("TRRTkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "TRRTkConfigDefault planner selected");
                break;
            case 9:
                move_group_interface.setPlannerId("PRMkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "PRMkConfigDefault planner selected");
                break;
            case 10:
                move_group_interface.setPlannerId("PRMstarkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "PRMstarkConfigDefault planner selected");
                break;    
            default:
                move_group_interface.setPlannerId("RRTConnectkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "RRTConnectkConfigDefault planner selected");
                break;
        }
        // Set the end effector trajectory constraints (optional)
        moveit_msgs::msg::Constraints trajectory_constraints;
        trajectory_constraints.name = "move_constraints";
        moveit_msgs::msg::PositionConstraint position_constraint;
        position_constraint.header.frame_id = "base_link";
        position_constraint.link_name = "end_effector";
        position_constraint.target_point_offset.x = 0.1;
        position_constraint.target_point_offset.y = 0.1;
        position_constraint.target_point_offset.z = 0.1;
        position_constraint.constraint_region.primitive_poses.push_back(target_pose);
        trajectory_constraints.position_constraints.push_back(position_constraint);
        move_group_interface.setPathConstraints(trajectory_constraints);

        // Compute the Cartesian path
        double eef_step = 0.01;  // Step size in meters
        double jump_threshold = 0.0;  // No jumping allowed between waypoints
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
        moveit::core::MoveItErrorCode mgi_status;
        if (fraction >= 0.9)  // if the Cartesian path is 90% achievable
        {
            plan.trajectory_ = trajectory;
            response->status = true;
            mgi_status = move_group_interface.execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Cartesian linear path computation failed!");
            response->status = false;
            executor.cancel();
        }
                if(mgi_status == moveit::core::MoveItErrorCode::SUCCESS){
            executor.cancel();
        }
    }
    /** 
     * \brief Joint trajectory service callback.
     * \param[in] request JointTrajectoryInterface request
     * \param[in] response JointTrajectoryInterface response
     * \return none. 
    */
    void callbackJointTrajectoryService(const miapr_ur5e_interfaces::srv::JointTrajectoryInterface::Request::SharedPtr request, 
                                        const miapr_ur5e_interfaces::srv::JointTrajectoryInterface::Response::SharedPtr response){
        RCLCPP_INFO(this->get_logger(),"### Joint Trajectory Serice ###");
        
        auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(move_group_node);
        std::thread([&executor]() { 
            executor.spin(); 
        }).detach();

        auto move_group_interface = MoveGroupInterface(move_group_node, "ur_manipulator");                                

        std::vector<double> target_joint_values = {request->j1, request->j2, request->j3, request->j4, request->j5, request->j6};

        move_group_interface.setJointValueTarget(target_joint_values);

        move_group_interface.setNumPlanningAttempts(5);
        switch(request->controller){
            case 0:
                move_group_interface.setPlannerId("SBLkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "SBLkConfigDefault planner selected");
                break;
            case 1:
                move_group_interface.setPlannerId("ESTkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "ESTkConfigDefault planner selected");
                break;
            case 2:
                move_group_interface.setPlannerId("LBKPIECEkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "LBKPIECEkConfigDefault planner selected");
                break;
            case 3:
                move_group_interface.setPlannerId("BKPIECEkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "BKPIECEkConfigDefault planner selected");
                break;
            case 4:
                move_group_interface.setPlannerId("KPIECEkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "KPIECEkConfigDefault planner selected");
                break;
            case 5:
                move_group_interface.setPlannerId("RRTkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "RRTkConfigDefault planner selected");
                break;
            case 6:
                move_group_interface.setPlannerId("RRTConnectkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "RRTConnectkConfigDefault planner selected");
                break;
            case 7:
                move_group_interface.setPlannerId("RRTstarkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "RRTstarkConfigDefault planner selected");
                break;
            case 8:
                move_group_interface.setPlannerId("TRRTkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "TRRTkConfigDefault planner selected");
                break;
            case 9:
                move_group_interface.setPlannerId("PRMkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "PRMkConfigDefault planner selected");
                break;
            case 10:
                move_group_interface.setPlannerId("PRMstarkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "PRMstarkConfigDefault planner selected");
                break;    
            default:
                move_group_interface.setPlannerId("RRTConnectkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "RRTConnectkConfigDefault planner selected");
                break;
        }

        auto const [success, plan] = [&move_group_interface]
        {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();
        moveit::core::MoveItErrorCode mgi_status;
        if (success)
        {
            response->status = true;
            mgi_status = move_group_interface.execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planing failed!");
            response->status = false;
            executor.cancel();
        }
        if(mgi_status == moveit::core::MoveItErrorCode::SUCCESS){
            executor.cancel();
        }
    }
    /** 
     * \brief Cartesian trajectory service callback.
     * \param[in] request CartesianTrajectoryInterface request
     * \param[in] response CartesianTrajectoryInterface response
     * \return none. 
    */
    void callbackCartesianTrajectoryService(const miapr_ur5e_interfaces::srv::CartesianTrajectoryInterface::Request::SharedPtr request,
                                            const miapr_ur5e_interfaces::srv::CartesianTrajectoryInterface::Response::SharedPtr response){
        RCLCPP_INFO(this->get_logger(),"### Cartesian Trajectory Serice ###");
        
        auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(move_group_node);
        std::thread([&executor]() { 
            executor.spin(); 
        }).detach();

        auto move_group_interface = MoveGroupInterface(move_group_node, "ur_manipulator");
        
        auto const target_pose = [=]
        {
            geometry_msgs::msg::Pose msg;
            msg.orientation.w = request->qw;
            msg.orientation.x = request->qx;
            msg.orientation.y = request->qy;
            msg.orientation.z = request->qz;
            msg.position.x = request->x;
            msg.position.y = request->y;
            msg.position.z = request->z;
            return msg;
        }();
        move_group_interface.setPoseTarget(target_pose);
        move_group_interface.setNumPlanningAttempts(5);
        switch(request->controller){
            case 0:
                move_group_interface.setPlannerId("SBLkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "SBLkConfigDefault planner selected");
                break;
            case 1:
                move_group_interface.setPlannerId("ESTkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "ESTkConfigDefault planner selected");
                break;
            case 2:
                move_group_interface.setPlannerId("LBKPIECEkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "LBKPIECEkConfigDefault planner selected");
                break;
            case 3:
                move_group_interface.setPlannerId("BKPIECEkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "BKPIECEkConfigDefault planner selected");
                break;
            case 4:
                move_group_interface.setPlannerId("KPIECEkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "KPIECEkConfigDefault planner selected");
                break;
            case 5:
                move_group_interface.setPlannerId("RRTkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "RRTkConfigDefault planner selected");
                break;
            case 6:
                move_group_interface.setPlannerId("RRTConnectkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "RRTConnectkConfigDefault planner selected");
                break;
            case 7:
                move_group_interface.setPlannerId("RRTstarkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "RRTstarkConfigDefault planner selected");
                break;
            case 8:
                move_group_interface.setPlannerId("TRRTkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "TRRTkConfigDefault planner selected");
                break;
            case 9:
                move_group_interface.setPlannerId("PRMkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "PRMkConfigDefault planner selected");
                break;
            case 10:
                move_group_interface.setPlannerId("PRMstarkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "PRMstarkConfigDefault planner selected");
                break;    
            default:
                move_group_interface.setPlannerId("RRTConnectkConfigDefault");
                RCLCPP_INFO(this->get_logger(), "RRTConnectkConfigDefault planner selected");
                break;
        }
        auto const [success, plan] = [&move_group_interface]
        {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();
        moveit::core::MoveItErrorCode mgi_status;
        if (success)
        {
            response->status = true;
            mgi_status = move_group_interface.execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planing failed!");
            response->status = false;
            executor.cancel();
        }
        if(mgi_status == moveit::core::MoveItErrorCode::SUCCESS){
            executor.cancel();
        }
    }
};
     
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryControlServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}