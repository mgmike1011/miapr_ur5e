#include "rclcpp/rclcpp.hpp"
#include "miapr_ur5e_interfaces/srv/cartesian_trajectory_interface.hpp"
#include "miapr_ur5e_interfaces/srv/joint_trajectory_interface.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/utils/moveit_error_code.h>


using moveit::planning_interface::MoveGroupInterface;
    
class TrajectoryControlServer : public rclcpp::Node 
{
public:
    TrajectoryControlServer() : Node("trajectory_control_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) 
    {
        RCLCPP_INFO(this->get_logger(),"### Trajectory Control Server ###");
        // this->move_group_interface = MoveGroupInterface(this, "ur_manipulator");
        this->cartesian_trajectory_service_ = this->create_service<miapr_ur5e_interfaces::srv::CartesianTrajectoryInterface>("cartesian_trajectory_service", 
        std::bind(&TrajectoryControlServer::callbackCartesianTrajectoryService, this, std::placeholders::_1, std::placeholders::_2));
        this->joint_trajectory_service_ = this->create_service<miapr_ur5e_interfaces::srv::JointTrajectoryInterface>("joint_trajectory_service", 
        std::bind(&TrajectoryControlServer::callbackJointTrajectoryService, this, std::placeholders::_1, std::placeholders::_2));
    }
     
private:
    // 
    // Members
    // 
    rclcpp::Service<miapr_ur5e_interfaces::srv::CartesianTrajectoryInterface>::SharedPtr cartesian_trajectory_service_;
    rclcpp::Service<miapr_ur5e_interfaces::srv::JointTrajectoryInterface>::SharedPtr joint_trajectory_service_;
    // 
    // Methods
    // 
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