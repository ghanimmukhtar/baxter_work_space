#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <boost/timer.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
std::vector<double> q(7);
//files to store all deduced end effector poses
std::ofstream ee_poses;
Eigen::Vector3d angles_stracter(Eigen::Matrix4d transform_l_ee_w){
    Eigen::Vector3d current_angles;
    double Roll, Pitch, Yaw;
    Pitch = asin(transform_l_ee_w(0,2));
    if (Pitch == M_PI/2)
    {
        Yaw = 0;
        Roll = atan2(transform_l_ee_w(1,0),-transform_l_ee_w(2,0));
    }
    else if (Pitch == -M_PI/2)
    {
        Yaw = 0;
        Roll = -atan2(transform_l_ee_w(1,0),transform_l_ee_w(2,0));
    }
    else
    {
        Yaw = atan2(-transform_l_ee_w(0,1)/cos(Pitch),transform_l_ee_w(0,0)/cos(Pitch));
        Roll = atan2(-transform_l_ee_w(1,2)/cos(Pitch),transform_l_ee_w(2,2)/cos(Pitch));
    }
    current_angles << Roll,Pitch,Yaw;
    return current_angles;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "baxter_work_space");
    ros::NodeHandle node;
    //open the file
    ee_poses.open("baxter_workspace.csv");
    Eigen::VectorXd current_position(3);
    Eigen::Vector3d current_angles;
    bool continue_status = true;
    double giant_angle_step = 0.5, fine_angle_step = 0.2;
    //defining joints limits, mean values, ranges and other variables to avoid joint limits later
    Eigen::VectorXd qmin(7),qmax(7);
    qmin << -1.7016,-2.147,-3.0541,-0.05,-3.059,-1.5707,-3.059;
    qmax << 1.7016,1.047,3.0541,2.618,3.059,2.094,3.059;
    Eigen::VectorXd ee_pose(6), my_values(7);
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotState my_robot_state(robot_model);
    std::vector <std::string> variable_names(7);
    variable_names[0] = "left_s0"; variable_names[1] = "left_s1"; variable_names[2] = "left_e0";
    variable_names[3] = "left_e1"; variable_names[4] = "left_w0"; variable_names[5] = "left_w1";
    variable_names[6] = "left_w2";
    for(int i = 0; i < q.size(); i++)
        q[i] = qmin(i);
    //iterate through all possible joints values
    while(ros::ok() && continue_status){
        //move left_s0 from qmin till its max value
        while(q[0] < qmax(0)){
            //move left_s1 from qmin till its max value
            while(q[1] < qmax(1)){
                //move left_e0 from qmin till its max value
                while(q[2] < qmax(2)){
                    //move left_e1 from qmin till its max value
                    while(q[3] < qmax(3)){
                        //move left_w0 from qmin till its max value
                        while(q[4] < qmax(4)){
                            //move left_w1 from qmin till its max value
                            while(q[5] < qmax(5)){
                                //move left_w2 from qmin till its max value
                                while(q[6] < qmax(6)){
                                    //for the current joint configuration find and store end effector pose
                                    my_robot_state.setVariablePositions(variable_names,q);
                                    my_robot_state.copyJointGroupPositions(my_robot_state.getRobotModel()->getJointModelGroup("left_arm"),my_values);
                                    current_position = my_robot_state.getGlobalLinkTransform("left_gripper").translation();
                                    Eigen::Affine3d f_trans_mat;
                                    f_trans_mat = my_robot_state.getGlobalLinkTransform("left_gripper");
                                    Eigen::Matrix4d transform_l_ee_w = f_trans_mat.matrix();
                                    current_angles = angles_stracter(transform_l_ee_w);
                                    ee_pose << current_position(0), current_position(1), current_position(2), current_angles(0),  current_angles(1), current_angles(2);
                                    ee_poses << ee_pose(0) << "," << ee_pose(1) << "," << ee_pose(2) << "," << ee_pose(3) << "," << ee_pose(4) << "," << ee_pose(5) << "\n";
                                    //std::cout << "end effector pose is: " << std::endl << ee_pose << std::endl;
                                    //std::cout << "press enter to try next joints configuration ..... " << std::endl;
                                    //std::cin.ignore();
                                    //update joint left_w2 with the angle step (0.05 radian)
                                    q[6] = q[6] + fine_angle_step;
                                }
                                //update joint left_w1 with the angle step (0.05 radian)
                                q[5] = q[5] + fine_angle_step;
                                //for the new left_w1 make sure to move subsequent joints (i.e. left_w2) from minimum to max angle to scan all possibilities
                                q[6] = qmin(6);
                            }
                            //update joint left_w0 with the angle step (0.05 radian)
                            q[4] = q[4] + fine_angle_step;
                            //for the new left_w0 make sure to move subsequent joints (i.e. left_w2 and left_w1) from minimum to max angle to scan all possibilities
                            q[6] = qmin(6); q[5] = qmin(5);
                        }
                        //update joint left_e1 with the angle step (0.05 radian)
                        q[3] = q[3] + fine_angle_step;
                        //for the new left_e1 make sure to move subsequent joints (i.e. left_w2, left_w1, and left_w0) from minimum to max angle to scan all possibilities
                        q[6] = qmin(6); q[5] = qmin(5); q[4] = qmin(4);
                    }
                    //update joint left_e0 with the angle step (0.05 radian)
                    q[2] = q[2] + fine_angle_step;
                    //for the new left_e0 make sure to move subsequent joints (i.e. left_w2, left_w1, left_w0 and left_e1) from minimum to max angle to scan all possibilities
                    q[6] = qmin(6); q[5] = qmin(5); q[4] = qmin(4); q[3] = qmin(3);
                }
                //update joint left_s1 with the angle step (0.05 radian)
                q[1] = q[1] + giant_angle_step;
                //for the new left_s1 make sure to move subsequent joints (i.e. left_w2, left_w1, left_w0, left_e1, and left_e0) from minimum to max angle to scan all possibilities
                q[6] = qmin(6); q[5] = qmin(5); q[4] = qmin(4); q[3] = qmin(3); q[2] = qmin(2);
            }
            //update joint left_s0 with the angle step (0.05 radian)
            q[0] = q[0] + giant_angle_step;
            std::cout << "joint left_s0 is now: " << q[0] << std::endl;
            //for the new left_s0 make sure to move subsequent joints (i.e. left_w2, left_w1, left_w0, left_e1, left_e0 and left_s1) from minimum to max angle to scan all possibilities
            q[6] = qmin(6); q[5] = qmin(5); q[4] = qmin(4); q[3] = qmin(3); q[2] = qmin(2); q[1] = qmin(1);
        }
        //when left_s0 reach its maximum value it means the whole joint space has been explored so finish the big while loop
        continue_status = false;
    }
    ee_poses.close();
    std::cout << " ************************ I finished ******************************* " << std::endl;
    return 0;
}

