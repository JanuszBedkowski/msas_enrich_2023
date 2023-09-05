#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <nav_msgs/Path.h>
#include "geometry_msgs/Twist.h"
#include <std_msgs/String.h>

#include <laszip/laszip_api.h>
#include <iostream>
 
#include <GL/freeglut.h>

#include "imgui.h"
#include "imgui_impl_glut.h"
#include "imgui_impl_opengl2.h"
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <memory>
#include <deque>
#include <mutex>
#include <deque>

#include <Eigen/Eigen>

#include "structures.h"
#include "transformations.h"

#include <Fusion.h>

#include <transformations.h>
#include <point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <relative_pose_tait_bryan_wc_jacobian.h>
#include <chrono>
//#include <smoothness_tait_bryan_wc_jacobian.h>
#include <point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>

#include <iostream>
#include <chrono>
#include <ctime>
#include <string>
#include <sys/stat.h>

#include <nlohmann/json.hpp>

#define SAMPLE_PERIOD (1.0 / 200.0)

typedef std::map<unsigned long long int, Bucket> BucketMap;

const unsigned int window_width = 640;
const unsigned int window_height = 480;
int mouse_buttons = 0;
int mouse_old_x, mouse_old_y;
bool gui_mouse_down{false};
float rotate_x = 0.0, rotate_y = 0.0;
float translate_z = -30.0;
float translate_x, translate_y = 0.0;

struct Point3Di
{
    Eigen::Vector3d point;
	double timestamp;
    float intensity;
    int index_pose;
};

enum MissionType{
    none,
    abort_mission,
    single_goal_forward,
    mission_forward,
    mission_backward
};

struct DemoOdometryParameters{
    int major = 0;
    int minor = 0;
    bool is_gui = true;
    std::string livox_config_file = "mid360_config_lio.json";
    float pc_filter_length = 1.0;
    float pc_filter_width = 0.5;
    int number_max_points = 10000;
	int number_max_points_init = 10000;

    bool fusionConventionNwu = true;
    bool fusionConventionEnu = false;
    bool fusionConventionNed = false;
    bool use_motion_from_previous_step = true;

    FusionEuler euler;
    float calib_height_above_ground = 0.2;

    bool compensate_roll_pitch_with_imu = true;
	double main_loop_time_execution = 0.0;

	double dec_bucket_x = 0.3;
	double dec_bucket_y = 0.3;
	double dec_bucket_z = 0.3;

	int decimated_points_size = 0;
	int points_size = 0;
	int number_of_iterations = 10;

	bool init = true;
	//double imu_yaw_prev = 0.0;
    int max_current_points = 1000000;

    double rot_speed = 0.1;//0.5;
    double rot_speed_slow = 0.01;//0.2;
    double forward_speed_slow = 0.1;//0.2;
    double forward_speed = 0.3;
    double forward_speed_fast = 0.5;//1.0;
    double backward_speed_slow = 0.01;
    double speed_boost = 1.0;//1.5;

    double rot_speed_tmp = rot_speed;
    double rot_speed_slow_tmp = rot_speed;
    double forward_speed_slow_tmp = forward_speed_slow;
    double forward_speed_tmp = forward_speed;
    double forward_speed_fast_tmp = forward_speed_fast;
    double backward_speed_slow_tmp = backward_speed_slow;


    std::string folder1 = "";
    std::string folder2 = "";

    float robot_lidar_offset = -0.2;

    int counter_fail = 0;
};

DemoOdometryParameters params;
std::mutex points_lock;
std::mutex imu_lock;
std::mutex buckets_lock;
std::deque<Point3Di> points;
FusionAhrs ahrs;
std::map<unsigned long long int, Bucket> buckets;
std::vector<Eigen::Affine3d> trajectory;

std::mutex current_points_lock;
std::deque<Point3Di> current_points;

std::mutex last_points_lock;
std::vector<Point3Di> last_points;

std::mutex mission_path_lock;
Eigen::Affine3d mission_goal;
MissionType mission_type = none;


//publishers
ros::Publisher pc_current_pub;
ros::Publisher current_map_pub;
ros::Publisher pub_current_robot_pose;
ros::Publisher pub_vel;
ros::Publisher pub_current_mission_goal;
ros::Publisher pub_debug_message;

struct Goal{
    Eigen::Affine3d pose;
    bool stop_scan = false;
};

std::mutex mission_lock;
std::vector<Goal> mission;
int mission_next_goal_index = 0;



//pub_path = nh.advertise< nav_msgs::Path > ("path", 1);
bool save_buckets(BucketMap &buckets, const std::string &path);
bool load_buckets(BucketMap &buckets, const std::string &path);

void update_rgd(Eigen::Vector3d b, std::map<unsigned long long int, Bucket> &buckets,
                std::vector<Point3Di>& points_global);

bool initGL(int *argc, char **argv);
void display();
void reshape(int w, int h);
void mouse(int glut_button, int state, int x, int y);
void motion(int x, int y);
void idle(void);
void main_loop(bool render);
std::vector<Point3Di> decimate(const std::vector<Point3Di> &points, double bucket_x, double bucket_y, double bucket_z);
void optimize(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory, 
    std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
    Eigen::Vector3d b, std::map<unsigned long long int, Bucket> &buckets);

unsigned long long int get_index(const int16_t x, const int16_t y, const int16_t z){
    return ((static_cast<unsigned long long int>(x) << 32) & (0x0000FFFF00000000ull)) |
           ((static_cast<unsigned long long int>(y) << 16) & (0x00000000FFFF0000ull)) |
           ((static_cast<unsigned long long int>(z) << 0 ) & (0x000000000000FFFFull));
}

unsigned long long int get_rgd_index(const Eigen::Vector3d p, const Eigen::Vector3d b)
{
    int16_t x = static_cast<int16_t>(p.x() / b.x());
    int16_t y = static_cast<int16_t>(p.y() / b.y());
    int16_t z = static_cast<int16_t>(p.z() / b.z());
    return get_index(x, y, z);
}

void abortCallback(const std_msgs::Bool::ConstPtr& msg)
{
    std::cout << "ABORT" << std::endl;
    mission_type = MissionType::none;

    geometry_msgs::Twist twist;
    twist.angular.z= 0;
    twist.linear.x = 0.0;
    pub_vel.publish(twist);

    //exit(1);
}

void getCurrentPointCloudCallback(const std_msgs::Int32::ConstPtr& msg_nr_of_point)
{
    std::cout << "getCurrentPointCloudCallback" << std::endl;
    int nr_points = msg_nr_of_point->data;
    if(nr_points > params.max_current_points){
        nr_points = params.max_current_points;
    }
    //exit(1);


    Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
    {
        std::lock_guard<std::mutex> lck(buckets_lock);
		if(trajectory.size() > 0){
			current_pose = trajectory[trajectory.size() - 1];
        }
    }

    {
        std::lock_guard<std::mutex> lck(current_points_lock);

        pcl::PointCloud<pcl::PointXYZI>::Ptr msg(new pcl::PointCloud<pcl::PointXYZI>);

        size_t point_count = 0;

        msg->header.frame_id = "odom";
        msg->height = 1;
        //msg->width = point_count;
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);

        //for (size_t i=0;i<point_count;++i)

        int index_begin = current_points.size() - nr_points;
        if(index_begin < 0){
            index_begin = 0;
        }
        //for(const auto &point:current_points)
        for(int i = index_begin; i < current_points.size(); i++)
        {
            const auto &point = current_points[i];
            pcl::PointXYZI p;
            auto pp = current_pose * point.point;
            p.x = pp.x();
            p.y = pp.y();
            p.z = pp.z();
            p.intensity = point.intensity;
            msg->points.push_back(p);
            point_count++;
        }
        msg->width = point_count;

        std::cout << "publish current pc: [" << point_count << " points]" << std::endl;
        pc_current_pub.publish(*msg);
    }
}

void getLastPointCloudCallback(const std_msgs::Int32::ConstPtr& msg_nr_of_point)
{
    std::cout << "getLastPointCloudCallback" << std::endl;
    int nr_points = msg_nr_of_point->data;
    if(nr_points > params.max_current_points){
        nr_points = params.max_current_points;
    }
    //exit(1);


    //Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
    //{
    //    std::lock_guard<std::mutex> lck(buckets_lock);
	//	if(trajectory.size() > 0){
	//		current_pose = trajectory[trajectory.size() - 1];
    //    }
    //}

    {
        std::lock_guard<std::mutex> lck(last_points_lock);

        pcl::PointCloud<pcl::PointXYZI>::Ptr msg(new pcl::PointCloud<pcl::PointXYZI>);

        size_t point_count = 0;

        msg->header.frame_id = "odom";
        msg->height = 1;
        //msg->width = point_count;
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);

        //for (size_t i=0;i<point_count;++i)

        int index_begin = last_points.size() - nr_points;
        if(index_begin < 0){
            index_begin = 0;
        }
        //for(const auto &point:current_points)
        for(int i = index_begin; i < last_points.size(); i++)
        {
            const auto &point = last_points[i];
            pcl::PointXYZI p;
            auto pp = point.point;
            p.x = pp.x();
            p.y = pp.y();
            p.z = pp.z();
            p.intensity = point.intensity;
            msg->points.push_back(p);
            point_count++;
        }
        msg->width = point_count;

        std::cout << "publish current pc: [" << point_count << " points]" << std::endl;
        pc_current_pub.publish(*msg);
    }
}




void getCurrentMapCallback(const std_msgs::Bool::ConstPtr& msg){
    std::cout << "getCurrentMapCallback" << std::endl;
    
    {
        std::lock_guard<std::mutex> lck(buckets_lock);

        pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);

        size_t point_count = 0;

        msg->header.frame_id = "odom";
        msg->height = 1;
        //msg->width = point_count;
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);

        //for (size_t i=0;i<point_count;++i)
        for(const auto &b:buckets)
        {
            pcl::PointXYZ p;
            p.x = b.second.mean.x();
            p.y = b.second.mean.y();
            p.z = b.second.mean.z();
            msg->points.push_back(p);
            point_count++;
        }
        msg->width = point_count;

        std::cout << "publish current map: [" << point_count << " buckets]" << std::endl;
        current_map_pub.publish(*msg);
    }
}

void resetCallback(const std_msgs::Bool::ConstPtr& msg){
    std::cout << "resetCallback" << std::endl;
    params.init = true;
}

void getCalibHeightAboveGround(const std_msgs::Float32::ConstPtr& msg)
{
    params.calib_height_above_ground = msg->data;
    params.init = true;
}

void getSingleGoalForward(const nav_msgs::Path::Ptr &msg){
    params.counter_fail = 0;
    std::lock_guard<std::mutex> lck(mission_path_lock);

    //stop robot
    geometry_msgs::Twist twist;
    twist.angular.z= 0;
    twist.linear.x = 0.0;
    pub_vel.publish(twist);

    mission_goal = Eigen::Affine3d::Identity();
    
    mission_goal.translation().x() = msg->poses[0].pose.position.x;
    mission_goal.translation().y() = msg->poses[0].pose.position.y;
    mission_goal.translation().z() = msg->poses[0].pose.position.z;

    Eigen::Quaterniond q;
    q.w() = msg->poses[0].pose.orientation.w;
    q.x() = msg->poses[0].pose.orientation.x;
    q.y() = msg->poses[0].pose.orientation.y;
    q.z() = msg->poses[0].pose.orientation.z;

    mission_goal.linear() = q.toRotationMatrix();
    mission_type = MissionType::single_goal_forward;
}

void multiple_goals_to_robotCallback(const nav_msgs::Path::Ptr &msg){
    params.counter_fail = 0;
    {
        std::lock_guard<std::mutex> lck(mission_lock);
        mission.clear();
        int ss = msg->poses.size();
        //printf("-----------------------------------------");
        //printf("msg->poses.size() %d\n", msg->poses.size());
        //printf("msg->poses.size() %d\n", msg->poses.size());
        //printf("msg->poses.size() %d\n", msg->poses.size());
        //printf("msg->poses.size() %d\n", msg->poses.size());
        //printf("-----------------------------------------");


        //mission.resize(ss);

        std_msgs::String d_message;
        //std::string string_message = "--------------------------" + std::to_string(msg->poses.size());
        std::string string_message = "msg->poses.size() " + std::to_string(msg->poses.size());
        //std_msgs::String d_message;
        d_message.data = string_message;
        pub_debug_message.publish(d_message);

        for(int i = 0; i < ss; i++){
            Eigen::Affine3d goal;
            goal.translation().x() = msg->poses[i].pose.position.x;
            goal.translation().y() = msg->poses[i].pose.position.y;
            goal.translation().z() = 0.0;//msg->poses[i].pose.position.z;

            Eigen::Quaterniond q;
            q.w() = msg->poses[0].pose.orientation.w;
            q.x() = msg->poses[0].pose.orientation.x;
            q.y() = msg->poses[0].pose.orientation.y;
            q.z() = msg->poses[0].pose.orientation.z;

            goal.linear() = q.toRotationMatrix();

            Goal g;
            g.pose = goal;
            g.stop_scan = false;

            //if(g.pose.translation().norm() > 0.1){
            mission.push_back(g);
        }
    }
}

void multiple_goals_to_robot_executeCallback(const std_msgs::Int32::Ptr &msg){
    params.counter_fail = 0;
    {
        std::lock_guard<std::mutex> lck(mission_lock);
        int num_scans_from_end = msg->data;

        if(mission.size() >= num_scans_from_end){
            for(int i = 0; i < num_scans_from_end; i++){
                mission[i].stop_scan = false;
            }
            for(int i = 0; i < num_scans_from_end; i++){
                if(mission.size() - 1 - i >= 0){
                    mission[mission.size() - 1 - i].stop_scan = true;
                }
            }
        }
        mission_next_goal_index = 0;
        mission_type = mission_forward;
    }
}

void project_gui(DemoOdometryParameters &paramters)
{
	if(ImGui::Begin(std::string("Project demo odometry v" + std::to_string(paramters.major) + "." + std::to_string(paramters.minor)).c_str())){
	    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::Text("main_loop_time_execution %.3f ms", params.main_loop_time_execution);
        ImGui::Text("points_size [%d] decimated_points_size [%d]", params.points_size, params.decimated_points_size);
        
		ImGui::InputInt("number_of_iterations", &params.number_of_iterations);
		if(params.number_of_iterations < 1){
			params.number_of_iterations = 1;
		}
		ImGui::InputInt("number_max_points", &params.number_max_points);
		if(params.number_max_points < 100){
			params.number_max_points = 100;
		}
		ImGui::InputInt("number_max_points_init", &params.number_max_points_init);
		if(params.number_max_points_init < 100){
			params.number_max_points_init = 100;
		}
		

		ImGui::InputFloat("calib_height_above_ground", &params.calib_height_above_ground);

		if(ImGui::Button("reset")){
			params.init = true;
		}

		ImGui::Checkbox("Compensate roll pitch with imu", &params.compensate_roll_pitch_with_imu);
        {
            std::lock_guard<std::mutex> lck(imu_lock);
            std::string roll = "roll: " + std::to_string(params.euler.angle.roll);
            ImGui::Text(roll.c_str());
            std::string pitch = "pitch: " + std::to_string(params.euler.angle.pitch);
            ImGui::Text(pitch.c_str());
            std::string yaw = "yaw: " + std::to_string(params.euler.angle.yaw);
            ImGui::Text(yaw.c_str());
        }

        ImGui::End();
    }
}

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
	if (data == nullptr) {
        return;
    }

	static std::vector<Eigen::Vector3d> points_to_save;

    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
        Eigen::Affine3d m_cal = Eigen::Affine3d::Identity();
        {

            Eigen::Affine3d m_roll_pitch = Eigen::Affine3d::Identity();
            TaitBryanPose tb_pose;
            {
                std::lock_guard<std::mutex> lck(imu_lock);
                
                tb_pose.px = 0.0;
                tb_pose.py = 0.0;
                tb_pose.pz = params.calib_height_above_ground;
                tb_pose.om = params.euler.angle.roll / 180.0 * M_PI;
                tb_pose.fi = params.euler.angle.pitch / 180.0 * M_PI;
                tb_pose.ka = 0.0;
                m_roll_pitch = affine_matrix_from_pose_tait_bryan(tb_pose);
            }

			std::lock_guard<std::mutex> lck(points_lock);
            m_cal = m_roll_pitch;


			for (uint32_t i = 0; i < data->dot_num; i++) {
				const auto & p = p_point_data[i];

                Eigen::Vector3d pt(p.x * 0.001, p.y * 0.001, p.z * 0.001);
				//if(fabs(pt.x()) < params.pc_filter_length * 0.5 && fabs(pt.y()) < params.pc_filter_width * 0.5){
				//}else{
                    if(params.compensate_roll_pitch_with_imu){
                        Point3Di new_p;
						new_p.intensity = p.reflectivity;
						new_p.point = m_cal * pt;
						new_p.index_pose = 1;
						points.push_back(new_p);
                    }else{
						Point3Di new_p;
						new_p.intensity = p.reflectivity;
						new_p.point = pt;
						new_p.index_pose = 1;
                        points.push_back(new_p);
                    }
				//}
				if(params.init){
					while (points.size() > params.number_max_points_init){
						points.pop_front();
					}
				}else{
					while (points.size() > params.number_max_points){
						points.pop_front();
					}
				}
				
			}
        }

        
        {
            std::lock_guard<std::mutex> lck(current_points_lock);
            for (uint32_t i = 0; i < data->dot_num; i++) {
				const auto & p = p_point_data[i];

                Eigen::Vector3d pt(p.x * 0.001, p.y * 0.001, p.z * 0.001);
				if(fabs(pt.x()) < params.pc_filter_length * 0.5 && fabs(pt.y()) < params.pc_filter_width * 0.5){
				}else{
                    if(params.compensate_roll_pitch_with_imu){
                        Point3Di new_p;
						new_p.intensity = p.reflectivity;
						new_p.point = m_cal * pt;
						new_p.index_pose = 1;
						current_points.push_back(new_p);
                        //std::cout << m_cal.matrix() << std::endl;
                    }else{
						Point3Di new_p;
						new_p.intensity = p.reflectivity;
						new_p.point = pt;
						new_p.index_pose = 1;
                        current_points.push_back(new_p);
                    }
				}
				
                while (current_points.size() > params.max_current_points){
                    current_points.pop_front();
                }
				
				
			}

            //std::mutex current_points_lock;
            //std::deque<Point3Di> current_points;

        }
    }
    else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
        LivoxLidarCartesianLowRawPoint *p_point_data = (LivoxLidarCartesianLowRawPoint *)data->data;
    } else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
        LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint *)data->data;
    }
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
    if (data == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lck(imu_lock);

    LivoxLidarImuRawPoint* p_imu_data = (LivoxLidarImuRawPoint*)data->data;
   
    const FusionVector gyroscope = {
        static_cast<float>((*p_imu_data).gyro_x * 180.0 / M_PI), 
        static_cast<float>((*p_imu_data).gyro_y * 180.0 / M_PI), 
        static_cast<float>((*p_imu_data).gyro_z * 180.0 / M_PI)};
    const FusionVector accelerometer = {(*p_imu_data).acc_x, (*p_imu_data).acc_y, (*p_imu_data).acc_z};
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

    FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);

    Eigen::Quaterniond d {quat.element.w,quat.element.x,quat.element.y,quat.element.z};
    Eigen::Affine3d t {Eigen::Matrix4d::Identity()};
    t.rotate(d);
    params.euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

//std::string roll = "roll: " + std::to_string(params.euler.angle.roll);
//            ImGui::Text(roll.c_str());
//            std::string pitch = "pitch: " + std::to_string(params.euler.angle.pitch);

    
    if(fabs(params.euler.angle.roll) > 60 || fabs(params.euler.angle.pitch) > 60){
        geometry_msgs::Twist twist;
        twist.angular.z= 0;
        twist.linear.x = 0;
        pub_vel.publish(twist);
        params.counter_fail ++;
        if(params.counter_fail > 10){
            mission_type = MissionType::abort_mission;
        }
    }

}

void pub_save_bucketsCallback(const std_msgs::Int32::Ptr &msg)
{
    std::cout << "pub_save_bucketsCallback" << std::endl;
    save_buckets(buckets, "buckets.json");
}

void pub_load_bucketsCallback(const std_msgs::Int32::Ptr &msg)
{
    std::cout << "pub_load_bucketsCallback" << std::endl;
    {
        std::lock_guard<std::mutex> lck(buckets_lock);
        trajectory.clear();
        buckets.clear();
        
        Eigen::Affine3d initial_pose = Eigen::Affine3d::Identity();
        initial_pose(2,3) = params.calib_height_above_ground;

        trajectory.push_back(initial_pose);
        
        //update_rgd(Eigen::Vector3d(0.3, 0.3, 0.3), buckets, decimated_points);
        load_buckets(buckets, "buckets.json");
    }
}

int main(int argc, char *argv[]){
    if(argc != 5){
        std::cout << "USAGE #1: " << argv[0] << " config_file_name 0 - non gui folder1 folder2" << std::endl;
        std::cout << "USAGE #2: " << argv[0] << " config_file_name 1 - gui folder1 folder2" << std::endl;
        return 1;
    }

    params.is_gui = atoi(argv[2]);
    params.folder1 = argv[3];
    params.folder2 = argv[4];

    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);

    // Convert the time to a string with the desired format
    std::tm* timeinfo = std::localtime(&time);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", timeinfo);
    std::string directoryName = buffer;

    //fs::path p1(params.folder1);

    //p1 /= directoryName;

    //std::cout << directoryName << std::endl;

    std::string new_dir1 = params.folder1 + "/" + directoryName;

    int result = mkdir(new_dir1.c_str(), 0777);

    if(result != 0){
        std::cout << "Failed to crate directory!!! '" << new_dir1 << "'" << std::endl;
        exit(1);
    }

    std::string new_dir2 = params.folder2 + "/" + directoryName;

    result = mkdir(new_dir2.c_str(), 0777);

    if(result != 0){
        std::cout << "Failed to crate directory!!! '" << new_dir2 << "'" << std::endl;
        exit(1);
    }

    params.folder1 = new_dir1;
    params.folder2 = new_dir2;
    
    // Create the new directory
    //if (fs::create_directory(directoryName)) {
    //    std::cout << "Directory created: " << directoryName << std::endl;
    //} else {
    //    std::cout << "Failed to create directory" << std::endl;
    //}






    FusionAhrsInitialise(&ahrs);
    if(params.fusionConventionNwu){
        ahrs.settings.convention = FusionConventionNwu;
    } 
    if(params.fusionConventionEnu){
        ahrs.settings.convention = FusionConventionEnu;
    } 
    if(params.fusionConventionNed){
        ahrs.settings.convention = FusionConventionNed;
    } 

    params.major = 0;
	params.minor = 1;

    if (!LivoxLidarSdkInit(argv[1])) {
		printf("Livox Init Failed\n");
		LivoxLidarSdkUninit();
		return 1;
	}
	
	SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
    //std::cout << "start sleeping 5 seconds" << std::endl;
    //sleep(5);
    //std::cout << "wake up" << std::endl;
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);

    if(params.is_gui){
		if (false == initGL(&argc, (char **)argv)) {
			return 4;
		}
		glutDisplayFunc(display);
		glutMouseFunc(mouse);
		glutMotionFunc(motion);
		glutIdleFunc(idle);
		glutMainLoop();
	}else{
        ros::init(argc, argv, "mandeye");
        ros::NodeHandle nh;
        //ros::Publisher pc_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI> > ("trajectory_as_cloud", 1);
        //ros::Publisher pc_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI> > ("buckets_map", 1);
        
        pc_current_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI> > ("current_point_cloud", 1);
        current_map_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("current_map", 1);
        pub_current_robot_pose = nh.advertise<nav_msgs::Path>  ("current_robot_pose", 1);
        pub_vel = nh.advertise< geometry_msgs::Twist >("cmd_vel", 1);
        pub_current_mission_goal = nh.advertise<nav_msgs::Path>  ("current_mission_goal", 1);
        pub_debug_message = nh.advertise<std_msgs::String>  ("debug_message", 1);
        
        //
        ros::Subscriber sub_get_current_pc = nh.subscribe("get_current_pc", 1, getCurrentPointCloudCallback);
        ros::Subscriber sub_get_current_map = nh.subscribe("get_current_map", 1, getCurrentMapCallback);
        ros::Subscriber sub_reset = nh.subscribe("reset_jackal", 1, resetCallback);
        ros::Subscriber sub_get_calib_height_above_ground = nh.subscribe("calib_height_above_ground", 1, getCalibHeightAboveGround);
        ros::Subscriber subub_single_goal_forward = nh.subscribe("single_goal_forward", 1, getSingleGoalForward);
        ros::Subscriber sub_abort_mission = nh.subscribe("abort_mission", 1, abortCallback);
        ros::Subscriber sub_multiple_goals_to_robot = nh.subscribe("multiple_goals_to_robot", 1, multiple_goals_to_robotCallback);
        ros::Subscriber sub_multiple_goals_to_robot_execute = nh.subscribe("multiple_goals_to_robot_execute", 1, multiple_goals_to_robot_executeCallback);

        ros::Subscriber sub_get_last_goal_pc = nh.subscribe("get_last_goal_pc", 1, getLastPointCloudCallback);

        ros::Subscriber sub_save_buckets = nh.subscribe("pub_save_buckets", 1, pub_save_bucketsCallback);
        ros::Subscriber sub_load_buckets = nh.subscribe("pub_load_buckets", 1, pub_load_bucketsCallback);


        ros::Rate loop_rate(1);

        while (ros::ok())
        {
            main_loop(false);
            ros::spinOnce();
        }
	}
	return 0;
}

bool initGL(int *argc, char **argv) {
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(window_width, window_height);
    glutCreateWindow("Lidar odometry demo");
    glutDisplayFunc(display);
    glutMotionFunc(motion);
    glutReshapeFunc(reshape);

    // default initialization
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);

    // viewport
    glViewport(0, 0, window_width, window_height);

    // projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat) window_width / (GLfloat) window_height, 0.01,
                   10000.0);

    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); //(void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    ImGui::StyleColorsDark();
    ImGui_ImplGLUT_Init();
    ImGui_ImplGLUT_InstallFuncs();
    ImGui_ImplOpenGL2_Init();
    return true;
}

void display() {
    ImGuiIO& io = ImGui::GetIO();
	glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
	glTranslatef(translate_x, translate_y, translate_z);
	glRotatef(rotate_x, 1.0, 0.0, 0.0);
	glRotatef(rotate_y, 0.0, 0.0, 1.0);

	main_loop(true);

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();

	{
		std::lock_guard<std::mutex> lck(buckets_lock);
		if(trajectory.size() > 0){
			pose = trajectory[trajectory.size() - 1];
		//	update_rgd(Eigen::Vector3d(0.3, 0.3, 0.3), buckets, decimated_points);
			glPointSize(2);
			glColor3f(1,0,0);
			glBegin(GL_POINTS);
			for(const auto &b:buckets){
				glVertex3f(b.second.mean.x(), b.second.mean.y(), b.second.mean.z());
			}
			glEnd();
			glPointSize(1);

			glColor3f(0,1,0);
			glBegin(GL_LINE_STRIP);
				for(const auto &p:trajectory){
					glVertex3f(p.translation().x(), p.translation().y(), p.translation().z());
				}
			glEnd();
		}
	}

	{
	std::lock_guard<std::mutex> lck(points_lock);
		glBegin(GL_POINTS);
		glColor3f(0.0f, 1.0f, 1.0f);
		for (auto &p : points){
			auto point = pose * p.point;
			
			glVertex3f(point.x(), point.y(), point.z());
		}
		glEnd();
	}


    glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(10.0f, 0.0f, 0.0f);

        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 10.0f, 0.0f);

        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 10.0f);
	glEnd();

    ImGui_ImplOpenGL2_NewFrame();
	ImGui_ImplGLUT_NewFrame();
	project_gui(params);
	ImGui::Render();
	ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

	glutSwapBuffers();
	glutPostRedisplay();
}

void mouse(int glut_button, int state, int x, int y) {
    ImGuiIO& io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);
    int button = -1;
    if (glut_button == GLUT_LEFT_BUTTON) button = 0;
    if (glut_button == GLUT_RIGHT_BUTTON) button = 1;
    if (glut_button == GLUT_MIDDLE_BUTTON) button = 2;
    if (button != -1 && state == GLUT_DOWN)
        io.MouseDown[button] = true;
    if (button != -1 && state == GLUT_UP)
        io.MouseDown[button] = false;

    if (!io.WantCaptureMouse)
    {
        if (state == GLUT_DOWN) {
            mouse_buttons |= 1 << glut_button;
        } else if (state == GLUT_UP) {
            mouse_buttons = 0;
        }
        mouse_old_x = x;
        mouse_old_y = y;
    }
}

void motion(int x, int y) {
    ImGuiIO& io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);

    if (!io.WantCaptureMouse)
    {
        float dx, dy;
        dx = (float) (x - mouse_old_x);
        dy = (float) (y - mouse_old_y);
        gui_mouse_down = mouse_buttons>0;
        if (mouse_buttons & 1) {
            rotate_x += dy * 0.2f;
            rotate_y += dx * 0.2f;
        } else if (mouse_buttons & 4) {
            translate_z += dy * 0.05f;
        } else if (mouse_buttons & 3) {
            translate_x += dx * 0.05f;
            translate_y -= dy * 0.05f;
        }
        mouse_old_x = x;
        mouse_old_y = y;
    }
    glutPostRedisplay();
}

void reshape(int w, int h) {
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat) w / (GLfloat) h, 0.01, 10000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void idle(void)
{
	glutPostRedisplay();
}
//std::vector<Point3Di> decimate(const std::vector<Point3Di> &points, double bucket_x, double bucket_y, double bucket_z)

bool exportLaz(const std::string &filename,
               const std::vector<Point3Di> &pointcloud)
{

    constexpr float scale = 0.0001f; // one tenth of milimeter
    // find max
    Eigen::Vector3d max(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
    Eigen::Vector3d min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    for (auto &p : pointcloud)
    {
        max.x() = std::max(max.x(), p.point.x());
        max.y() = std::max(max.y(), p.point.y());
        max.z() = std::max(max.z(), p.point.z());

        min.x() = std::min(min.x(), p.point.x());
        min.y() = std::min(min.y(), p.point.y());
        min.z() = std::min(min.z(), p.point.z());
    }

    // create the writer
    laszip_POINTER laszip_writer;
    if (laszip_create(&laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: creating laszip writer\n");
        return false;
    }

    // get a pointer to the header of the writer so we can populate it

    laszip_header *header;

    if (laszip_get_header_pointer(laszip_writer, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip writer\n");
        return false;
    }

    // populate the header

    header->file_source_ID = 4711;
    header->global_encoding = (1 << 0); // see LAS specification for details
    header->version_major = 1;
    header->version_minor = 2;
    //    header->file_creation_day = 120;
    //    header->file_creation_year = 2013;
    header->point_data_format = 1;
    header->point_data_record_length = 0;
    header->number_of_point_records = pointcloud.size();
    header->number_of_points_by_return[0] = pointcloud.size();
    header->number_of_points_by_return[1] = 0;
    header->point_data_record_length = 28;
    header->x_scale_factor = scale;
    header->y_scale_factor = scale;
    header->z_scale_factor = scale;

    header->max_x = max.x();
    header->min_x = min.x();
    header->max_y = max.y();
    header->min_y = min.y();
    header->max_z = max.z();
    header->min_z = min.z();

    // optional: use the bounding box and the scale factor to create a "good" offset
    // open the writer
    laszip_BOOL compress = (strstr(filename.c_str(), ".laz") != 0);

    if (laszip_open_writer(laszip_writer, filename.c_str(), compress))
    {
        fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", filename.c_str());
        return false;
    }

    fprintf(stderr, "writing file '%s' %scompressed\n", filename.c_str(), (compress ? "" : "un"));

    // get a pointer to the point of the writer that we will populate and write

    laszip_point *point;
    if (laszip_get_point_pointer(laszip_writer, &point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
        return false;
    }

    laszip_I64 p_count = 0;
    laszip_F64 coordinates[3];

    for (int i = 0; i < pointcloud.size(); i++)
    {
        const auto &p = pointcloud[i];
        point->intensity = p.intensity;
        p_count++;
        coordinates[0] = p.point.x();
        coordinates[1] = p.point.y();
        coordinates[2] = p.point.z();
        if (laszip_set_coordinates(laszip_writer, coordinates))
        {
            fprintf(stderr, "DLL ERROR: setting coordinates for point %I64d\n", p_count);
            return false;
        }

        // p.SetIntensity(pp.intensity);

        // if (i < intensity.size()) {
        //     point->intensity = intensity[i];
        // }
        // laszip_set_point

        if (laszip_write_point(laszip_writer))
        {
            fprintf(stderr, "DLL ERROR: writing point %I64d\n", p_count);
            return false;
        }
    }

    if (laszip_get_point_count(laszip_writer, &p_count))
    {
        fprintf(stderr, "DLL ERROR: getting point count\n");
        return false;
    }

    fprintf(stderr, "successfully written %I64d points\n", p_count);

    // close the writer

    if (laszip_close_writer(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: closing laszip writer\n");
        return false;
    }

    // destroy the writer

    if (laszip_destroy(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
        return false;
    }

    std::cout << "exportLaz DONE" << std::endl;
    return true;
}

void save_laz_files()
{
    //params.folder1 = argv[3];
    //params.folder2 = argv[4];

    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);

    // Convert the time to a string with the desired format
    std::tm* timeinfo = std::localtime(&time);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", timeinfo);
    std::string fileName = buffer;

    sleep(1);

    std::string pc1_fn1 = params.folder1 + "/" + fileName + "_high_resolution.laz";
    std::string pc1_fn2 = params.folder1 + "/" + fileName + "_small_resolution.laz";
    std::string pc1_fn3 = params.folder1 + "/" + fileName + "_navigation.laz";

    std::string pc2_fn1 = params.folder2 + "/" + fileName + "_high_resolution.laz";
    std::string pc2_fn2 = params.folder2 + "/" + fileName + "_small_resolution.laz";
    std::string pc2_fn3 = params.folder2 + "/" + fileName + "_navigation.laz";

    std::vector<Point3Di> pointcloud1;
    std::vector<Point3Di> pointcloud2;
    std::vector<Point3Di> pointcloud3;
    
    Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
    {
        std::lock_guard<std::mutex> lck(buckets_lock);
		if(trajectory.size() > 0){
			current_pose = trajectory[trajectory.size() - 1];
        }
    }

    {
        std::lock_guard<std::mutex> lck(current_points_lock);
        
        int index_begin = current_points.size() - 100000;
        if(index_begin < 0){
            index_begin = 0;
        }
        for(int i = index_begin; i < current_points.size(); i++)
        {
            const auto point = current_points[i];
            pointcloud1.push_back(point);
        }

        for(auto &p:pointcloud1){
            p.point = current_pose * p.point;
        }

        pointcloud2 = decimate(pointcloud1, 0.03, 0.03, 0.03);

        for(int i = 0; i < pointcloud2.size(); i++){
            if(pointcloud2[i].point.z() < 1){
                pointcloud3.push_back(pointcloud2[i]);
            }
        }
    }
    exportLaz(pc1_fn1, pointcloud1);
    exportLaz(pc1_fn2, pointcloud2);
    exportLaz(pc1_fn3, pointcloud3);

    exportLaz(pc2_fn1, pointcloud1);
    exportLaz(pc2_fn2, pointcloud2);
    exportLaz(pc2_fn3, pointcloud3);

    {
        std::lock_guard<std::mutex> lck(last_points_lock);
        last_points = pointcloud1;
    }

}

void main_loop(bool render){
	auto start = std::chrono::steady_clock::now();

	std::vector<Point3Di> all_points;
	{
	    std::lock_guard<std::mutex> lck(points_lock);
	    all_points = std::vector<Point3Di>(points.begin(), points.end());
	}
	params.points_size = all_points.size();
    if(all_points.size() < 100){
        return;
    }
	std::vector<Point3Di> decimated_points = decimate(all_points, params.dec_bucket_x, params.dec_bucket_y, params.dec_bucket_z);
	params.decimated_points_size = decimated_points.size();

	if(params.init){//} && all_points.size() == params.number_max_points_init ){
        if(all_points.size() >= params.number_max_points_init ){
            {
                std::lock_guard<std::mutex> lck(buckets_lock);
                trajectory.clear();
                buckets.clear();
                
                Eigen::Affine3d initial_pose = Eigen::Affine3d::Identity();
                initial_pose(2,3) = params.calib_height_above_ground;

                trajectory.push_back(initial_pose);
                for(auto &p:decimated_points){
                    p.point = initial_pose * p.point;
                }
                update_rgd(Eigen::Vector3d(0.3, 0.3, 0.3), buckets, decimated_points);
            }
            params.init = false;
        }
		
	}else{
		{
			std::lock_guard<std::mutex> lck(buckets_lock);
            if(trajectory.size() == 0){
                std::cout << "trajectory.size() == 0" << std::endl;
                return;
            }

		 	auto pose = trajectory[trajectory.size() - 1];
			auto diff_pose = Eigen::Affine3d::Identity();

			//if(trajectory.size() > 2){
			//	diff_pose = trajectory[trajectory.size() - 2].inverse() * trajectory[trajectory.size() - 1];
			//}

			std::vector<Eigen::Affine3d> intermediate_trajectory;
			intermediate_trajectory.push_back(pose);
			intermediate_trajectory.push_back(pose);// * diff_pose);

    		std::vector<Eigen::Affine3d> intermediate_trajectory_motion_model;
			intermediate_trajectory_motion_model.push_back(pose);
			intermediate_trajectory_motion_model.push_back(pose);// * diff_pose);

			for(int iter = 0 ; iter < params.number_of_iterations; iter++){
				optimize(decimated_points, intermediate_trajectory, intermediate_trajectory_motion_model,
    			Eigen::Vector3d(0.3, 0.3, 0.3), buckets);
			}
			intermediate_trajectory[1](2,3) = params.calib_height_above_ground;
			//trajectory.push_back(intermediate_trajectory[1]);
			trajectory.push_back(trajectory[trajectory.size()-1] * (intermediate_trajectory[0].inverse()*intermediate_trajectory[1]));
			for(auto &p:decimated_points){
				p.point = trajectory[trajectory.size()-1] * p.point;
			}

			update_rgd(Eigen::Vector3d(0.3, 0.3, 0.3), buckets, decimated_points);

            //send robot current pose
            static int count = 0;
            count++;
            if(count % 10 == 0){ 
                nav_msgs::Path path;
                path.header.frame_id = "odom";
                path.header.stamp = ros::Time::now();
                path.poses.resize(1);
                auto & p = path.poses[0];
                p.pose.position.x = trajectory[trajectory.size()-1].translation().x();
                p.pose.position.y = trajectory[trajectory.size()-1].translation().y();
                p.pose.position.z = trajectory[trajectory.size()-1].translation().z();

                Eigen::Quaterniond q(trajectory[trajectory.size()-1].rotation());
                p.pose.orientation.w = q.w();
                p.pose.orientation.x = q.x();
                p.pose.orientation.y = q.y();
                p.pose.orientation.z = q.z();

                pub_current_robot_pose.publish(path);
            }
		}

        ////////////////////////mission///////////////////////
        params.rot_speed = params.rot_speed_tmp;
        params.rot_speed_slow = params.rot_speed_slow_tmp;
        params.forward_speed_slow = params.forward_speed_slow_tmp;
        params.forward_speed = params.forward_speed_tmp;
        params.forward_speed_fast = params.forward_speed_fast_tmp;
        params.backward_speed_slow = params.backward_speed_slow_tmp;

        
        //std::cout << "all_points.size() " << all_points.size() << std::endl;
        
        int count_front = 0;
        int count_back = 0;

        for(int i = 0; i < all_points.size(); i+=2)
        {
            //const auto &point = all_points[i];

            if(all_points[i].point.z() > 0.2 && all_points[i].point.z() < 1.8){
                if(all_points[i].point.x() > 0.0 && all_points[i].point.x() < 0.4){
                    if(all_points[i].point.y() > -0.25 && all_points[i].point.y() < 0.25){
                        count_front++;
                    }
                }

                if(all_points[i].point.x() > -0.6 && all_points[i].point.x() < 0.0){
                    if(all_points[i].point.y() > -0.25 && all_points[i].point.y() < 0.25){
                        count_back++;
                    }
                }
            }
        }

        //std::cout << "count_front " << count_front << " count_back " << count_back << std::endl;
        if(count_front > 10 || count_back > 10){
            params.forward_speed_slow = 0;
            params.forward_speed = 0;
            params.forward_speed_fast = 0;
            params.backward_speed_slow = 0;
            params.rot_speed = 0;
            params.rot_speed_slow = 0;
        }      


        switch(mission_type){
            case MissionType::none:{
            break;
            }

            case MissionType::abort_mission:{
                geometry_msgs::Twist twist;
                twist.angular.z= 0;
                twist.linear.x = 0.0;
                pub_vel.publish(twist);
                mission_type = none;
            break;
            }
            
            case MissionType::single_goal_forward:{
                Eigen::Affine3d goal;
                Eigen::Affine3d current_pose;
                {
                    std::lock_guard<std::mutex> lck(mission_path_lock);
                    goal = mission_goal;
                    goal(2,3) = 0.0;
                }
                {
                    std::lock_guard<std::mutex> lck(buckets_lock);
                    current_pose = trajectory[trajectory.size()-1];
                    current_pose(2,3) = 0.0;
                }

                Eigen::Vector3d current_pose_goal = goal.translation() - current_pose.translation();
                double distance_to_goal = current_pose_goal.norm();
                current_pose_goal = current_pose_goal/current_pose_goal.norm();
                
                Eigen::Vector3d robot_heading(current_pose(0,0), current_pose(1,0), current_pose(2,0)); 

                double angle = acos(current_pose_goal.dot(robot_heading))/ M_PI * 180.0;
                double sign = (current_pose_goal.cross(robot_heading)).z();
                
                std_msgs::String d_message;
                std::string string_message = "angle between current_pose_goal and goal_heading " + std::to_string(angle) + 
                " [deg] sign " + std::to_string(sign) + " distance to goal " + std::to_string(distance_to_goal);
                
                if(distance_to_goal < 0.2){//manage headin and finish
                    if(angle > 5.0){
                        if(sign < 0){
                            geometry_msgs::Twist twist;
                            twist.angular.z= params.rot_speed;
                            twist.linear.x = 0.0;
                            pub_vel.publish(twist);  
                        }else{
                            geometry_msgs::Twist twist;
                            twist.angular.z= -params.rot_speed;
                            twist.linear.x = 0.0;
                            pub_vel.publish(twist);  
                        }
                    }else{
                        geometry_msgs::Twist twist;
                        twist.angular.z= 0;
                        twist.linear.x = 0.0;
                        pub_vel.publish(twist);

                        mission_type = MissionType::none;
                        std_msgs::String d_message;
                        std::string string_message = "single goal reached";
                        d_message.data = string_message;
                        pub_debug_message.publish(d_message);
                    }
                }else if(distance_to_goal < 1.0){//precize 
                    if(angle > 45){
                        if(sign < 0){
                            geometry_msgs::Twist twist;
                            twist.angular.z= params.rot_speed_slow;
                            twist.linear.x = 0;
                            pub_vel.publish(twist);  
                        }else{
                            geometry_msgs::Twist twist;
                            twist.angular.z= -params.rot_speed_slow;
                            twist.linear.x = 0;
                            pub_vel.publish(twist);  
                        }
                    }else{
                        if(sign < 0){
                            geometry_msgs::Twist twist;
                            twist.angular.z= params.rot_speed_slow;
                            twist.linear.x = params.forward_speed_slow;
                            pub_vel.publish(twist);  
                        }else{
                            geometry_msgs::Twist twist;
                            twist.angular.z= -params.rot_speed_slow;
                            twist.linear.x = params.forward_speed_slow;
                            pub_vel.publish(twist);  
                        }
                    }
                }else{
                    if(angle > 10){
                        if(sign < 0){
                            geometry_msgs::Twist twist;
                            twist.angular.z= params.rot_speed;
                            twist.linear.x = 0;
                            pub_vel.publish(twist);  
                        }else{
                            geometry_msgs::Twist twist;
                            twist.angular.z= -params.rot_speed;
                            twist.linear.x = 0;
                            pub_vel.publish(twist);  
                        }
                    }else{
                        if(angle > 5){
                            if(sign < 0){
                                geometry_msgs::Twist twist;
                                twist.angular.z= params.rot_speed;
                                twist.linear.x = params.forward_speed;
                                pub_vel.publish(twist);  
                            }else{
                                geometry_msgs::Twist twist;
                                twist.angular.z= -params.rot_speed;
                                twist.linear.x = params.forward_speed;
                                pub_vel.publish(twist);  
                            }
                        }else{
                            geometry_msgs::Twist twist;
                            twist.angular.z= 0;
                            if(distance_to_goal > 3){
                                twist.linear.x = params.forward_speed_fast * params.speed_boost;
                            }else{
                                twist.linear.x = params.forward_speed_fast;
                            }
                            
                            pub_vel.publish(twist);  
                        }
                    }
                }
                
                d_message.data = string_message;
                pub_debug_message.publish(d_message);

                //debug mission goal
                {
                    std::lock_guard<std::mutex> lck(mission_path_lock);
                    static int count = 0;
                    count++;
                    if(count % 10 == 0){ 
                        nav_msgs::Path path;
                        path.header.frame_id = "odom";
                        path.header.stamp = ros::Time::now();
                        path.poses.resize(1);
                        auto & p = path.poses[0];
                        p.pose.position.x = mission_goal.translation().x();
                        p.pose.position.y = mission_goal.translation().y();
                        p.pose.position.z = mission_goal.translation().z();

                        Eigen::Quaterniond q(mission_goal.rotation());
                        p.pose.orientation.w = q.w();
                        p.pose.orientation.x = q.x();
                        p.pose.orientation.y = q.y();
                        p.pose.orientation.z = q.z();

                        pub_current_mission_goal.publish(path);
                    }
                }
            break;
            }
            case mission_forward:{
                ///////////////////////////////////////
                //std::mutex mission_lock;
                //std::vector<Goal> mission;
                //int mission_next_goal_index = 0;
                int num_tmp = 0;
                Eigen::Affine3d goal;
                Eigen::Affine3d current_pose;
                {
                    std::lock_guard<std::mutex> lck(mission_lock);
                    goal = mission[mission_next_goal_index].pose;
                    goal(2,3) = 0.0;
                    num_tmp = mission.size();
                }
                {
                    std::lock_guard<std::mutex> lck(buckets_lock);
                    Eigen::Affine3d rp = Eigen::Affine3d::Identity();
                    rp(0,3) = params.robot_lidar_offset;
                    current_pose = trajectory[trajectory.size()-1] * rp;
                    current_pose(2,3) = 0.0;
                }

                Eigen::Vector3d current_pose_goal = goal.translation() - current_pose.translation();
                double distance_to_goal = current_pose_goal.norm();
                current_pose_goal = current_pose_goal/current_pose_goal.norm();
                
                Eigen::Vector3d robot_heading(current_pose(0,0), current_pose(1,0), current_pose(2,0)); 

                double angle = acos(current_pose_goal.dot(robot_heading))/ M_PI * 180.0;
                double sign = (current_pose_goal.cross(robot_heading)).z();
                
                std_msgs::String d_message;
                std::string string_message = "angle between current_pose_goal and goal_heading " + std::to_string(angle) + 
                " [deg] sign " + std::to_string(sign) + " distance to goal " + std::to_string(distance_to_goal) +
                 " number of goals " + std::to_string(num_tmp);
                //std_msgs::String d_message;
                d_message.data = string_message;
                pub_debug_message.publish(d_message);

                if(distance_to_goal < 0.2){//manage heading and finish>
                    if(angle > 5.0){
                        if(sign < 0){
                            geometry_msgs::Twist twist;
                            twist.angular.z= params.rot_speed_slow;
                            twist.linear.x = -params.backward_speed_slow;
                            pub_vel.publish(twist);  
                        }else{
                            geometry_msgs::Twist twist;
                            twist.angular.z= -params.rot_speed_slow;
                            twist.linear.x = -params.backward_speed_slow;
                            pub_vel.publish(twist);  
                        }
                    }else{
                        geometry_msgs::Twist twist;
                        twist.angular.z= 0;
                        twist.linear.x = 0.0;
                        pub_vel.publish(twist);

                        //mission_type = MissionType::none;
                        std_msgs::String d_message;
                        std::string string_message = "single goal reached";
                        d_message.data = string_message;
                        pub_debug_message.publish(d_message);

                        {
                            std::lock_guard<std::mutex> lck(mission_lock);
                            mission_next_goal_index++;

                            if(mission_next_goal_index == mission.size()){
                                mission_type = mission_backward;
                                mission_next_goal_index--;
                                mission_next_goal_index--;


////////////////////////////////////////////////////////////////////
                                save_laz_files();
///////////////////////////////////////////////////////////////////


                            }
                        }
                    }
                }else if(distance_to_goal < 1.0){//precize 
                    if(angle > 30){
                        if(sign < 0){
                            geometry_msgs::Twist twist;
                            twist.angular.z= params.rot_speed;
                            twist.linear.x = 0;//-params.backward_speed_slow;
                            pub_vel.publish(twist);  
                        }else{
                            geometry_msgs::Twist twist;
                            twist.angular.z= -params.rot_speed;
                            twist.linear.x = 0;//-params.backward_speed_slow;
                            pub_vel.publish(twist);  
                        }
                    }else{
                        if(sign < 0){
                            geometry_msgs::Twist twist;
                            twist.angular.z= params.rot_speed_slow;
                            twist.linear.x = params.forward_speed_slow;
                            pub_vel.publish(twist);  
                        }else{
                            geometry_msgs::Twist twist;
                            twist.angular.z= -params.rot_speed_slow;
                            twist.linear.x = params.forward_speed_slow;
                            pub_vel.publish(twist);  
                        }
                    }
                }else{
                    if(angle > 10){
                        if(sign < 0){
                            geometry_msgs::Twist twist;
                            twist.angular.z= params.rot_speed*2.0;
                            twist.linear.x = 0;//-0.05;
                            pub_vel.publish(twist);  
                        }else{
                            geometry_msgs::Twist twist;
                            twist.angular.z= -params.rot_speed*2.0;
                            twist.linear.x = 0;
                            pub_vel.publish(twist);  
                        }
                    }else{
                        if(angle > 5){
                            if(sign < 0){
                                geometry_msgs::Twist twist;
                                twist.angular.z= params.rot_speed;
                                twist.linear.x = params.forward_speed;
                                pub_vel.publish(twist);  
                            }else{
                                geometry_msgs::Twist twist;
                                twist.angular.z= -params.rot_speed;
                                twist.linear.x = params.forward_speed;
                                pub_vel.publish(twist);  
                            }
                        }else{
                            geometry_msgs::Twist twist;
                            twist.angular.z= 0;
                            if(distance_to_goal > 3.0){
                                twist.linear.x = params.forward_speed_fast * params.speed_boost;
                            }else{
                                twist.linear.x = params.forward_speed_fast;
                            }
                            
                            pub_vel.publish(twist);  
                        }
                    }
                }

                {
                    //std::lock_guard<std::mutex> lck(mission_path_lock);
                    static int count = 0;
                    count++;
                    if(count % 10 == 0){ 
                        nav_msgs::Path path;
                        path.header.frame_id = "odom";
                        path.header.stamp = ros::Time::now();
                        path.poses.resize(1);
                        auto & p = path.poses[0];
                        p.pose.position.x = goal.translation().x();
                        p.pose.position.y = goal.translation().y();
                        p.pose.position.z = goal.translation().z();

                        Eigen::Quaterniond q(goal.rotation());
                        p.pose.orientation.w = q.w();
                        p.pose.orientation.x = q.x();
                        p.pose.orientation.y = q.y();
                        p.pose.orientation.z = q.z();

                        pub_current_mission_goal.publish(path);
                    }
                }
                //////////////////////////////////////
                break;
            }
            case mission_backward:{
                ///////////////////////////////////////
                //std::mutex mission_lock;
                //std::vector<Goal> mission;
                //int mission_next_goal_index = 0;

                int num_tmp = 0;
                Eigen::Affine3d goal;
                Eigen::Affine3d current_pose;
                {
                    std::lock_guard<std::mutex> lck(mission_lock);
                    goal = mission[mission_next_goal_index].pose;
                    goal(2,3) = 0.0;

                    if(mission_next_goal_index == 0){
                        Eigen::Affine3d go = Eigen::Affine3d::Identity();
                        go(0,3) = -1.0;
                        goal = goal * go;
                    }

                    //rotate goal 180
                    //TaitBryanPose rot_tb;
                    //rot_tb.px = 0;
                    //rot_tb.py = 0;
                    //rot_tb.pz = 0;
                    //rot_tb.om = 0.0;
                    //rot_tb.fi = 0;
                    //rot_tb.ka = M_PI;
                    //goal = goal * affine_matrix_from_pose_tait_bryan(rot_tb);
                    num_tmp = mission.size();
                }
                {
                    std::lock_guard<std::mutex> lck(buckets_lock);
                    Eigen::Affine3d rp = Eigen::Affine3d::Identity();
                    rp(0,3) = params.robot_lidar_offset;
                    current_pose = trajectory[trajectory.size()-1] * rp;
                    current_pose(2,3) = 0.0;
                }

                //Eigen::Vector3d current_pose_goal = goal.translation() - current_pose.translation();
                Eigen::Vector3d current_pose_goal = current_pose.translation() - goal.translation();
                double distance_to_goal = current_pose_goal.norm();
                current_pose_goal = current_pose_goal/current_pose_goal.norm();
                
                Eigen::Vector3d robot_heading(current_pose(0,0), current_pose(1,0), current_pose(2,0)); 

                double angle = acos(current_pose_goal.dot(robot_heading))/ M_PI * 180.0;
                double sign = (current_pose_goal.cross(robot_heading)).z();
                
                std_msgs::String d_message;
                std::string string_message = "angle between current_pose_goal and goal_heading " + std::to_string(angle) + 
                " [deg] sign " + std::to_string(sign) + " distance to goal " + std::to_string(distance_to_goal) + std::to_string(distance_to_goal) 
                    + " number of goals " + std::to_string(num_tmp);
                d_message.data = string_message;
                pub_debug_message.publish(d_message);

                std::cout << "distance_to_goal" << distance_to_goal << std::endl;
                std::cout << "sign " << (int) sign << std::endl;
                std::cout << "angle " << angle << std::endl;


                if(distance_to_goal < 0.2){//manage heading and finish
                    if(angle > 5.0){
                        if(sign < 0){
                            geometry_msgs::Twist twist;
                            twist.angular.z= params.rot_speed;
                            twist.linear.x = -params.backward_speed_slow * -1;
                            pub_vel.publish(twist);  
                            std::cout << "1" << std::endl;
                        }else{
                            geometry_msgs::Twist twist;
                            twist.angular.z= -params.rot_speed;
                            twist.linear.x = -params.backward_speed_slow * -1;
                            pub_vel.publish(twist);  
                            std::cout << "2" << std::endl;
                        }
                    }else{
                        std::cout << "3" << std::endl;
                        geometry_msgs::Twist twist;
                        twist.angular.z= 0;
                        twist.linear.x = 0.0;
                        pub_vel.publish(twist);

                        //mission_type = MissionType::none;
                        std_msgs::String d_message;
                        std::string string_message = "single goal reached";
                        d_message.data = string_message;
                        pub_debug_message.publish(d_message);

                        {
                            std::lock_guard<std::mutex> lck(mission_lock);
                            mission_next_goal_index--;

                            if(mission_next_goal_index < 0){
                                geometry_msgs::Twist twist;
                                twist.angular.z= 0;
                                twist.linear.x = 0;
                                pub_vel.publish(twist);  
                                mission_type = none;
                                //exit(1);
                            }
                        }
                    }
                }else if(distance_to_goal < 1.0){//precize 
                //std::cout << "4" << std::endl;
                    if(angle > 30){
                        //std::cout << "5" << std::endl;
                        if(sign < 0){
                            geometry_msgs::Twist twist;
                            twist.angular.z= params.rot_speed ;
                            twist.linear.x = 0;//-params.backward_speed_slow * -1;
                            pub_vel.publish(twist);  
                            //std::cout << "6" << std::endl;
                        }else{
                            geometry_msgs::Twist twist;
                            twist.angular.z= -params.rot_speed ;
                            twist.linear.x = 0;//-params.backward_speed_slow * -1;
                            pub_vel.publish(twist);  
                            //std::cout << "7" << std::endl;
                        }
                    }else{
                        if(sign < 0){
                            //std::cout << "8" << std::endl;
                            geometry_msgs::Twist twist;
                            twist.angular.z= params.rot_speed_slow;
                            twist.linear.x = params.forward_speed_slow * -1;
                            pub_vel.publish(twist);  
                        }else{
                            //std::cout << "9" << std::endl;
                            geometry_msgs::Twist twist;
                            twist.angular.z= -params.rot_speed_slow;
                            twist.linear.x = params.forward_speed_slow * -1;
                            pub_vel.publish(twist);  
                        }
                    }
                }else{
                    //std::cout << "10" << std::endl;
                    if(angle > 10){
                        //std::cout << "11" << std::endl;
                        if(sign < 0){
                            //std::cout << "12" << std::endl;
                            geometry_msgs::Twist twist;
                            twist.angular.z= params.rot_speed*2.0;
                            twist.linear.x = -params.backward_speed_slow * -1;
                            pub_vel.publish(twist);  
                        }else{
                            //std::cout << "13" << std::endl;
                            geometry_msgs::Twist twist;
                            twist.angular.z= -params.rot_speed*2.0;
                            twist.linear.x = -params.backward_speed_slow * -1;
                            pub_vel.publish(twist);  
                        }
                    }else{
                        //std::cout << "14" << std::endl;
                        if(angle > 5){
                            //std::cout << "15" << std::endl;
                            if(sign < 0){
                                //std::cout << "16" << std::endl;
                                geometry_msgs::Twist twist;
                                twist.angular.z= params.rot_speed; 
                                twist.linear.x = params.forward_speed * -1;
                                pub_vel.publish(twist);  
                            }else{
                                //std::cout << "17" << std::endl;
                                geometry_msgs::Twist twist;
                                twist.angular.z= -params.rot_speed;
                                twist.linear.x = params.forward_speed * -1;
                                pub_vel.publish(twist);  
                            }
                        }else{
                            //std::cout << "18" << std::endl;
                            geometry_msgs::Twist twist;
                            twist.angular.z= 0;
                            if(distance_to_goal > 3.0){
                                twist.linear.x = params.forward_speed_fast * -1 * params.speed_boost;
                            }else{
                                twist.linear.x = params.forward_speed_fast * -1;
                            }
                            pub_vel.publish(twist);  
                        }
                    }
                }

                {
                    //std::lock_guard<std::mutex> lck(mission_path_lock);
                    static int count = 0;
                    count++;
                    if(count % 10 == 0){ 
                        nav_msgs::Path path;
                        path.header.frame_id = "odom";
                        path.header.stamp = ros::Time::now();
                        path.poses.resize(1);
                        auto & p = path.poses[0];
                        p.pose.position.x = goal.translation().x();
                        p.pose.position.y = goal.translation().y();
                        p.pose.position.z = goal.translation().z();

                        Eigen::Quaterniond q(goal.rotation());
                        p.pose.orientation.w = q.w();
                        p.pose.orientation.x = q.x();
                        p.pose.orientation.y = q.y();
                        p.pose.orientation.z = q.z();

                        pub_current_mission_goal.publish(path);
                    }
                }
                //////////////////////////////////////
                break;
            }
        }
	}
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-start;
	params.main_loop_time_execution = elapsed_seconds.count();
    std::cout << "timer main_loop: " << params.main_loop_time_execution << std::endl;
}

std::vector<Point3Di> decimate(const std::vector<Point3Di> &points, double bucket_x, double bucket_y, double bucket_z)
{

    Eigen::Vector3d b(bucket_x, bucket_y, bucket_z);
    std::vector<Point3Di> out;
   
    std::vector<PointBucketIndexPair> ip;
    ip.resize(points.size());
    out.reserve(points.size());

    for (int i = 0; i < points.size(); i++)
    {
        ip[i].index_of_point = i;
        ip[i].index_of_bucket = get_rgd_index(points[i].point, b);
    }

    std::sort(ip.begin(), ip.end(), [](const PointBucketIndexPair &a, const PointBucketIndexPair &b)
                { return a.index_of_bucket < b.index_of_bucket; });

    for (int i = 1; i < ip.size(); i++)
    {
        if (ip[i - 1].index_of_bucket != ip[i].index_of_bucket)
        {
            out.emplace_back(points[ip[i].index_of_point]);
        }
    }
    return out;
}

void optimize(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory, 
    std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
    Eigen::Vector3d b, std::map<unsigned long long int, Bucket> &buckets)
{
    std::vector<Eigen::Triplet<double>> tripletListA;
	std::vector<Eigen::Triplet<double>> tripletListP;
	std::vector<Eigen::Triplet<double>> tripletListB;

    Eigen::SparseMatrix<double> AtPAndt(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    Eigen::SparseMatrix<double> AtPBndt(intermediate_trajectory.size() * 6, 1);

    for(int i = 0; i < intermediate_points.size(); i += 1){
        if(intermediate_points[i].point.norm() < 1.0){
            continue;
        }

        Eigen::Vector3d point_global = intermediate_trajectory[intermediate_points[i].index_pose] * intermediate_points[i].point;
        auto index_of_bucket = get_rgd_index(point_global, b);

        //if(!buckets.contains(index_of_bucket)){
        if(!buckets.count(index_of_bucket)){
            continue;
        }

        if(buckets[index_of_bucket].number_of_points >= 5){
            Eigen::Matrix3d infm = buckets[index_of_bucket].cov.inverse();

            double threshold = 10000.0;

            if (infm(0, 0) > threshold)
                continue;
            if (infm(0, 1) > threshold)
                continue;
            if (infm(0, 2) > threshold)
                continue;
            if (infm(1, 0) > threshold)
                continue;
            if (infm(1, 1) > threshold)
                continue;
            if (infm(1, 2) > threshold)
                continue;
            if (infm(2, 0) > threshold)
                continue;
            if (infm(2, 1) > threshold)
                continue;
            if (infm(2, 2) > threshold)
                continue;

            if (infm(0, 0) < -threshold)
                continue;
            if (infm(0, 1) < -threshold)
                continue;
            if (infm(0, 2) < -threshold)
                continue;
            if (infm(1, 0) < -threshold)
                continue;
            if (infm(1, 1) < -threshold)
                continue;
            if (infm(1, 2) < -threshold)
                continue;
            if (infm(2, 0) < -threshold)
                continue;
            if (infm(2, 1) < -threshold)
                continue;
            if (infm(2, 2) < -threshold)
                continue;
           
            Eigen::Affine3d m_pose = intermediate_trajectory[intermediate_points[i].index_pose];
            Eigen::Vector3d &p_s = intermediate_points[i].point;
            TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
            //
            Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA;
            point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
                AtPA, 
                pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                p_s.x(), p_s.y(), p_s.z(),
                infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));

            Eigen::Matrix<double, 6, 1> AtPB;
            point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
                AtPB, 
                pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                p_s.x(), p_s.y(), p_s.z(),
                infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
                buckets[index_of_bucket].mean.x(), buckets[index_of_bucket].mean.y(), buckets[index_of_bucket].mean.z());
            
            int c = intermediate_points[i].index_pose * 6;

            for(int row = 0; row < 6; row++){
                for(int col = 0; col < 6; col++){
                    AtPAndt.coeffRef(c + row, c + col) += AtPA(row, col);
                 }
            }

            for(int row = 0; row < 6; row++){
                AtPBndt.coeffRef(c + row, 0) -= AtPB(row, 0);
            }
        }
    }

    std::vector<std::pair<int, int>> odo_edges;
    for(size_t i = 1; i < intermediate_trajectory.size(); i++){
		odo_edges.emplace_back(i-1,i);
	}

    std::vector<TaitBryanPose> poses;
    std::vector<TaitBryanPose> poses_desired;

    for(size_t i = 0 ; i < intermediate_trajectory.size(); i++){
        poses.push_back(pose_tait_bryan_from_affine_matrix(intermediate_trajectory[i]));
    }
    for(size_t i = 0 ; i < intermediate_trajectory_motion_model.size(); i++){
        poses_desired.push_back(pose_tait_bryan_from_affine_matrix(intermediate_trajectory_motion_model[i]));
    }

    for(size_t i = 0 ; i < odo_edges.size(); i++){
        Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
        relative_pose_tait_bryan_wc_case1(relative_pose_measurement_odo,
                poses_desired[odo_edges[i].first].px,
                poses_desired[odo_edges[i].first].py,
                poses_desired[odo_edges[i].first].pz,
                poses_desired[odo_edges[i].first].om,
                poses_desired[odo_edges[i].first].fi,
                poses_desired[odo_edges[i].first].ka,
                poses_desired[odo_edges[i].second].px,
                poses_desired[odo_edges[i].second].py,
                poses_desired[odo_edges[i].second].pz,
                poses_desired[odo_edges[i].second].om,
                poses_desired[odo_edges[i].second].fi,
                poses_desired[odo_edges[i].second].ka);

        Eigen::Matrix<double, 6, 1> delta;
        relative_pose_obs_eq_tait_bryan_wc_case1(
                delta,
                poses[odo_edges[i].first].px,
                poses[odo_edges[i].first].py,
                poses[odo_edges[i].first].pz,
                poses[odo_edges[i].first].om,
                poses[odo_edges[i].first].fi,
                poses[odo_edges[i].first].ka,
                poses[odo_edges[i].second].px,
                poses[odo_edges[i].second].py,
                poses[odo_edges[i].second].pz,
                poses[odo_edges[i].second].om,
                poses[odo_edges[i].second].fi,
                poses[odo_edges[i].second].ka,
                relative_pose_measurement_odo(0,0),
                relative_pose_measurement_odo(1,0),
                relative_pose_measurement_odo(2,0),
                relative_pose_measurement_odo(3,0),
                relative_pose_measurement_odo(4,0),
                relative_pose_measurement_odo(5,0));

        Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
        relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
                poses[odo_edges[i].first].px,
                poses[odo_edges[i].first].py,
                poses[odo_edges[i].first].pz,
                poses[odo_edges[i].first].om,
                poses[odo_edges[i].first].fi,
                poses[odo_edges[i].first].ka,
                poses[odo_edges[i].second].px,
                poses[odo_edges[i].second].py,
                poses[odo_edges[i].second].pz,
                poses[odo_edges[i].second].om,
                poses[odo_edges[i].second].fi,
                poses[odo_edges[i].second].ka);

        int ir = tripletListB.size();

        int ic_1 = odo_edges[i].first * 6;
        int ic_2 = odo_edges[i].second * 6;

        for(size_t row = 0 ; row < 6; row ++){
            tripletListA.emplace_back(ir + row, ic_1    , -jacobian(row,0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row,1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row,2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row,3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row,4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row,5));

            tripletListA.emplace_back(ir + row, ic_2    , -jacobian(row,6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row,7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row,8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row,9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row,10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row,11));
        }

        tripletListB.emplace_back(ir,     0, delta(0,0));
        tripletListB.emplace_back(ir + 1, 0, delta(1,0));
        tripletListB.emplace_back(ir + 2, 0, delta(2,0));
        tripletListB.emplace_back(ir + 3, 0, delta(3,0));
        tripletListB.emplace_back(ir + 4, 0, delta(4,0));
        tripletListB.emplace_back(ir + 5, 0, delta(5,0));

		double wa = 1.0 / (1.0 / 180.0 * M_PI );
		wa *= wa;

        tripletListP.emplace_back(ir ,    ir,     100);
        tripletListP.emplace_back(ir + 1, ir + 1, 100);
        tripletListP.emplace_back(ir + 2, ir + 2, 100);
        tripletListP.emplace_back(ir + 3, ir + 3, wa);
        tripletListP.emplace_back(ir + 4, ir + 4, wa);
        tripletListP.emplace_back(ir + 5, ir + 5, wa);
    }

	int ir = tripletListB.size();
	tripletListA.emplace_back(ir     , 0, 1);
	tripletListA.emplace_back(ir + 1 , 1, 1);
	tripletListA.emplace_back(ir + 2 , 2, 1);
	tripletListA.emplace_back(ir + 3 , 3, 1);
	tripletListA.emplace_back(ir + 4 , 4, 1);
	tripletListA.emplace_back(ir + 5 , 5, 1);

	tripletListP.emplace_back(ir     , ir,     1000000);
	tripletListP.emplace_back(ir + 1 , ir + 1, 1000000);
	tripletListP.emplace_back(ir + 2 , ir + 2, 1000000);
	tripletListP.emplace_back(ir + 3 , ir + 3, 1000000);
	tripletListP.emplace_back(ir + 4 , ir + 4, 1000000);
	tripletListP.emplace_back(ir + 5 , ir + 5, 1000000);

	tripletListB.emplace_back(ir     , 0, 0);
	tripletListB.emplace_back(ir + 1 , 0, 0);
	tripletListB.emplace_back(ir + 2 , 0, 0);
	tripletListB.emplace_back(ir + 3 , 0, 0);
	tripletListB.emplace_back(ir + 4 , 0, 0);
	tripletListB.emplace_back(ir + 5 , 0, 0);

    Eigen::SparseMatrix<double> matA(tripletListB.size(), intermediate_trajectory.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    Eigen::SparseMatrix<double> AtPB(intermediate_trajectory.size() * 6, 1);

    {
    Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
    AtPA = (AtP) * matA;
    AtPB = (AtP) * matB;
    }

    tripletListA.clear();
    tripletListP.clear();
    tripletListB.clear();

    AtPA += AtPAndt; 
    AtPB += AtPBndt; 
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
    Eigen::SparseMatrix<double> x = solver.solve(AtPB);
    std::vector<double> h_x;
    for (int k=0; k<x.outerSize(); ++k){
        for (Eigen::SparseMatrix<double>::InnerIterator it(x,k); it; ++it){
            h_x.push_back(it.value());
        }
    }
    
    if(h_x.size() == 6 * intermediate_trajectory.size()){
        int counter = 0;

        for(size_t i = 0; i < intermediate_trajectory.size(); i++){
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(intermediate_trajectory[i]);
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];
			if(i > 0){
            	intermediate_trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
			}
        }
    }
return;
}

void update_rgd(Eigen::Vector3d b, std::map<unsigned long long int, Bucket> &buckets,
                std::vector<Point3Di>& points_global)
{
    for (int i = 0; i < points_global.size(); i++)
    {
        auto index_of_bucket = get_rgd_index(points_global[i].point, b);

        //if (buckets.contains(index_of_bucket))
        if (buckets.count(index_of_bucket))
        {
        	buckets[index_of_bucket].number_of_points ++;
            auto curr_mean = points_global[i].point;
            auto mean = buckets[index_of_bucket].mean;
			int number_of_points = buckets[index_of_bucket].number_of_points;
            //buckets[index_of_bucket].mean += (mean - curr_mean) / buckets[index_of_bucket].number_of_points;
			//buckets[index_of_bucket].mean = (curr_mean * number_of_points + mean)/(number_of_points + 1);
            
            Eigen::Matrix3d cov_update;
            cov_update(0, 0) = (mean.x() - curr_mean.x()) * (mean.x() - curr_mean.x());
            cov_update(0, 1) = (mean.x() - curr_mean.x()) * (mean.y() - curr_mean.y());
            cov_update(0, 2) = (mean.x() - curr_mean.x()) * (mean.z() - curr_mean.z());
            cov_update(1, 0) = (mean.y() - curr_mean.y()) * (mean.x() - curr_mean.x());
            cov_update(1, 1) = (mean.y() - curr_mean.y()) * (mean.y() - curr_mean.y());
            cov_update(1, 2) = (mean.y() - curr_mean.y()) * (mean.z() - curr_mean.z());
            cov_update(2, 0) = (mean.z() - curr_mean.z()) * (mean.x() - curr_mean.x());
            cov_update(2, 1) = (mean.z() - curr_mean.z()) * (mean.y() - curr_mean.y());
            cov_update(2, 2) = (mean.z() - curr_mean.z()) * (mean.z() - curr_mean.z());

            buckets[index_of_bucket].cov = buckets[index_of_bucket].cov * (buckets[index_of_bucket].number_of_points - 1) / buckets[index_of_bucket].number_of_points +
                cov_update * (buckets[index_of_bucket].number_of_points - 1)/ (buckets[index_of_bucket].number_of_points * buckets[index_of_bucket].number_of_points);
		}
        else
        {
            buckets[index_of_bucket].mean = points_global[i].point;
            buckets[index_of_bucket].cov = Eigen::Matrix3d::Zero();
            buckets[index_of_bucket].cov(0,0) = 0.03 * 0.03;
            buckets[index_of_bucket].cov(1,1) = 0.03 * 0.03;
            buckets[index_of_bucket].cov(2,2) = 0.03 * 0.03;
            buckets[index_of_bucket].number_of_points = 1;
        }
    }
}

bool save_buckets(BucketMap &buckets, const std::string &path)
{
    nlohmann::json jj;
    nlohmann::json jbuckets;
    for(std::map<unsigned long long int, Bucket>::iterator it = buckets.begin(); it != buckets.end(); ++it) {
        //const auto key_str = std::to_string(it->first);
        nlohmann::json jbucket{
            {"key", it->first},
            {"index_begin", it->second.index_begin},
            {"index_end", it->second.index_end},
            {"number_of_points", it->second.number_of_points},
            {"mean_x", it->second.mean.x()},
            {"mean_y", it->second.mean.y()},
            {"mean_z", it->second.mean.z()},
            {"cov_00", it->second.cov(0, 0)},
            {"cov_01", it->second.cov(0, 1)},
            {"cov_02", it->second.cov(0, 2)},
            {"cov_10", it->second.cov(1, 0)},
            {"cov_11", it->second.cov(1, 1)},
            {"cov_12", it->second.cov(1, 2)},
            {"cov_20", it->second.cov(2, 0)},
            {"cov_21", it->second.cov(2, 1)},
            {"cov_22", it->second.cov(2, 2)}
        };

        jbuckets.push_back(jbucket);
    }
    jj["buckets"] = jbuckets;

    std::ofstream fs(path);
    if (!fs.good()){
        return false;
    }
    fs << jj.dump(2);
    fs.close();

    return true;
}

bool load_buckets(BucketMap &buckets, const std::string &path)
{
    buckets.clear();
    try
    {
        std::ifstream fs(path);
        if (!fs.good()){
            return false;
        }
        nlohmann::json data = nlohmann::json::parse(fs);
        fs.close();

        for (const auto &jbucket : data["buckets"])
        {
            //struct Bucket{
            //    long long unsigned int index_begin;
            //    long long unsigned int index_end;
            //    long long unsigned int number_of_points;
            //    Eigen::Vector3d mean;
            //    Eigen::Matrix3d cov;
            //};

            Bucket bucket;
            bucket.index_begin = jbucket["index_begin"];
            bucket.index_end = jbucket["index_end"];
            bucket.number_of_points = jbucket["number_of_points"];
            bucket.mean.x() = jbucket["mean_x"];
            bucket.mean.y() = jbucket["mean_y"];
            bucket.mean.z() = jbucket["mean_z"];
            bucket.cov(0, 0) = jbucket["cov_00"];
            bucket.cov(0, 1) = jbucket["cov_01"];
            bucket.cov(0, 2) = jbucket["cov_02"];
            bucket.cov(1, 0) = jbucket["cov_10"];
            bucket.cov(1, 1) = jbucket["cov_11"];
            bucket.cov(1, 2) = jbucket["cov_12"];
            bucket.cov(2, 0) = jbucket["cov_20"];
            bucket.cov(2, 1) = jbucket["cov_21"];
            bucket.cov(2, 2) = jbucket["cov_22"];

            //std::map<unsigned long long int, Bucket> buckets;
            unsigned long long int index_of_bucket = jbucket["key"];
            buckets[index_of_bucket] = bucket;
        }
        return true;
    }
    catch (std::exception &e)
    {
        std::cout << "cant load data: " << e.what() << std::endl;
        return false;
    }
}