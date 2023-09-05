#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Path.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <GL/freeglut.h>
#include "imgui.h"
#include "imgui_impl_glut.h"
#include "imgui_impl_opengl2.h"
#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>
#include <ImGuizmo.h>
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
#include <Eigen/Eigen>
#include "structures.h"
#include "transformations.h"
#include <laszip/laszip_api.h>
#include <portable-file-dialogs.h>

struct LaserBeam {
	Eigen::Vector3d position;
	Eigen::Vector3d direction;
	float distance;
	float range;
};

struct Point3Di
{
    Eigen::Vector3d point;
	double timestamp;
    float intensity;
    int index_pose;
};

struct PointCloud{
    std::vector<Point3Di> points;
    int decimation;
    bool show = true;
};

struct BaseStationParameters{
    int major = 0;
    int minor = 0;
    double main_loop_time_execution = 0.0;
    int get_current_point_cloud_nr_points = 100000;

    bool show_buckets = true;
    float intensity_offset = 0.5;

    int pcs_point_size = 1;

    std::mutex current_robot_pose_lock;
    Eigen::Affine3d current_robot_pose = Eigen::Affine3d::Identity();

    std::mutex current_mission_goal_lock;
    Eigen::Affine3d current_mission_goal = Eigen::Affine3d::Identity();

    float robot_w = 0.43;
    float robot_l = 0.508;
    float robot_h = 0.4;

    float calib_height_above_ground = 0.2;

    float visualization_height_threshold = 40;

    bool planner_mode = false;
    bool single_goal_forward = false;
    bool multiple_goals = false;

    Eigen::Vector3d goal_arrow_end = {0,0,0};
    Eigen::Vector3d goal_arrow_begin = {0,0,0};

    double calib_x_offset = -0.2;

    int num_scans_from_end = 0;
};

float m_gizmo[] = {1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1};


const unsigned int window_width = 640;
const unsigned int window_height = 480;
int mouse_buttons = 0;
int mouse_old_x, mouse_old_y;
bool gui_mouse_down{false};
float rotate_x = 0.0, rotate_y = 0.0;
float translate_z = -30.0;
float translate_x, translate_y = 0.0;

//ros::NodeHandle *nh;
//ros::Publisher pub_path;
//ros::Publisher pub_vel;

ros::Publisher pub_get_current_pc;
ros::Publisher pub_get_current_map;
ros::Publisher pub_reset_jackal;
ros::Publisher pub_calib_height_above_ground;
ros::Publisher pub_single_goal_forward;
ros::Publisher pub_abort;
ros::Publisher pub_multiple_goals_to_robot;
ros::Publisher pub_multiple_goals_to_robot_execute;
ros::Publisher pub_get_last_goal_pc;
ros::Publisher pub_save_buckets;
ros::Publisher pub_load_buckets;

ros::Subscriber sub_get_current_pc;
ros::Subscriber sub_get_current_map;
ros::Subscriber sub_current_robot_pose;
ros::Subscriber sub_current_mission_goal;

//pub_current_robot_pose = nh.advertise<nav_msgs::Path>  ("current_robot_pose", 1);

BaseStationParameters params;

std::mutex current_map_lock;
std::vector<Eigen::Vector3d> current_map;
std::mutex point_clouds_lock;
std::vector<PointCloud> point_clouds;

bool load_pc_laz(std::string input_file_name, PointCloud &pc){
    pc.decimation = 10;
    pc.show = true;
    
    laszip_POINTER laszip_reader;
	if (laszip_create(&laszip_reader))
	{
		fprintf(stderr, ":DLL ERROR: creating laszip reader\n");
		/*PointCloud pc;
		pc.m_pose = Eigen::Affine3d::Identity();
		pc.m_initial_pose = pc.m_pose;
		pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
		pc.gui_translation[0] = pc.pose.px;
		pc.gui_translation[1] = pc.pose.py;
		pc.gui_translation[2] = pc.pose.pz;
		pc.gui_rotation[0] = rad2deg(pc.pose.om);
		pc.gui_rotation[1] = rad2deg(pc.pose.fi);
		pc.gui_rotation[2] = rad2deg(pc.pose.ka);
		pc.file_name = input_file_names[i];
		point_clouds.push_back(pc);*/
		return false;
	}

	laszip_BOOL is_compressed = 0;
	if (laszip_open_reader(laszip_reader, input_file_name.c_str(), &is_compressed))
	{
		fprintf(stderr, ":DLL ERROR: opening laszip reader for '%s'\n", input_file_name.c_str());
		/*PointCloud pc;
		pc.m_pose = Eigen::Affine3d::Identity();
		pc.m_initial_pose = pc.m_pose;
		pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
		pc.gui_translation[0] = pc.pose.px;
		pc.gui_translation[1] = pc.pose.py;
		pc.gui_translation[2] = pc.pose.pz;
		pc.gui_rotation[0] = rad2deg(pc.pose.om);
		pc.gui_rotation[1] = rad2deg(pc.pose.fi);
		pc.gui_rotation[2] = rad2deg(pc.pose.ka);
		pc.file_name = input_file_names[i];
		point_clouds.push_back(pc);*/
		return false;
	}
	std::cout << "compressed : " << is_compressed << std::endl;
	laszip_header *header;

	
	if (laszip_get_header_pointer(laszip_reader, &header))
	{
		fprintf(stderr, ":DLL ERROR: getting header pointer from laszip reader\n");
		/*PointCloud pc;
		pc.m_pose = Eigen::Affine3d::Identity();
		pc.m_initial_pose = pc.m_pose;
		pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
		pc.gui_translation[0] = pc.pose.px;
		pc.gui_translation[1] = pc.pose.py;
		pc.gui_translation[2] = pc.pose.pz;
		pc.gui_rotation[0] = rad2deg(pc.pose.om);
		pc.gui_rotation[1] = rad2deg(pc.pose.fi);
		pc.gui_rotation[2] = rad2deg(pc.pose.ka);
		pc.file_name = input_file_names[i];
		point_clouds.push_back(pc);*/
		return false;
	}
	fprintf(stderr, "file '%s' contains %u points\n", input_file_name.c_str(), header->number_of_point_records);
	laszip_point *point;
	if (laszip_get_point_pointer(laszip_reader, &point))
	{
		fprintf(stderr, ":DLL ERROR: getting point pointer from laszip reader\n");
		return false;
	}

	//pc.m_pose = Eigen::Affine3d::Identity();
	//pc.m_initial_pose = pc.m_pose;
	//pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
	//pc.gui_translation[0] = pc.pose.px;
	//pc.gui_translation[1] = pc.pose.py;
	//pc.gui_translation[2] = pc.pose.pz;
	//pc.gui_rotation[0] = rad2deg(pc.pose.om);
	//pc.gui_rotation[1] = rad2deg(pc.pose.fi);
	//pc.gui_rotation[2] = rad2deg(pc.pose.ka);

	for (int j = 0; j < header->number_of_point_records; j++)
	{
		if (laszip_read_point(laszip_reader))
		{
			fprintf(stderr, ":DLL ERROR: reading point %u\n", j);
			continue;
		}

		//LAZPoint p;
        Eigen::Vector3d p;
		p.x() = header->x_offset + header->x_scale_factor * static_cast<double>(point->X);
		p.y() = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y);
		p.z() = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);

		//Eigen::Vector3d pp(p.x, p.y, p.z);
        Point3Di ppp;
        ppp.index_pose = 0;
        ppp.intensity = point->intensity;
        ppp.timestamp = 0;
        ppp.point = p;
        pc.points.push_back(ppp);
        /*struct Point3Di
        {
            Eigen::Vector3d point;
            double timestamp;
            float intensity;
            int index_pose;
        };

        struct PointCloud{
            std::vector<Point3Di> points;
            int decimation;
            bool show = true;
        };*/


        
        //pc.points_local.push_back(pp);
		//pc.intensities.push_back(point->intensity);
	}
	laszip_close_reader(laszip_reader);
}


struct MuLtipleGoal{
    Eigen::Affine3d goal;
    bool gizmo = false;
};

std::mutex multiple_goals_lock;
std::vector<MuLtipleGoal> multiple_goals;

bool initGL(int *argc, char **argv);
void display();
void reshape(int w, int h);
void mouse(int glut_button, int state, int x, int y);
void motion(int x, int y);
void idle(void);
void main_loop(bool render);
Eigen::Vector3d GLWidgetGetOGLPos(int x, int y);

void currentPointCloudCallback(const pcl::PointCloud<pcl::PointXYZI>::Ptr& msg)
{
    std::lock_guard<std::mutex> lck(point_clouds_lock);
    //current_points.clear();
    std::vector<Point3Di> current_points;

    int pc_count = 0;
    for (pcl::PointCloud<pcl::PointXYZI>::iterator it = msg->begin(); it != msg->end(); ++it)
	{
        Point3Di p;
        p.index_pose = 0;
        p.intensity = it->intensity;
        p.timestamp = 0;
        p.point = Eigen::Vector3d(it->x, it->y, it->z);
        current_points.push_back(p);
        pc_count ++;
	}

    PointCloud pc;
    pc.decimation = 1;
    pc.show = true;
    pc.points = current_points;
    point_clouds.push_back(pc);
    std::cout << "currentPointCloudCallback: [" << pc_count << " points]" << std::endl;
}

void currentMapCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg)
{
    std::lock_guard<std::mutex> lck(current_map_lock);
    current_map.clear();
    int pc_count = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = msg->begin(); it != msg->end(); ++it)
	{
        current_map.emplace_back(it->x, it->y, it->z);
        pc_count ++;
	}
    std::cout << "currentMapCallback: [" << pc_count << " active buckets]" << std::endl;
}

void currentRobotPoseCallback(const nav_msgs::Path::Ptr &msg){
    std::lock_guard<std::mutex> lck(params.current_robot_pose_lock);

    params.current_robot_pose.translation().x() = msg->poses[0].pose.position.x;
    params.current_robot_pose.translation().y() = msg->poses[0].pose.position.y;
    params.current_robot_pose.translation().z() = msg->poses[0].pose.position.z;

    Eigen::Quaterniond q;
    q.w() = msg->poses[0].pose.orientation.w;
    q.x() = msg->poses[0].pose.orientation.x;
    q.y() = msg->poses[0].pose.orientation.y;
    q.z() = msg->poses[0].pose.orientation.z;

    params.current_robot_pose.linear() = q.toRotationMatrix();
}

void currentMissionGoalCallback(const nav_msgs::Path::Ptr &msg){

    //std::mutex current_mission_goal_lock;
    //E//igen::Affine3d current_mission_goal = Eigen::Affine3d::Identity();

    std::lock_guard<std::mutex> lck(params.current_mission_goal_lock);

    params.current_mission_goal.translation().x() = msg->poses[0].pose.position.x;
    params.current_mission_goal.translation().y() = msg->poses[0].pose.position.y;
    params.current_mission_goal.translation().z() = msg->poses[0].pose.position.z;

    Eigen::Quaterniond q;
    q.w() = msg->poses[0].pose.orientation.w;
    q.x() = msg->poses[0].pose.orientation.x;
    q.y() = msg->poses[0].pose.orientation.y;
    q.z() = msg->poses[0].pose.orientation.z;

    params.current_mission_goal.linear() = q.toRotationMatrix();
}

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



void project_gui(BaseStationParameters &paramters)
{
	if(ImGui::Begin(std::string("Project demo odometry v" + std::to_string(paramters.major) + "." + std::to_string(paramters.minor)).c_str())){
	    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::Text("main_loop_time_execution %.3f ms", params.main_loop_time_execution);
        ImGui::Text("----Point Clouds visualisation begin----"); 
        ImGui::InputFloat("intensity_offset", &params.intensity_offset);
        ImGui::InputInt("point_size", &params.pcs_point_size);
        if(params.pcs_point_size < 1){
            params.pcs_point_size = 1;
        }
        ImGui::Text("----Point Clouds visualisation end----"); 

        ImGui::SliderFloat("visualization_height_threshold", &params.visualization_height_threshold, 0, 40);
                

        //pub_calib_height_above_ground
        float calib_height_above_ground_prev = params.calib_height_above_ground;
        ImGui::InputFloat("calib_height_above_ground", &params.calib_height_above_ground);
        if(calib_height_above_ground_prev != params.calib_height_above_ground){
            ros::NodeHandle nh;
            std_msgs::Float32 h;
            h.data = params.calib_height_above_ground;

            std::cout << "params.calib_height_above_ground: " << params.calib_height_above_ground << std::endl;
            pub_calib_height_above_ground.publish(h);
        }

        if(ImGui::Button("reset jackal (clear map)")){
            ros::NodeHandle nh;
            std_msgs::Bool reset;
            reset.data = true;
            pub_reset_jackal.publish(reset);
            std::cout << "reset jackal (clear map)" << std::endl;
        }

        if(ImGui::Button("get current point cloud")){
            ros::NodeHandle nh;
            std_msgs::Int32 get_current_pc;
            get_current_pc.data = params.get_current_point_cloud_nr_points;
            pub_get_current_pc.publish(get_current_pc);
            std::cout << "send get_current_pc" << std::endl;
        }
        ImGui::SameLine();
        ImGui::InputInt("nr_points", &params.get_current_point_cloud_nr_points);

        if(ImGui::Button("get current map (means in buckets)")){
            ros::NodeHandle nh;
            std_msgs::Bool get;
            get.data = true;
            pub_get_current_map.publish(get);
            std::cout << "send get_current_map" << std::endl;
            params.show_buckets = true;
        }
        ImGui::SameLine();
        ImGui::Checkbox("show", &params.show_buckets);

        ImGui::Text("-------------------------Planner--------------------------------");
        if(ImGui::Button("ABORT MISSION")){
            std_msgs::Bool abort;
            abort.data = true;
            pub_abort.publish(abort);
            static int count = 0;
            std::cout << "[" << count++ <<"] ABORT" << std::endl;

            {
                std::lock_guard<std::mutex> lckg(params.current_mission_goal_lock);
                params.current_mission_goal = Eigen::Affine3d::Identity();
            }
        }

        ImGui::Checkbox("planner_mode", &params.planner_mode);
        
        if(params.planner_mode){
            ImGui::Checkbox("single_goal_forward", &params.single_goal_forward);
            if(params.single_goal_forward){
                params.multiple_goals = false;
            }
            ImGui::Checkbox("multiple_goals", &params.multiple_goals);
            if(params.multiple_goals){
                params.single_goal_forward = false;
            }
        }else{
            params.single_goal_forward = false;
            params.multiple_goals = false;
        }

        
 

        //bool planner_mode = false;
        //bool single_goal_forward = fasle;
        ImGui::Text("-------------------------Multiple Goals------------------------------");
        {
            std::lock_guard<std::mutex> lck(multiple_goals_lock);
            //for(auto &goal:multiple_goals){

            //}
            int index_remove = -1;
            for(int i = 0; i < multiple_goals.size(); i++){
                std::string text = "gizmo [" + std::to_string(i) + "]";
                ImGui::Checkbox(text.c_str(), &multiple_goals[i].gizmo);

                if(multiple_goals[i].gizmo){
                    params.single_goal_forward = false;
                    params.multiple_goals = false;
                    params.planner_mode = false;
                    for (size_t j = 0; j < multiple_goals.size(); j++)
                    {
                        if (i != j)
                        {
                            multiple_goals[j].gizmo = false;
                        }
                    }
                    m_gizmo[0] = (float)multiple_goals[i].goal(0, 0);
                    m_gizmo[1] = (float)multiple_goals[i].goal(1, 0);
                    m_gizmo[2] = (float)multiple_goals[i].goal(2, 0);
                    m_gizmo[3] = (float)multiple_goals[i].goal(3, 0);
                    m_gizmo[4] = (float)multiple_goals[i].goal(0, 1);
                    m_gizmo[5] = (float)multiple_goals[i].goal(1, 1);
                    m_gizmo[6] = (float)multiple_goals[i].goal(2, 1);
                    m_gizmo[7] = (float)multiple_goals[i].goal(3, 1);
                    m_gizmo[8] = (float)multiple_goals[i].goal(0, 2);
                    m_gizmo[9] = (float)multiple_goals[i].goal(1, 2);
                    m_gizmo[10] = (float)multiple_goals[i].goal(2, 2);
                    m_gizmo[11] = (float)multiple_goals[i].goal(3, 2);
                    m_gizmo[12] = (float)multiple_goals[i].goal(0, 3);
                    m_gizmo[13] = (float)multiple_goals[i].goal(1, 3);
                    m_gizmo[14] = (float)multiple_goals[i].goal(2, 3);
                    m_gizmo[15] = (float)multiple_goals[i].goal(3, 3);
                }

                ImGui::SameLine();
                text = "remove goal[" + std::to_string(i) + "]";
                if(ImGui::Button(text.c_str())){
                    index_remove = i;
                }
            }

            if(index_remove != -1){
                std::vector<MuLtipleGoal> new_multiple_goals;
                for(int i = 0; i < multiple_goals.size(); i++){
                    if(i != index_remove){
                        new_multiple_goals.push_back(multiple_goals[i]);
                    }
                }
                multiple_goals = new_multiple_goals;
            }

            if(multiple_goals.size() >= 2){
                //static int num = 0;
                ImGui::InputInt("num_scans_from_end", &params.num_scans_from_end);
                if(params.num_scans_from_end < 0){
                    params.num_scans_from_end = 0;
                }
                if(params.num_scans_from_end > multiple_goals.size()){
                    params.num_scans_from_end = multiple_goals.size();
                }

                //ImGui::SameLine();
                if(ImGui::Button("set on robot")){
                    nav_msgs::Path destination;
                    destination.header.frame_id = "odom";
                    destination.header.stamp = ros::Time::now();
                    destination.poses.resize(multiple_goals.size());
                    std::cout << "Sending " << multiple_goals.size() << " goals" << std::endl;
                    for(int i = 0; i < multiple_goals.size(); i++){
                        Eigen::Affine3d goal = multiple_goals[i].goal;
                        auto & p = destination.poses[i];
                        p.pose.position.x = goal.translation().x();
                        p.pose.position.y = goal.translation().y();
                        p.pose.position.z = goal.translation().z();
                        Eigen::Quaterniond q(goal.rotation());
                        p.pose.orientation.w = q.w();
                        p.pose.orientation.x = q.x();
                        p.pose.orientation.y = q.y();
                        p.pose.orientation.z = q.z();
                        pub_multiple_goals_to_robot.publish(destination);
                    }
                }
                ImGui::SameLine();
                if(ImGui::Button("execute")){
                    //pub_multiple_goals_to_robot_execute = nh.advertise< std_msgs::Int32 > ("multiple_goals_to_robot_execute", 1);
                    ros::NodeHandle nh;
                    std_msgs::Int32 num_scans_from_end;
                    num_scans_from_end.data = params.num_scans_from_end;

                    std::cout << "send params.num_scans_from_end: " << params.num_scans_from_end << std::endl;
                    pub_multiple_goals_to_robot_execute.publish(num_scans_from_end);

                    params.planner_mode = false;
                    params.single_goal_forward = false;
                    params.multiple_goals = false;
                }
            }
        }

        ImGui::Text("-------------------------Point Clouds--------------------------------");
        if(ImGui::Button("get point cloud from last goal")){
            ros::NodeHandle nh;
            std_msgs::Int32 get_current_pc;
            get_current_pc.data = params.get_current_point_cloud_nr_points;
            pub_get_last_goal_pc.publish(get_current_pc);
            std::cout << "send pub_get_last_goal_pc" << std::endl;
        }
        {
            std::lock_guard<std::mutex> lck(point_clouds_lock);

            if(ImGui::Button("Load Point Cloud (laz)")){
                static std::shared_ptr<pfd::open_file> open_file;
                std::vector<std::string> input_file_names;
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
                const auto t = [&]()
                {
                    std::vector<std::string> filters;
                    auto sel = pfd::open_file("Load las files", "C:\\", filters, true).result();
                    for (int i = 0; i < sel.size(); i++)
                    {
                        input_file_names.push_back(sel[i]);
                        // std::cout << "las file: '" << input_file_name << "'" << std::endl;
                    }
                };
                std::thread t1(t);
                t1.join();

                if (input_file_names.size() > 0)
                {
                    for (size_t i = 0; i < input_file_names.size(); i++)
                    {
                        PointCloud pc;
                        load_pc_laz(input_file_names[i], pc);
                        point_clouds.push_back(pc);

                        //std::cout << input_file_names[i] << std::endl;
                    }
                }
                //ToDo
                //struct PointCloud{
                //    std::vector<Point3Di> points;
                //    int decimation;
                //    bool show = true;
                //};

            }

            //ImGui::SameLine();

            if(point_clouds.size() > 0){
                if(ImGui::Button("Export ALL as single Point Cloud (laz)")){
                    std::vector<Point3Di> points;
                    for (int i = 0; i < point_clouds.size(); i++){
                        for(int p = 0; p < point_clouds[i].points.size(); p++){
                            points.push_back(point_clouds[i].points[p]);
                        }
                    }

                    {
                        std::lock_guard<std::mutex> lck(current_map_lock);
                        for(const auto &p:current_map){
                            Point3Di pp;
                            pp.index_pose = 0;
                            pp.intensity = 100;
                            pp.timestamp = 0;
                            pp.point = p;
                            points.push_back(pp);
                         }
                    }

                    std::shared_ptr<pfd::save_file> save_file;
                    std::string output_file_name = "";
                    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                    const auto t = [&]()
                    {
                        auto sel = pfd::save_file("Save file", "C:\\").result();
                        output_file_name = sel;
                        std::cout << "file to save: '" << output_file_name << "'" << std::endl;
                    };
                    std::thread t1(t);
                    t1.join();


                    if (output_file_name.size() > 0)
                    {
                        exportLaz(output_file_name, points);
                    }//


                    //ToDo
                    //struct PointCloud{
                    //    std::vector<Point3Di> points;
                    //    int decimation;
                    //    bool show = true;
                    //};

                }
            }
        
            int index_to_remove = -1;
            for (int i = 0; i < point_clouds.size(); i++){
                std::string text = "show [" + std::to_string(i) + "]";
                ImGui::Checkbox(text.c_str(), &point_clouds[i].show);

                ImGui::SameLine();
                std::string text_dec = "dec [" + std::to_string(i) + "]";
                ImGui::InputInt(text_dec.c_str(), &point_clouds[i].decimation);
                if(point_clouds[i].decimation < 1){
                    point_clouds[i].decimation = 1;
                }
                ImGui::SameLine();
                std::string text_rem = "remove pc[" + std::to_string(i) + "]";
                if(ImGui::Button(text_rem.c_str())){
                    index_to_remove = i;
                }
                ImGui::SameLine();
                if(ImGui::Button("Save Point Cloud (laz)")){
                    std::vector<Point3Di> points;
                    for (int ii = 0; ii < point_clouds[i].points.size(); ii++){
                        //for(int p = 0; p < point_clouds[i].points.size(); p++){
                        //    points.push_back(point_clouds[i].points[p]);
                        //}
                        points.push_back(point_clouds[i].points[ii]);
                    }
                    for (int i = 0; i < point_clouds.size(); i++){
                        for(int p = 0; p < point_clouds[i].points.size(); p++){
                            points.push_back(point_clouds[i].points[p]);
                        }
                    }

                    std::shared_ptr<pfd::save_file> save_file;
                    std::string output_file_name = "";
                    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                    const auto t = [&]()
                    {
                        auto sel = pfd::save_file("Save file", "C:\\").result();
                        output_file_name = sel;
                        std::cout << "file to save: '" << output_file_name << "'" << std::endl;
                    };
                    std::thread t1(t);
                    t1.join();


                    if (output_file_name.size() > 0)
                    {
                        exportLaz(output_file_name, points);
                    }//


                    //ToDo
                    //bool exportLaz(const std::string &filename,
                    //    const std::vector<Point3Di> &pointcloud)
                }

            }
            if(index_to_remove != -1){
                std::vector<PointCloud> new_point_clouds;
                for (int j = 0; j < point_clouds.size(); j++){
                    if(index_to_remove != j){
                        new_point_clouds.push_back(point_clouds[j]);
                    }
                }
                point_clouds = new_point_clouds;
            }
        }

        if(ImGui::Button("save_buckets")){
            ros::NodeHandle nh;
            std_msgs::Int32 get_current_pc;
            get_current_pc.data = 1;
            pub_save_buckets.publish(get_current_pc);
            std::cout << "send pub_save_buckets" << std::endl;
        }
        ImGui::SameLine();
        if(ImGui::Button("load_buckets")){
            ros::NodeHandle nh;
            std_msgs::Int32 get_current_pc;
            get_current_pc.data = 1;
            pub_load_buckets.publish(get_current_pc);
            std::cout << "send pub_load_buckets" << std::endl;
        }

//ros::Publisher pub_save_buckets;
//ros::Publisher pub_load_buckets;


        /*
        if (ImGui::Button("Path")){
            nav_msgs::Path path;
            path.header.frame_id = "odom";
            path.header.stamp = ros::Time::now();
            path.poses.resize(3);
            for (int i =0; i < path.poses.size(); i++){
                auto & p = path.poses[i];
                p.pose.position.x = i;
                p.pose.orientation.w =1.0;
            }
            pub_path.publish(path);
        }
        if (ImGui::Button("vel"))
        {
            geometry_msgs::Twist twist;
            twist.angular.z= 0;
            twist.linear.x = 0.1;
            pub_vel.publish(twist);
        }*/

        ImGui::End();
    }
}

int main(int argc, char *argv[]){
    params.major = 0;
	params.minor = 1;

    ros::init(argc, argv, "BaseStationUnicorn");
    
    ros::NodeHandle nh;
    pub_get_current_pc = nh.advertise<std_msgs::Int32> ("get_current_pc", 1);
    pub_get_current_map = nh.advertise< std_msgs::Bool > ("get_current_map", 1);
    pub_reset_jackal = nh.advertise< std_msgs::Bool > ("reset_jackal", 1);
    pub_calib_height_above_ground = nh.advertise< std_msgs::Float32 > ("calib_height_above_ground", 1);
    pub_single_goal_forward = nh.advertise< nav_msgs::Path > ("single_goal_forward", 1);

    pub_get_last_goal_pc = nh.advertise<std_msgs::Int32> ("get_last_goal_pc", 1);

    pub_multiple_goals_to_robot = nh.advertise< nav_msgs::Path > ("multiple_goals_to_robot", 1);
    pub_multiple_goals_to_robot_execute = nh.advertise< std_msgs::Int32 > ("multiple_goals_to_robot_execute", 1);

    pub_save_buckets = nh.advertise<std_msgs::Int32> ("pub_save_buckets", 1);
    pub_load_buckets = nh.advertise<std_msgs::Int32> ("pub_load_buckets", 1);

//ros::Publisher pub_save_buckets;
//ros::Publisher pub_load_buckets;


    //pub_current_robot_pose = nh.advertise<nav_msgs::Path>  ("current_robot_pose", 1);


    sub_get_current_pc = nh.subscribe("current_point_cloud", 1, currentPointCloudCallback);
    sub_get_current_map = nh.subscribe("current_map", 1, currentMapCallback);
    sub_current_robot_pose = nh.subscribe("current_robot_pose", 1, currentRobotPoseCallback);
    sub_current_mission_goal = nh.subscribe("current_mission_goal", 1, currentMissionGoalCallback);

    pub_abort = nh.advertise< std_msgs::Bool > ("abort_mission", 1);
    //pub_path = nh.advertise< nav_msgs::Path > ("path", 1);
    //pub_vel = nh.advertise< geometry_msgs::Twist >("cmd_vel", 1);
    ros::Rate loop_rate(1);

    
    if (false == initGL(&argc, (char **)argv)) {
        return 4;
    }

    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutIdleFunc(idle);
    glutMainLoop();
	
	return 0;
}

bool initGL(int *argc, char **argv) {
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(window_width, window_height);
    glutCreateWindow("base station unicorn");
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
    //if(!params.planner_mode){
    glTranslatef(translate_x, translate_y, translate_z);
    glRotatef(rotate_x, 1.0, 0.0, 0.0);
    glRotatef(rotate_y, 0.0, 0.0, 1.0);
    //}

	main_loop(true);

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

    {
    std::lock_guard<std::mutex> lck(point_clouds_lock);
        glPointSize(params.pcs_point_size);
        for(const auto &pc:point_clouds){
            if(pc.show){
                glBegin(GL_POINTS);
                //for(const auto &p:pc.points){
                for(int i = 0 ; i < pc.points.size(); i += pc.decimation){
                    const auto & p = pc.points[i];
                    if(p.point.z() < params.visualization_height_threshold){
                        glColor3f(p.intensity / 256.0 + params.intensity_offset, p.intensity / 256.0 + params.intensity_offset, p.intensity / 256.0 + params.intensity_offset);
                        glVertex3f(p.point.x(), p.point.y(), p.point.z());
                    }
                }
                glEnd();
            }
        }
        glPointSize(1);
    }

    if(params.show_buckets)
    {
        std::lock_guard<std::mutex> lck(current_map_lock);
        glColor3f(1,0,0);
        glBegin(GL_POINTS);
        for(const auto &p:current_map){
            if(p.z() < params.visualization_height_threshold){
                glVertex3f(p.x(), p.y(), p.z());
            }
        }
        glEnd();
    }

    //robot pose
    {
        std::lock_guard<std::mutex> lck(params.current_robot_pose_lock);
        glBegin(GL_LINES);
            const auto &p = params.current_robot_pose;
            glColor3f(1.0f, 0.0f, 0.0f);
            glVertex3f(p(0,3), p(1,3), 0);
            glVertex3f(p(0,3) + p(0,0)*20, p(1,3) + p(1,0)*20, 0);

            glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(p(0,3), p(1,3), 0);
            glVertex3f(p(0,3) + p(0,1), p(1,3) + p(1,1), 0);

            glColor3f(0.0f, 0.0f, 1.0f);
            glVertex3f(p(0,3), p(1,3), 0);
            glVertex3f(p(0,3) + p(0,2), p(1,3) + p(1,2), 0);
	    glEnd();

        glColor3f(1.0, 0.2, 0.5);
        glBegin(GL_LINES);
            Eigen::Vector3d v0b(-params.robot_l * 0.5 + params.calib_x_offset, -params.robot_w * 0.5, -params.robot_h * 0.5);
            Eigen::Vector3d v1b( params.robot_l * 0.5 + params.calib_x_offset, -params.robot_w * 0.5, -params.robot_h * 0.5);
            Eigen::Vector3d v2b( params.robot_l * 0.5 + params.calib_x_offset,  params.robot_w * 0.5, -params.robot_h * 0.5);
            Eigen::Vector3d v3b(-params.robot_l * 0.5 + params.calib_x_offset,  params.robot_w * 0.5, -params.robot_h * 0.5);
            Eigen::Vector3d v0u(-params.robot_l * 0.5 + params.calib_x_offset, -params.robot_w * 0.5,  params.robot_h * 0.5);
            Eigen::Vector3d v1u( params.robot_l * 0.5 + params.calib_x_offset, -params.robot_w * 0.5,  params.robot_h * 0.5);
            Eigen::Vector3d v2u( params.robot_l * 0.5 + params.calib_x_offset,  params.robot_w * 0.5,  params.robot_h * 0.5);
            Eigen::Vector3d v3u(-params.robot_l * 0.5 + params.calib_x_offset,  params.robot_w * 0.5,  params.robot_h * 0.5);

            Eigen::Vector3d v0bt = p * v0b;
            Eigen::Vector3d v1bt = p * v1b;
            Eigen::Vector3d v2bt = p * v2b;
            Eigen::Vector3d v3bt = p * v3b;
            Eigen::Vector3d v0ut = p * v0u;
            Eigen::Vector3d v1ut = p * v1u;
            Eigen::Vector3d v2ut = p * v2u;
            Eigen::Vector3d v3ut = p * v3u;

            glVertex3f(v0bt.x(), v0bt.y(), v0bt.z());
            glVertex3f(v1bt.x(), v1bt.y(), v1bt.z());

            //glVertex3f(v1bt.x(), v1bt.y(), v1bt.z());
            //glVertex3f(v2bt.x(), v2bt.y(), v2bt.z());

            glVertex3f(v2bt.x(), v2bt.y(), v2bt.z());
            glVertex3f(v3bt.x(), v3bt.y(), v3bt.z());

            glVertex3f(v0bt.x(), v0bt.y(), v0bt.z());
            glVertex3f(v3bt.x(), v3bt.y(), v3bt.z());
            //
            glVertex3f(v0ut.x(), v0ut.y(), v0ut.z());
            glVertex3f(v1ut.x(), v1ut.y(), v1ut.z());

            //glVertex3f(v1ut.x(), v1ut.y(), v1ut.z());
            //glVertex3f(v2ut.x(), v2ut.y(), v2ut.z());

            glVertex3f(v2ut.x(), v2ut.y(), v2ut.z());
            glVertex3f(v3ut.x(), v3ut.y(), v3ut.z());

            glVertex3f(v0ut.x(), v0ut.y(), v0ut.z());
            glVertex3f(v3ut.x(), v3ut.y(), v3ut.z());
            //
            glVertex3f(v0bt.x(), v0bt.y(), v0bt.z());
            glVertex3f(v0ut.x(), v0ut.y(), v0ut.z());

            //glVertex3f(v1bt.x(), v1bt.y(), v1bt.z());
            //glVertex3f(v1ut.x(), v1ut.y(), v1ut.z());

            //glVertex3f(v2bt.x(), v2bt.y(), v2bt.z());
            //glVertex3f(v2ut.x(), v2ut.y(), v2ut.z());

            glVertex3f(v3bt.x(), v3bt.y(), v3bt.z());
            glVertex3f(v3ut.x(), v3ut.y(), v3ut.z());

            glColor3f(0, 1, 1);
            glVertex3f(v1bt.x(), v1bt.y(), v1bt.z());
            glVertex3f(v2bt.x(), v2bt.y(), v2bt.z());

            glVertex3f(v1ut.x(), v1ut.y(), v1ut.z());
            glVertex3f(v2ut.x(), v2ut.y(), v2ut.z());

            glVertex3f(v1bt.x(), v1bt.y(), v1bt.z());
            glVertex3f(v1ut.x(), v1ut.y(), v1ut.z());

            glVertex3f(v2bt.x(), v2bt.y(), v2bt.z());
            glVertex3f(v2ut.x(), v2ut.y(), v2ut.z());

        glEnd();
    }

    //mission goal
    {
        std::lock_guard<std::mutex> lckp(params.current_robot_pose_lock);
        std::lock_guard<std::mutex> lckg(params.current_mission_goal_lock);

        glBegin(GL_LINES);
            const auto &p = params.current_robot_pose;
            const auto &g = params.current_mission_goal;
            glColor3f(1.0f, 0.0f, 1.0f);
            glVertex3f(p(0,3), p(1,3), p(2,3));
            glVertex3f(g(0,3), g(1,3), g(2,3));
	    glEnd(); 

        glLineWidth(2);
        glBegin(GL_LINES);
            glColor3f(1.0f, 1.0f, 0.0f);
            glVertex3f(g(0,3), g(1,3), g(2,3));
            glVertex3f(g(0,3) + g(0,0) * 0.5, g(1,3) + g(1,0) * 0.5, g(2,3) + g(2,0) * 0.5);
        glEnd(); 
        glLineWidth(1);
    }
    
    //multiple goals
    {
        std::lock_guard<std::mutex> lck(multiple_goals_lock);

        glColor3f(1,0,0);
        glPointSize(3);
        glBegin(GL_POINTS);
        for(const auto &p:multiple_goals){
            glVertex3f(p.goal(0,3), p.goal(1,3), p.goal(2,3));
        }
        glEnd();
        glPointSize(1);

        glBegin(GL_LINE_STRIP);
        for(const auto &p:multiple_goals){
            glVertex3f(p.goal(0,3), p.goal(1,3), p.goal(2,3));
        }
        glEnd();

        glColor3f(1,1,0);
        glLineWidth(2);
        glBegin(GL_LINES);
        for(const auto &p:multiple_goals){
            glVertex3f(p.goal(0,3), p.goal(1,3), p.goal(2,3));
            glVertex3f(p.goal(0,3) + p.goal(0, 0) * 0.3, p.goal(1,3) + p.goal(1, 0) * 0.3, p.goal(2, 3) + p.goal(2, 0) * 0.3);
        }
        glEnd();
        glLineWidth(1);

        glColor3f(1,1,1);
        for(int i = 0; i < multiple_goals.size(); i++){
            glRasterPos3f(multiple_goals[i].goal(0,3), multiple_goals[i].goal(1,3) + 0.4, multiple_goals[i].goal(2,3) + 0.2);
		    glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, std::to_string(i).c_str()[0]);
        }
        
        if(multiple_goals.size() >= 2){
            glColor3f (1,1,1);
            for(int i = 0; i < params.num_scans_from_end; i++){
                if(multiple_goals.size() - 1 - i >= 0){
                    Eigen::Affine3d pose = multiple_goals[multiple_goals.size() - 1 - i].goal;
                    glBegin(GL_LINES);
                        glVertex3f(pose(0,3) - 0.5, pose(1,3), 0);
                        glVertex3f(pose(0,3) + 0.5, pose(1,3), 0);

                        glVertex3f(pose(0,3), pose(1,3) - 0.5, 0);
                        glVertex3f(pose(0,3), pose(1,3) + 0.5, 0);
                    glEnd();
                }
            }
        }
    }


    if(params.planner_mode){
        glColor3f(0, 1, 0);
        glBegin(GL_LINES);
            glVertex3f(params.goal_arrow_begin.x(), params.goal_arrow_begin.y(), params.goal_arrow_begin.z());
            glVertex3f(params.goal_arrow_end.x(), params.goal_arrow_end.y(), params.goal_arrow_end.z());
        glEnd();

        glPointSize(3);
        glBegin(GL_POINTS);
            glVertex3f(params.goal_arrow_begin.x(), params.goal_arrow_begin.y(), params.goal_arrow_begin.z());
        glEnd();
        glPointSize(1);
    }
    
    
    

    ImGui_ImplOpenGL2_NewFrame();
	ImGui_ImplGLUT_NewFrame();
	project_gui(params);

    {
        std::lock_guard<std::mutex> lck(multiple_goals_lock);
        for(int i = 0; i < multiple_goals.size(); i++){
            if (multiple_goals[i].gizmo){
                ImGuiIO &io = ImGui::GetIO();
                // ImGuizmo -----------------------------------------------
                ImGuizmo::BeginFrame();
                ImGuizmo::Enable(true);
                ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

                GLfloat projection[16];
                glGetFloatv(GL_PROJECTION_MATRIX, projection);

                GLfloat modelview[16];
                glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

                ImGuizmo::Manipulate(&modelview[0], &projection[0], ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y, ImGuizmo::WORLD, m_gizmo, NULL);
            
                Eigen::Affine3d m_g = Eigen::Affine3d::Identity();

                m_g(0, 0) = m_gizmo[0];
                m_g(1, 0) = m_gizmo[1];
                m_g(2, 0) = m_gizmo[2];
                m_g(3, 0) = m_gizmo[3];
                m_g(0, 1) = m_gizmo[4];
                m_g(1, 1) = m_gizmo[5];
                m_g(2, 1) = m_gizmo[6];
                m_g(3, 1) = m_gizmo[7];
                m_g(0, 2) = m_gizmo[8];
                m_g(1, 2) = m_gizmo[9];
                m_g(2, 2) = m_gizmo[10];
                m_g(3, 2) = m_gizmo[11];
                m_g(0, 3) = m_gizmo[12];
                m_g(1, 3) = m_gizmo[13];
                m_g(2, 3) = 0.0;//m_gizmo[14];
                m_g(3, 3) = m_gizmo[15];

                multiple_goals[i].goal = m_g;
            }
        }
    }

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
            if(params.planner_mode){
                params.goal_arrow_begin = GLWidgetGetOGLPos(x, y);
                params.goal_arrow_end = params.goal_arrow_begin;
            }


        } else if (state == GLUT_UP) {
            mouse_buttons = 0;

            if(params.planner_mode){
                params.goal_arrow_end = GLWidgetGetOGLPos(x, y);

                if(params.single_goal_forward){
                    double norm = (params.goal_arrow_begin - params.goal_arrow_end).norm();
                    if(norm > 0.1){
                        static int count = 0;
                        std::cout << "[" << count++ <<"] sending single_goal_forward" << std::endl;

                        //pub_single_goal_forward
                        Eigen::Vector3d x = params.goal_arrow_end - params.goal_arrow_begin;
                        x = x / (params.goal_arrow_end - params.goal_arrow_begin).norm();
                        Eigen::Vector3d z(0,0,1);

                        Eigen::Vector3d y = z.cross(x);
                        //std::cout << y << std::endl;

                        Eigen::Affine3d goal = Eigen::Affine3d::Identity();
                        goal(0,0) = x.x();
                        goal(1,0) = x.y();
                        goal(2,0) = x.z();

                        goal(0,1) = y.x();
                        goal(1,1) = y.y();
                        goal(2,1) = y.z();

                        goal(0,2) = z.x();
                        goal(1,2) = z.y();
                        goal(2,2) = z.z();

                        goal(0,3) = params.goal_arrow_begin.x();
                        goal(1,3) = params.goal_arrow_begin.y();
                        goal(2,3) = params.goal_arrow_begin.z();

                        nav_msgs::Path destination;
                        destination.header.frame_id = "odom";
                        destination.header.stamp = ros::Time::now();
                        destination.poses.resize(1);
                        auto & p = destination.poses[0];
                        p.pose.position.x = goal.translation().x();
                        p.pose.position.y = goal.translation().y();
                        p.pose.position.z = goal.translation().z();

                        Eigen::Quaterniond q(goal.rotation());
                        p.pose.orientation.w = q.w();
                        p.pose.orientation.x = q.x();
                        p.pose.orientation.y = q.y();
                        p.pose.orientation.z = q.z();
//pub_multiple_goals_to_robot
                        pub_single_goal_forward.publish(destination);

                        params.goal_arrow_begin = {0,0,0};
                        params.goal_arrow_end = {0,0,0};
                    }
                }
                if(params.multiple_goals){
                    std::lock_guard<std::mutex> lck(multiple_goals_lock);
                    double norm = (params.goal_arrow_begin - params.goal_arrow_end).norm();
                    if(norm > 0.1){
                        static int count = 0;
                        std::cout << "[" << count++ <<"] add goal" << std::endl;
                        Eigen::Vector3d x = params.goal_arrow_end - params.goal_arrow_begin;
                        x = x / (params.goal_arrow_end - params.goal_arrow_begin).norm();
                        Eigen::Vector3d z(0,0,1);
                        Eigen::Vector3d y = z.cross(x);
                        Eigen::Affine3d goal = Eigen::Affine3d::Identity();
                        goal(0,0) = x.x();
                        goal(1,0) = x.y();
                        goal(2,0) = x.z();

                        goal(0,1) = y.x();
                        goal(1,1) = y.y();
                        goal(2,1) = y.z();

                        goal(0,2) = z.x();
                        goal(1,2) = z.y();
                        goal(2,2) = z.z();

                        goal(0,3) = params.goal_arrow_begin.x();
                        goal(1,3) = params.goal_arrow_begin.y();
                        goal(2,3) = params.goal_arrow_begin.z();

                        params.goal_arrow_begin = {0,0,0};
                        params.goal_arrow_end = {0,0,0};
                        MuLtipleGoal _goal;
                        _goal.goal = goal;
                        _goal.gizmo = false;
                        multiple_goals.push_back(_goal);
                    }
                }
            }
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
        if(params.planner_mode){
            params.goal_arrow_end = GLWidgetGetOGLPos(x, y);
        }else{
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

void main_loop(bool render){
	auto start = std::chrono::steady_clock::now();

    ros::spinOnce();

	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-start;
	params.main_loop_time_execution = elapsed_seconds.count();
}

float distanceToPlane(const Plane &plane, const Eigen::Vector3d &p)
{
    return (plane.a * p.x() + plane.b * p.y() + plane.c * p.z() + plane.d);
}

Eigen::Vector3d rayIntersection(const LaserBeam &laser_beam, const Plane &plane)
{
    float TOLERANCE = 0.0001;
    Eigen::Vector3d out_point;
    out_point.x() = laser_beam.position.x();
    out_point.y() = laser_beam.position.y();
    out_point.z() = laser_beam.position.z();

    float a = plane.a * laser_beam.direction.x() + plane.b * laser_beam.direction.y() + plane.c * laser_beam.direction.z();

    if (a > -TOLERANCE && a < TOLERANCE)
    {
        return out_point;
    }

    float distance = distanceToPlane(plane, out_point);

    out_point.x() = laser_beam.position.x() - laser_beam.direction.x() * (distance / a);
    out_point.y() = laser_beam.position.y() - laser_beam.direction.y() * (distance / a);
    out_point.z() = laser_beam.position.z() - laser_beam.direction.z() * (distance / a);

    return out_point;
}

Eigen::Vector3d GLWidgetGetOGLPos(int x, int y)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posXnear, posYnear, posZnear;
    GLdouble posXfar, posYfar, posZfar;

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    winX = (float)x;
    winY = (float)viewport[3] - (float)y;

    LaserBeam laser_beam;
    gluUnProject(winX, winY, 0, modelview, projection, viewport, &posXnear, &posYnear, &posZnear);
    gluUnProject(winX, winY, -1000, modelview, projection, viewport, &posXfar, &posYfar, &posZfar);

    laser_beam.position.x() = posXnear;
    laser_beam.position.y() = posYnear;
    laser_beam.position.z() = posZnear;

    laser_beam.direction.x() = posXfar - posXnear;
    laser_beam.direction.y() = posYfar - posYnear;
    laser_beam.direction.z() = posZfar - posZnear;

    laser_beam.direction.normalize();

    Plane pl;

    pl.a = 0;
    pl.b = 0;
    pl.c = 1;
    pl.d = 0;

    Eigen::Vector3d pos = rayIntersection(laser_beam, pl);
    return pos;
}