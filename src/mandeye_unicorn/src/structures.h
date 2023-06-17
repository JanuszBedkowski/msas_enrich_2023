#ifndef _STRUCTURES_H_
#define _STRUCTURES_H_

#include <iostream>
#include <Eigen/Eigen>
#include <deque>

#define NUMBER_BB 20
#define NUMBER_BB_INNER 10

struct TaitBryanPose
{
	double px = 0;
	double py = 0;
	double pz = 0;
	double om = 0;
	double fi = 0;
	double ka = 0;
};

struct GridParameters2D_XY{
	float bounding_box_min_X;
	float bounding_box_min_Y;
	float bounding_box_max_X;
	float bounding_box_max_Y;
	float bounding_box_extension;
	int number_of_buckets_X;
	int number_of_buckets_Y;
	long long unsigned int number_of_buckets;
	float resolution_X;
	float resolution_Y;
};

struct GridParameters2D_XZ{
	float bounding_box_min_X;
	float bounding_box_min_Z;
	float bounding_box_max_X;
	float bounding_box_max_Z;
	float bounding_box_extension;
	int number_of_buckets_X;
	int number_of_buckets_Z;
	long long unsigned int number_of_buckets;
	float resolution_X;
	float resolution_Z;
};

struct GridParameters2D_Y{
	float bounding_box_min_Y;
	//float bounding_box_min_Z;
	float bounding_box_max_Y;
	//float bounding_box_max_Z;
	float bounding_box_extension;
	int number_of_buckets_Y;
	//int number_of_buckets_Z;
	long long unsigned int number_of_buckets;
	float resolution_Y;
	//float resolution_Z;
};

enum LineFollowerType{
	left_and_right,
	only_left,
	only_right
};

enum OptimizationErrorCode{
	not_initialized,
	ok,
	ground_points_size_not_sufficient,
	highest_points_size_not_sufficient,
	closest_left_points_size_not_sufficient,
	closest_right_points_size_not_sufficient,
	optimisation_failed,
	less_than_traktor_width,
	end_of_raw,
	obstacle_in_front
};

enum StatusAlgorithmExecution{
	on_init,
	on_expected,
	on_calibration_current_data,
	on_calibration_with_historical_data,
	on_end_of_raw,
	on_unknown_error
};

struct InnerSafetyCheckArea{
	bool is_ok = false;
	Eigen::Vector3d v0;
	Eigen::Vector3d v1;
	Eigen::Vector3d v2;
	unsigned char index = 0;
};

struct  SafetyCheckArea{
	//bool safety_bb = false;
	//Eigen::Vector3d safety_bb_0;
	//Eigen::Vector3d safety_bb_1;
	//Eigen::Vector3d safety_bb_2;
	std::pair<float, float> xy_collision_position;

	//	TaitBryanPose pose;
	Eigen::Affine3d pose;
	double length;
	double width;
	double height;
	bool alarm_safety = false;
	int threshold_number_of_points_inside = 100;
	int number_of_points_inside = 0;

	InnerSafetyCheckArea inner[NUMBER_BB_INNER];

	//std::deque<std::pair<Eigen::Vector3d, int>> additional_local_points;
};

struct Parameters{
	bool is_from_imgui = false;

	int number_max_points = 7500;
	Eigen::Affine3d calibration;
	std::string serial_dev_name = "/dev/ttyAMA0";
	float get_ground_points_res_X = 0.5;
	float get_ground_points_res_Y = 0.5;
	float get_ground_points_min_x = 0.0;
	float get_ground_points_max_x = 10.0;
	float get_highest_points_min_x = 1.0;
	float get_highest_points_max_x = 5.0;
	float get_ground_points_width = 5.0;
	float left_right_res_X = 2.0;
	float left_right_res_Z = 0.1;
	float get_ground_points_max_x_increment = 0.5;
	float get_ground_points_min_x_increment_threshold = 10.0;
	float get_ground_points_max_x_increment_threshold = 20.0;
	//float pc_filter_width = 2.0;
	//float pc_filter_length = 4.0;
	float pc_filter_width = 0.0;
	float pc_filter_length = 0.0;
	int mirror_type = 1;

	float plane_alpha_rad = M_PI * 0.5;
	float plane_d1 = -1.25;
	float plane_d2 = 1.25;

	//float plane_alpha_rad_ld = M_PI * 0.5;
	//float plane_d1_ld = -1;
	//float plane_d2_ld = 1;

	//float plane_alpha_rad_h = M_PI * 0.5;
	//float plane_d1_h = -1;
	//float plane_d2_h = 1;

	float max_above_ground = 1.5;
	float min_above_ground = 0.1;
	bool filter_points = false;

	float max_required_calibration_hits = 0;
	float step_hist_ka_deg = 0.1;
	float step_hist_y_cm = 1.0;
	float begin_hist_ka_deg = -90;
	float end_hist_ka_deg = -90;

	float end_of_row_offset = 5;
	float begin_hist_y_cm = -300;
	float end_hist_y_cm = 300;

	bool is_gui = false;

	float expected_distance_to_plane = 1.25;

	LineFollowerType type_follow=LineFollowerType::left_and_right;

	//int max_quality_points = 100;

	float param2a = 1000;
	float param1 = 1;
	float param2 = 1;
	float param3 = 1;
	float param4 = 1;
	float param5 = 0;
	//float param_cal_ka_deg = 0.0;
	bool params_updated = false;

	//2.Wersjonowanie, proszę abyś wysyłał wersję oprogramowania
	//w nieużywanych bajtach ramki LF bajt 11(major) i 12(minor)
	//są to bajty wykorzystywane wcześniej jako "lidar roll" czyli przechylenie.

	//0.25 - bez rozwijania
	//0.26 - z rozwijaniem
	//0.30 - wylaczenie 1.5m nad ziemia
	//0.31 - min abpove ground
	//0.32 -- wlaczona predkos i miazszosc
	//0.33 -- wlaczone wykrywanie kolozji
	//0.34 -- wylaczona kalibracja, poprawione strumienie i locki
	//char major = 0;
	//char minor = 44;
	char major = 1;
	char minor = 0;

	bool ignore_rs232_errors = true;
	//bool ignore_rs232_errors = false;

	//double threshold_update_d = 0.1;
	double threshold_update_d = 1.0;
	//bool use_initial_d1d2_guess = true;
	//bool use_initial_d1d2_guess = true;

	float a = 0.0;
	float b = 0.0;
	float c = 0.0;
	float lmq = 0.0;

	OptimizationErrorCode err = OptimizationErrorCode::not_initialized;
	//bool algorithm_results_view = false;
	StatusAlgorithmExecution status_lf = StatusAlgorithmExecution::on_init;
	//std::mutex points_lock;
	//std::deque < Eigen::Vector3d > points;
	int curr_task = -1;
	int previous_task = -1;
	uint8_t message[256];

	int sum_ground_points = 0;
	int sum_highest_points = 0;
	int sum_closest_left_points = 0;
	int sum_closest_right_points = 0;

	double main_loop_time_execution = 0.0;
	int sum_received_points = 0;
	int number_of_points_in_front = 0;

	//std::pair<float, float> xy_collision_path[20];

	SafetyCheckArea safety_check_areas[NUMBER_BB];

	int counter_pc = 0;
	int counter_send_safety = 0;

	float percentage_highest_points = 30.0;
};



inline void print_error(const OptimizationErrorCode &err){
	switch (err){
		case OptimizationErrorCode::not_initialized:{
			std::cout << "OptimizationErrorCode::not_initialized" << std::endl;
			break;
		}
		case OptimizationErrorCode::ok:{
			std::cout << "OptimizationErrorCode::ok" << std::endl;
			break;
		}
		case OptimizationErrorCode::ground_points_size_not_sufficient:{
			std::cout << "OptimizationErrorCode::ground_points_size_not_sufficient" << std::endl;
			break;
		}
		case OptimizationErrorCode::closest_left_points_size_not_sufficient:{
			std::cout << "OptimizationErrorCode::closest_left_points_size_not_sufficient" << std::endl;
			break;
		}
		case OptimizationErrorCode::closest_right_points_size_not_sufficient:{
			std::cout << "OptimizationErrorCode::closest_right_points_size_not_sufficient" << std::endl;
			break;
		}
		case OptimizationErrorCode::optimisation_failed:{
			std::cout << "OptimizationErrorCode::optimisation_failed" << std::endl;
			break;
		}
		case OptimizationErrorCode::less_than_traktor_width:{
			std::cout << "OptimizationErrorCode::less_than_traktor_width" << std::endl;
			break;
		}
		case OptimizationErrorCode::end_of_raw:{
			std::cout << "OptimizationErrorCode::end_of_raw" << std::endl;
			std::cout << "XXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
			std::cout << "XXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
			std::cout << "XXXXX END OF RAW XXXXXX" << std::endl;
			std::cout << "XXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
			std::cout << "XXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
			break;
		}
	}
}

struct Plane{
	double a;
	double b;
	double c;
	double d;
};

enum MirrorCode{
	not_used,
	bottom,
	up_left,
	up_right
};


inline double distance_to_plane(Eigen::Vector3d v, Plane plane){
	return plane.a * v.x() + plane.b * v.y() + plane.c * v.z() + plane.d;
}


struct GridParameters{
	double bounding_box_min_X;
	double bounding_box_min_Y;
	double bounding_box_min_Z;
	double bounding_box_max_X;
	double bounding_box_max_Y;
	double bounding_box_max_Z;
	double bounding_box_extension;
	int number_of_buckets_X;
	int number_of_buckets_Y;
	int number_of_buckets_Z;
	long long unsigned int number_of_buckets;
	double resolution_X;
	double resolution_Y;
	double resolution_Z;
};

struct PointBucketIndexPair{
	int index_of_point;
	long long unsigned int index_of_bucket;
	//int index_pose;
};

struct Bucket{
	long long unsigned int index_begin;
	long long unsigned int index_end;
	long long unsigned int number_of_points;
	Eigen::Vector3d mean;
	Eigen::Matrix3d cov;
};

/*struct Bucket
{
	long long unsigned int index_begin;
	long long unsigned int index_end;
	long long unsigned int number_of_points;
	Eigen::Vector3d mean;
	Eigen::Matrix3d cov;
};*/

//struct SafetyCheckArea{
//	TaitBryanPose pose;
//	double length;
//	double width;
//	double height;
//	bool safe;
//	int threshold;
//};

#endif
