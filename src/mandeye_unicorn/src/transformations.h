#ifndef _TRANSFORMATIONS_H_
#define _TRANSFORMATIONS_H_

#include "structures.h"

inline TaitBryanPose pose_tait_bryan_from_affine_matrix(Eigen::Affine3d m){
	TaitBryanPose pose;

	pose.px = m(0,3);
	pose.py = m(1,3);
	pose.pz = m(2,3);

	if (m(0,2) < 1) {
		if (m(0,2) > -1) {
			//case 1
			pose.fi = asin(m(0,2));
			pose.om = atan2(-m(1,2), m(2,2));
			pose.ka = atan2(-m(0,1), m(0,0));

			return pose;
		}
		else //r02 = −1
		{
			//case 2
			// not a unique solution: thetaz − thetax = atan2 ( r10 , r11 )
			pose.fi = -M_PI / 2.0;
			pose.om = -atan2(m(1,0), m(1,1));
			pose.ka = 0;
			return pose;
		}
	}
	else {
		//case 3
		// r02 = +1
		// not a unique solution: thetaz + thetax = atan2 ( r10 , r11 )
		pose.fi = M_PI / 2.0;
		pose.om = atan2(m(1,0), m(1,1));
		pose.ka = 0.0;
		return pose;
	}

	return pose;
}

inline Eigen::Affine3d affine_matrix_from_pose_tait_bryan(TaitBryanPose pose)
{
	Eigen::Affine3d m = Eigen::Affine3d::Identity();

	double sx = sin(pose.om);
	double cx = cos(pose.om);
	double sy = sin(pose.fi);
	double cy = cos(pose.fi);
	double sz = sin(pose.ka);
	double cz = cos(pose.ka);

	m(0,0) = cy * cz;
	m(1,0) = cz * sx * sy + cx * sz;
	m(2,0) = -cx * cz * sy + sx * sz;

	m(0,1) = -cy * sz;
	m(1,1) = cx * cz - sx * sy * sz;
	m(2,1) = cz * sx + cx * sy * sz;

	m(0,2) = sy;
	m(1,2) = -cy * sx;
	m(2,2) = cx * cy;

	m(0,3) = pose.px;
	m(1,3) = pose.py;
	m(2,3) = pose.pz;

	return m;
}

inline double normalize_angle(double src_rad)
{
	double arg;

	arg = fmod(src_rad, 2.0 * M_PI);
	if (arg < 0)
		arg = arg + 2.0 * M_PI;
	if (arg > M_PI)
		arg = arg - 2.0 * M_PI;
	return arg;
}

#endif
