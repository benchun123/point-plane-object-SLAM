/**
* This file is part of CubeSLAM
*
* Copyright (C) 2018  Shichao Yang (Carnegie Mellon Univ)
*/
#pragma once

#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "matrix_utils.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm> // std::swap

typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 16, 1> Vector16d;

namespace g2o
{

using namespace Eigen;

class cuboid
{
public:
	SE3Quat pose;   // 6 dof for object, object to world by default
	Vector3d scale; // [length, width, height]  half!

	cuboid()
	{
		pose = SE3Quat();
		scale.setZero();
	}

	// xyz roll pitch yaw half_scale
	inline void fromMinimalVector(const Vector9d &v)
	{
		Eigen::Quaterniond posequat = zyx_euler_to_quat(v(3), v(4), v(5));
		pose = SE3Quat(posequat, v.head<3>());
		scale = v.tail<3>();
	}

	// xyz quaternion, half_scale
	inline void fromVector(const Vector10d &v)
	{
		pose.fromVector(v.head<7>());
		scale = v.tail<3>();
	}

	inline const Vector3d &translation() const { return pose.translation(); }
	inline void setTranslation(const Vector3d &t_) { pose.setTranslation(t_); }
	inline void setRotation(const Quaterniond &r_) { pose.setRotation(r_); }
	inline void setRotation(const Matrix3d &R) { pose.setRotation(Quaterniond(R)); }
	inline void setScale(const Vector3d &scale_) { scale = scale_; }

	// apply update to current cuboid. exponential map
	cuboid exp_update(const Vector9d &update) // apply update to current cuboid
	{
		cuboid res;
		res.pose = this->pose * SE3Quat::exp(update.head<6>()); // NOTE bug before. switch position
		res.scale = this->scale + update.tail<3>();
		return res;
	}

	// actual error between two cuboids.
	Vector9d cube_log_error(const cuboid &newone) const
	{
		Vector9d res;
		SE3Quat pose_diff = newone.pose.inverse() * this->pose;
		res.head<6>() = pose_diff.log(); //treat as se3 log error. could also just use yaw error
		res.tail<3>() = this->scale - newone.scale;
		return res;
	}

	// function called by g2o.
	Vector9d min_log_error(const cuboid &newone, bool print_details = false) const
	{
		bool whether_rotate_cubes = true; // whether rotate cube to find smallest error
		if (!whether_rotate_cubes)
			return cube_log_error(newone);

		// NOTE rotating cuboid... since we cannot determine the front face consistenly, different front faces indicate different yaw, scale representation.
		// need to rotate all 360 degrees (global cube might be quite different from local cube)
		// this requires the sequential object insertion. In this case, object yaw practically should not change much. If we observe a jump, we can use code
		// here to adjust the yaw.
		Vector4d rotate_errors_norm;
		Vector4d rotate_angles(-1, 0, 1, 2); // rotate -90 0 90 180
		Eigen::Matrix<double, 9, 4> rotate_errors;
		for (int i = 0; i < rotate_errors_norm.rows(); i++)
		{
			cuboid rotated_cuboid = newone.rotate_cuboid(rotate_angles(i) * M_PI / 2.0); // rotate new cuboids
			Vector9d cuboid_error = this->cube_log_error(rotated_cuboid);
			rotate_errors_norm(i) = cuboid_error.norm();
			rotate_errors.col(i) = cuboid_error;
		}
		int min_label;
		rotate_errors_norm.minCoeff(&min_label);
		if (print_details)
			if (min_label != 1)
				std::cout << "Rotate cube   " << min_label << std::endl;
		return rotate_errors.col(min_label);
	}

	// change front face by rotate along current body z axis. another way of representing cuboid. representing same cuboid (IOU always 1)
	cuboid rotate_cuboid(double yaw_angle) const // to deal with different front surface of cuboids
	{
		cuboid res;
		SE3Quat rot(Eigen::Quaterniond(cos(yaw_angle * 0.5), 0, 0, sin(yaw_angle * 0.5)), Vector3d(0, 0, 0)); // change yaw to rotation.
		res.pose = this->pose * rot;
		res.scale = this->scale;
		if ((yaw_angle == M_PI / 2.0) || (yaw_angle == -M_PI / 2.0) || (yaw_angle == 3 * M_PI / 2.0))
			std::swap(res.scale(0), res.scale(1));

		return res;
	}

	// transform a local cuboid to global cuboid  Twc is camera pose. from camera to world
	cuboid transform_from(const SE3Quat &Twc) const
	{
		cuboid res;
		res.pose = Twc * this->pose;
		res.scale = this->scale;
		return res;
	}

	// transform a global cuboid to local cuboid  Twc is camera pose. from camera to world
	cuboid transform_to(const SE3Quat &Twc) const
	{
		cuboid res;
		res.pose = Twc.inverse() * this->pose;
		res.scale = this->scale;
		return res;
	}

	// xyz roll pitch yaw half_scale
	inline Vector9d toMinimalVector() const
	{
		Vector9d v;
		// v.head<6>() = pose.toXYZPRYVector();
		Vector7d pose_7d = pose.toVector();
		Eigen::Quaterniond q(pose_7d(6), pose_7d(3), pose_7d(4), pose_7d(5));
		double qw = q.w();
		double qx = q.x();
		double qy = q.y();
		double qz = q.z();
		double roll = atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy));
		double pitch = asin(2*(qw*qy-qz*qx));
		double yaw = atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz));
		v.head<3>() = pose_7d.head<3>();
		v[3]=roll;
		v[4]=pitch;
		v[5]=yaw;
		v.tail<3>() = scale;
		return v;
	}

	// xyz quaternion, half_scale
	inline Vector10d toVector() const
	{
		Vector10d v;
		v.head<7>() = pose.toVector();
		v.tail<3>() = scale;
		return v;
	}

	Matrix4d similarityTransform() const
	{
		Matrix4d res = pose.to_homogeneous_matrix();
		Matrix3d scale_mat = scale.asDiagonal();
		res.topLeftCorner<3, 3>() = res.topLeftCorner<3, 3>() * scale_mat;
		return res;
	}

	// this*inv(other)
	cuboid timesInverse(const cuboid &othercube) const
	{
		Matrix4d current_homomat = similarityTransform();
		Matrix4d other_homomat = othercube.similarityTransform();
		Matrix4d result_homomat = current_homomat * other_homomat.inverse(); // [RS, t]
		Matrix3d result_rot = pose.rotation().toRotationMatrix() * othercube.pose.rotation().toRotationMatrix().inverse();

		cuboid res;
		res.setTranslation(result_homomat.col(3).head<3>());
		res.setScale(scale.array() / othercube.scale.array());
		res.setRotation(result_rot);

		return res;
	}

	// 8 corners 3*8 matrix, each col is x y z of a corner
	Matrix3Xd compute3D_BoxCorner() const
	{
		Matrix3Xd corners_body;
		corners_body.resize(3, 8);
		corners_body << 1, 1, -1, -1, 1, 1, -1, -1,
			1, -1, -1, 1, 1, -1, -1, 1,
			-1, -1, -1, -1, 1, 1, 1, 1;
		Matrix3Xd corners_world = homo_to_real_coord<double>(similarityTransform() * real_to_homo_coord<double>(corners_body));
		return corners_world;
	}

	// project corners onto image to get 8 points.  cam pose: world to cam
	Matrix2Xd projectOntoImage(const SE3Quat &campose_cw, const Matrix3d &Kalib) const
	{
		Matrix3Xd corners_3d_world = compute3D_BoxCorner();
		Matrix2Xd corner_2d = homo_to_real_coord<double>(Kalib * homo_to_real_coord<double>(campose_cw.to_homogeneous_matrix() * real_to_homo_coord<double>(corners_3d_world)));
		return corner_2d;
	}

	// get rectangles after projection  [topleft, bottomright]
	Vector4d projectOntoImageRect(const SE3Quat &campose_cw, const Matrix3d &Kalib) const
	{
		Matrix3Xd corners_3d_world = compute3D_BoxCorner();
		Matrix2Xd corner_2d = homo_to_real_coord<double>(Kalib * homo_to_real_coord<double>(campose_cw.to_homogeneous_matrix() * real_to_homo_coord<double>(corners_3d_world)));
		Vector2d bottomright = corner_2d.rowwise().maxCoeff(); // x y
		Vector2d topleft = corner_2d.rowwise().minCoeff();
		return Vector4d(topleft(0), topleft(1), bottomright(0), bottomright(1));
	}

	// get rectangles after projection  [center, width, height]
	Vector4d projectOntoImageBbox(const SE3Quat &campose_cw, const Matrix3d &Kalib) const
	{
		Vector4d rect_project = projectOntoImageRect(campose_cw, Kalib); // top_left, bottom_right  x1 y1 x2 y2
		Vector2d rect_center = (rect_project.tail<2>() + rect_project.head<2>()) / 2;
		Vector2d widthheight = rect_project.tail<2>() - rect_project.head<2>();
		return Vector4d(rect_center(0), rect_center(1), widthheight(0), widthheight(1));
	}

	// compute point surface error on the object
	Vector3d point_boundary_error(const Vector3d &point, const double max_outside_margin_ratio, double point_scale = 1) const
	{
		// transform the point to local object frame  TODO actually can compute gradient analytically...
		Vector3d local_pt = point_scale * (this->pose.inverse() * point).cwiseAbs(); // change global point to local cuboid body frame.  make it positive.
		Vector3d error;

		// if point is within the cube, error=0, otherwise penalty how far it is outside cube
		for (int i = 0; i < 3; i++)
		{
			if (local_pt(i) < this->scale(i))
				error(i) = 0;
			else if (local_pt(i) < (max_outside_margin_ratio + 1) * this->scale(i))
				error(i) = local_pt(i) - this->scale(i);
			else
				error(i) = max_outside_margin_ratio * this->scale(i); // if points two far, give a constant error, don't optimize.
		}

		return error;
	}

};

class VertexCuboid : public BaseVertex<9, cuboid> // NOTE  this vertex stores object pose to world
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	VertexCuboid()
	{
		fixedscale.setZero();
		whether_fixrollpitch = false;
		whether_fixrotation = false;
		whether_fixheight = false;
	};

	virtual void setToOriginImpl()
	{
		_estimate = cuboid();
		if (fixedscale(0) > 0)
			_estimate.scale = fixedscale;
	}

	virtual void oplusImpl(const double *update_);

	virtual bool read(std::istream &is) { return true; };
	virtual bool write(std::ostream &os) const { return os.good(); };

	// some mode parameters. a more efficient way is to create separate vertex
	Vector3d fixedscale;	   // if want to fix scale, set it to be true value
	bool whether_fixrollpitch; // for ground object, only update yaw
	bool whether_fixrotation;  // don't update any rotation
	bool whether_fixheight;	// object height is fixed
};

// camera -object 2D projection error, rectangle difference, could also change to iou
class EdgeSE3CuboidProj : public BaseBinaryEdge<4, Vector4d, VertexSE3Expmap, VertexCuboid>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeSE3CuboidProj(){};

	virtual bool read(std::istream &is) { return true; };
	virtual bool write(std::ostream &os) const { return os.good(); };

	void computeError();
	double get_error_norm();
	Matrix3d Kalib;
};

// camera -object 2D projection error, 8 corners prjection error 
class EdgeSE3CuboidCornerProj : public BaseBinaryEdge<16, Vector16d, VertexSE3Expmap, VertexCuboid>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeSE3CuboidCornerProj(){};

	virtual bool read(std::istream &is) { return true; };
	virtual bool write(std::ostream &os) const { return os.good(); };

	void computeError();
	double get_error_norm();
	Matrix3d Kalib;
};


// camera -object 3D error
class EdgeSE3Cuboid : public BaseBinaryEdge<9, cuboid, VertexSE3Expmap, VertexCuboid>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeSE3Cuboid(){};

	virtual bool read(std::istream &is) { return true; };
	virtual bool write(std::ostream &os) const { return os.good(); };

	void computeError()
	{
		const VertexSE3Expmap *SE3Vertex = static_cast<const VertexSE3Expmap *>(_vertices[0]); //  world to camera pose
		const VertexCuboid *cuboidVertex = static_cast<const VertexCuboid *>(_vertices[1]);	//  object pose to world

		SE3Quat cam_pose_Twc = SE3Vertex->estimate().inverse();
		cuboid global_cube = cuboidVertex->estimate();
		cuboid esti_global_cube = _measurement.transform_from(cam_pose_Twc);
		_error = global_cube.min_log_error(esti_global_cube);
	}

	double get_error_norm() //for debug
	{
		const VertexSE3Expmap *SE3Vertex = static_cast<const VertexSE3Expmap *>(_vertices[0]);
		const VertexCuboid *cuboidVertex = static_cast<const VertexCuboid *>(_vertices[1]);

		SE3Quat cam_pose_Twc = SE3Vertex->estimate().inverse();
		cuboid global_cube = cuboidVertex->estimate();
		cuboid esti_global_cube = _measurement.transform_from(cam_pose_Twc);
		return global_cube.min_log_error(esti_global_cube).norm();
	}
};

// one object connected with all fixed points. only optimize object.  want object to contain points
class EdgePointCuboidOnlyObject : public BaseUnaryEdge<3, Vector3d, VertexCuboid>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgePointCuboidOnlyObject()
	{
		print_details = false;
		prior_object_half_size.setZero();
	};

	virtual bool read(std::istream &is) { return true; };
	virtual bool write(std::ostream &os) const { return os.good(); };

	bool print_details;

	void computeError();
	Vector3d computeError_debug();

	std::vector<Vector3d> object_points; // all the fixed points.
	double max_outside_margin_ratio;	 // truncate the error if point is too far from object
	Vector3d prior_object_half_size;	 // give a prior object size, otherwise a huge cuboid can always contain all points
};


} // namespace g2o