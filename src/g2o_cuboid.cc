#include "g2o_cuboid.h"

namespace g2o
{

SE3Quat exptwist_norollpitch(const Vector6d &update)
{
    Vector3d omega;
    for (int i = 0; i < 3; i++)
        omega[i] = update[i];
    Vector3d upsilon;
    for (int i = 0; i < 3; i++)
        upsilon[i] = update[i + 3];

    double theta = omega.norm();
    Matrix3d Omega = skew(omega);

    Matrix3d R;
    R << cos(omega(2)), -sin(omega(2)), 0,
        sin(omega(2)), cos(omega(2)), 0,
        0, 0, 1;

    Matrix3d V;
    if (theta < 0.00001)
    {
        V = R;
    }
    else
    {
        Matrix3d Omega2 = Omega * Omega;

        V = (Matrix3d::Identity() + (1 - cos(theta)) / (theta * theta) * Omega + (theta - sin(theta)) / (pow(theta, 3)) * Omega2);
    }

    return SE3Quat(Quaterniond(R), V * upsilon);
}

// cuboid vertex 
void VertexCuboid::oplusImpl(const double *update_)
{
    Eigen::Map<const Vector9d> update(update_);

    g2o::cuboid newcube;
    if (whether_fixrotation)
    {
        newcube.pose.setTranslation(_estimate.pose.translation() + update.segment<3>(3));
    }
    else if (whether_fixrollpitch) //NOTE this only works for cuboid already has parallel to ground. otherwise update_z will also change final RPY
    {
        Vector9d update2 = update;
        update2(0) = 0;
        update2(1) = 0;
        newcube.pose = _estimate.pose * exptwist_norollpitch(update2.head<6>()); //NOTE object pose is from object to world!!!!
    }
    else
        newcube.pose = _estimate.pose * SE3Quat::exp(update.head<6>());

    if (whether_fixheight) // use previous height
        newcube.setTranslation(Vector3d(newcube.translation()(0), _estimate.translation()(1), newcube.translation()(2)));

    if (fixedscale(0) > 0) // if fixed scale is set, use it.
        newcube.scale = fixedscale;
    else
        newcube.scale = _estimate.scale + update.tail<3>();

    setEstimate(newcube);
}

// camera -object 2D projection error, rectangle difference, could also change to iou
void EdgeSE3CuboidProj::computeError()
{
    const VertexSE3Expmap *SE3Vertex = dynamic_cast<const VertexSE3Expmap *>(_vertices[0]); //  world to camera pose
    const VertexCuboid *cuboidVertex = dynamic_cast<const VertexCuboid *>(_vertices[1]);    //  object pose to world

    SE3Quat cam_pose_Tcw = SE3Vertex->estimate();
    cuboid global_cube = cuboidVertex->estimate();

    Vector4d rect_project = global_cube.projectOntoImageBbox(cam_pose_Tcw, Kalib); // center, width, height

    _error = rect_project - _measurement;

    // double left = max(rect_project(0)-rect_project(2), _measurement(0)-_measurement(2));
    // double top =  max(rect_project(1)-rect_project(3), _measurement(2)-_measurement(3));
    // double right = min(rect_project(0)+rect_project(2), _measurement(0)+_measurement(2));
    // double bottom = min(rect_project(1)+rect_project(3), _measurement(2)+_measurement(3));
    // double inter_area = max(0.0, right-left) * max(0.0, bottom-top);
    // double union_area = 4*rect_project(2)*rect_project(3) + 4*_measurement(2)*_measurement(3) - inter_area;
    // double iou_2d = inter_area/union_area;
    // // _error = Vector4d(iou_2d, 0, 0, 0);
    // _error = Vector4d(iou_2d, iou_2d, iou_2d, iou_2d);
}

double EdgeSE3CuboidProj::get_error_norm()
{
    computeError();
    std::cout << "rect_project: " << _measurement.transpose() + _error.transpose() << std::endl;
    std::cout << "_measurement: " << _measurement.transpose() << std::endl;
    std::cout << "_error: " << _error.transpose() << std::endl;
    return _error.norm();
}

// camera -object 2D projection error, 8 corners prjection error 
void EdgeSE3CuboidCornerProj::computeError()
{
    const VertexSE3Expmap *SE3Vertex = dynamic_cast<const VertexSE3Expmap *>(_vertices[0]); //  world to camera pose
    const VertexCuboid *cuboidVertex = dynamic_cast<const VertexCuboid *>(_vertices[1]);    //  object pose to world

    SE3Quat cam_pose_Tcw = SE3Vertex->estimate();
    cuboid global_cube = cuboidVertex->estimate();

    Matrix2Xd corners_2d = global_cube.projectOntoImage(cam_pose_Tcw, Kalib); // center, width, height
    Vector16d corners_2d_vec;
    for (size_t i = 0; i < 8; i++)
    {
        corners_2d_vec(i*2) = corners_2d(0,i);
        corners_2d_vec(i*2+1) = corners_2d(1,i);
    }
    
    _error = corners_2d_vec - _measurement;
}

double EdgeSE3CuboidCornerProj::get_error_norm()
{
    computeError();
    // std::cout << "rect_project: " << _measurement.transpose() + _error.transpose() << std::endl;
    // std::cout << "_measurement: " << _measurement.transpose() << std::endl;
    // std::cout << "_error: " << _error.transpose() << std::endl;
    return _error.norm();
}

// one object connected with all fixed points. only optimize object.  want object to contain points
void EdgePointCuboidOnlyObject::computeError()
{
    const VertexCuboid *cuboidVertex = dynamic_cast<const VertexCuboid *>(_vertices[0]); // world to camera pose

    _error.setZero();

    const g2o::cuboid &estimate_cube = cuboidVertex->estimate();

    Vector3d point_edge_error;
    point_edge_error.setZero();
    for (size_t i = 0; i < object_points.size(); i++) // use abs  otherwise   pos neg will counteract by different pts.     maybe each edge one pt?
        point_edge_error += estimate_cube.point_boundary_error(object_points[i], max_outside_margin_ratio).cwiseAbs();
    if (object_points.size() > 0)
        point_edge_error = point_edge_error / object_points.size();

    point_edge_error = point_edge_error.array() / estimate_cube.scale.array(); //scale it

    // add prior shape dimension error?
    double prior_weight = 0.2;
    // double prior_weight = 0.0;
    Vector3d prior_shape_error = estimate_cube.scale; // setZero?  or penalize large box! or set a range?
    if (prior_object_half_size(0) > 0)                // if prior shape is being set, such as KITTI, then give large weight for shape error
    {
        prior_weight = 50.0;
        prior_shape_error = ((estimate_cube.scale - prior_object_half_size).array() / prior_object_half_size.array()).cwiseAbs();
    }

    _error = 1.0 * point_edge_error + prior_weight * prior_shape_error;
}

Vector3d EdgePointCuboidOnlyObject::computeError_debug()
{
    computeError();
    return _error;
}

} // namespace g2o
