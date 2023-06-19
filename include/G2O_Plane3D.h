//
// Created by fishmarch on 19-5-28.
//

#ifndef ORB_SLAM2_G2O_PLANE3D_H
#define ORB_SLAM2_G2O_PLANE3D_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "Thirdparty/g2o/g2o/stuff/misc.h"
#include "Thirdparty/g2o/g2o/core/eigen_types.h"
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/hyper_graph_action.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "g2o_cuboid.h"


namespace g2o 
{

    class  Plane3D 
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        friend Plane3D operator*(const Isometry3D& t, const Plane3D& plane);

        Plane3D(){
            Vector4D v;
            v << 1., 0., 0., -1.;
            fromVector(v);
        }

        Plane3D(const Vector4D& v){
            fromVector(v);
        }

        inline Vector4D toVector() const {
            return _coeffs;
        }

        inline const Vector4D& coeffs() const {return _coeffs;}

        inline void fromVector(const Vector4D& coeffs_) {
            _coeffs=coeffs_;
            normalize(_coeffs);
        }

        static double azimuth(const Vector3D& v) {
            return std::atan2(v(1),v(0));
        }

        static  double elevation(const Vector3D& v) {
            return std::atan2(v(2), v.head<2>().norm());
        }

        double distance() const {
            return -_coeffs(3);
        }

        Vector3D normal() const {
            return _coeffs.head<3>();
        }

        static Matrix3D rotation(const Vector3D& v)  {
            double _azimuth = azimuth(v);
            double _elevation = elevation(v);
            Eigen::AngleAxisd azimuth_v(_azimuth, Vector3D::UnitZ());
            Eigen::AngleAxisd elevation_v(- _elevation, Vector3D::UnitY());
            return (azimuth_v * elevation_v).toRotationMatrix();
        }

        inline void oplus(const Vector3D& v){
            //construct a normal from azimuth and evelation;
            double _azimuth=v[0];
            double _elevation=v[1];
            double s=std::sin(_elevation), c=std::cos(_elevation);
            Vector3D n (c*std::cos(_azimuth), c*std::sin(_azimuth), s) ;

            // rotate the normal
            Matrix3D R=rotation(normal());
            double d=distance()+v[2];
            _coeffs.head<3>() = R*n;
            _coeffs(3) = -d;
            normalize(_coeffs);
        }

        inline Vector3D ominus(const Plane3D& plane){
            //construct the rotation that would bring the plane normal in (1 0 0)
            Matrix3D R=rotation(normal()).transpose();
            Vector3D n=R*plane.normal();
            double d=distance()-plane.distance();
            return Vector3D(azimuth(n), elevation(n), d);
        }

        inline Vector2D ominus_ver(const Plane3D& plane){
            //construct the rotation that would bring the plane normal in (1 0 0)
            Vector3D v = normal().cross(plane.normal());
            Eigen::AngleAxisd ver(M_PI/2, v/v.norm());
            Vector3D b = ver * normal();

            Matrix3D R = rotation(b).transpose();
            Vector3D n = R*plane.normal();
            return Vector2D(azimuth(n), elevation(n));
        }

        inline Vector2D ominus_par(const Plane3D& plane){
            //construct the rotation that would bring the plane normal in (1 0 0)
            Vector3D nor = normal();
            if(plane.normal().dot(nor) < 0)
                nor = -nor;
            Matrix3D R=rotation(nor).transpose();
            Vector3D n=R*plane.normal();

            return Vector2D(azimuth(n), elevation(n));
        }
        //protected:

        static inline void normalize(Vector4D& coeffs) {
            double n=coeffs.head<3>().norm();
            coeffs = coeffs * (1./n);
            if(coeffs(3)<0.0)
                coeffs = -coeffs;
        }

        Vector4D _coeffs;
    };

    // input t : transform matrix applying to the point
    inline Plane3D operator*(const Isometry3D& t, const Plane3D& plane){
        Vector4D v=plane._coeffs;
        Vector4D v2;
        Matrix3D R=t.rotation();
        v2.head<3>() = R*v.head<3>();
        v2(3)=v(3) - t.translation().dot(v2.head<3>());
        if(v2(3) < 0.0)
            v2 = -v2;
        return Plane3D(v2);
    };

    class  VertexPlane : public BaseVertex<3, Plane3D>
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexPlane();

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

        virtual void setToOriginImpl() { _estimate = Plane3D(); }

        virtual void oplusImpl(const double* update_) {
            Eigen::Map<const Vector3D> update(update_);
            _estimate.oplus(update);
        }

        virtual bool setEstimateDataImpl(const double* est){
            Eigen::Map<const Vector4D> _est(est);
            _estimate.fromVector(_est);
            return true;
        }

        virtual bool getEstimateData(double* est) const{
            Eigen::Map<Vector4D> _est(est);
            _est = _estimate.toVector();
            return true;
        }

        virtual int estimateDimension() const {
            return 4;
        }

    };

    class EdgePlane : public BaseBinaryEdge<3, Plane3D, VertexPlane, VertexSE3Expmap> 
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgePlane();
        void computeError()
        {
            const VertexSE3Expmap* poseVertex = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexPlane* planeVertex = static_cast<const VertexPlane*>(_vertices[0]);

            const Plane3D& plane = planeVertex->estimate();
            // measurement function: remap the plane in global coordinates
            Isometry3D w2n = poseVertex->estimate();
            Plane3D localPlane = w2n*plane;

            _error = localPlane.ominus(_measurement);
        }

        void setMeasurement(const Plane3D& m){
            _measurement = m;
        }

        bool isDepthPositive(){
            const VertexSE3Expmap* poseVertex = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexPlane* planeVertex = static_cast<const VertexPlane*>(_vertices[0]);

            const Plane3D& plane = planeVertex->estimate();
            // measurement function: remap the plane in global coordinates
            Isometry3D w2n = poseVertex->estimate();
            Plane3D localPlane = w2n*plane;

            return localPlane.distance() > 0;
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;
    };


    class EdgeVerticalPlane : public BaseBinaryEdge<2, Plane3D, VertexPlane, VertexSE3Expmap> 
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeVerticalPlane();
        void computeError()
        {
            const VertexSE3Expmap* poseVertex = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexPlane* planeVertex = static_cast<const VertexPlane*>(_vertices[0]);

            const Plane3D& plane = planeVertex->estimate();
            // measurement function: remap the plane in global coordinates
            Isometry3D w2n = poseVertex->estimate();
            Plane3D localPlane = w2n*plane;

            _error = localPlane.ominus_ver(_measurement);
        }

        void setMeasurement(const Plane3D& m){
            _measurement = m;
        }

        bool isDepthPositive(){
            const VertexSE3Expmap* poseVertex = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexPlane* planeVertex = static_cast<const VertexPlane*>(_vertices[0]);

            const Plane3D& plane = planeVertex->estimate();
            // measurement function: remap the plane in global coordinates
            Isometry3D w2n = poseVertex->estimate();
            Plane3D localPlane = w2n*plane;

            return localPlane.distance() > 0;
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;
    };

    class EdgeTwoVerPlanes : public BaseBinaryEdge<2, double, VertexPlane, VertexPlane> 
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeTwoVerPlanes() {}
        void computeError()
        {
            const VertexPlane* planeVertex1 = static_cast<const VertexPlane*>(_vertices[0]);
            const VertexPlane* planeVertex2 = static_cast<const VertexPlane*>(_vertices[1]);

            Plane3D plane1 = planeVertex1->estimate();
            const Plane3D& plane2 = planeVertex2->estimate();
            // measurement function: remap the plane in global coordinates
            _error = plane1.ominus_ver(plane2);
        }

        virtual bool read(std::istream& is) {}
        virtual bool write(std::ostream& os) const{}
    };


    class EdgeParallelPlane : public BaseBinaryEdge<2, Plane3D, VertexPlane, VertexSE3Expmap> 
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeParallelPlane();
        void computeError()
        {
            const VertexSE3Expmap* poseVertex = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexPlane* planeVertex = static_cast<const VertexPlane*>(_vertices[0]);

            const Plane3D& plane = planeVertex->estimate();
            // measurement function: remap the plane in global coordinates
            Isometry3D w2n = poseVertex->estimate();
            Plane3D localPlane = w2n*plane;

            _error = localPlane.ominus_par(_measurement);
        }

        void setMeasurement(const Plane3D& m){
            _measurement = m;
        }

        bool isDepthPositive(){
            const VertexSE3Expmap* poseVertex = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexPlane* planeVertex = static_cast<const VertexPlane*>(_vertices[0]);

            const Plane3D& plane = planeVertex->estimate();
            // measurement function: remap the plane in global coordinates
            Isometry3D w2n = poseVertex->estimate();
            Plane3D localPlane = w2n*plane;

            return localPlane.distance() > 0;
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;
    };

    class EdgeTwoParPlanes : public BaseBinaryEdge<2, double, VertexPlane, VertexPlane> 
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeTwoParPlanes(){}
        void computeError()
        {
            const VertexPlane* planeVertex2 = static_cast<const VertexPlane*>(_vertices[1]);
            const VertexPlane* planeVertex1 = static_cast<const VertexPlane*>(_vertices[0]);

            Plane3D plane1 = planeVertex1->estimate();
            const Plane3D& plane2 = planeVertex2->estimate();
            // measurement function: remap the plane in global coordinates
            _error = plane1.ominus_par(plane2);
        }

        virtual bool read(std::istream& is) {}
        virtual bool write(std::ostream& os) const{}
    };

    class EdgeCuboidPlane : public BaseBinaryEdge<3, Vector3D, VertexCuboid, VertexPlane>
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeCuboidPlane(){}

        virtual bool read(std::istream& is)
        {
            return true;
        }

        virtual bool write(std::ostream& os) const
        {
            return os.good();
        }

        void computeError_before()
        {
		    const VertexCuboid *cuboidVertex = static_cast<const VertexCuboid *>(_vertices[0]);	//  object pose to world
            const VertexPlane* planeVertex = static_cast<const VertexPlane*>(_vertices[1]); // plane in world

            g2o::cuboid  vCuboid = cuboidVertex->estimate();
            g2o::Plane3D vPlane = planeVertex->estimate();
            Eigen::Vector4d plane_vec = vPlane.toVector();
            // cv::Mat plane_coef = (cv::Mat_<float>(4,1) << plane_vec(0), plane_vec(1), plane_vec(2), plane_vec(3));

            Eigen::VectorXd cuboid_vec = vCuboid.pose.toVector(); // 7d, xyz, qwxyz
            Eigen::Matrix4d cuboid_mat;
            cuboid_mat.setIdentity();
            cuboid_mat.block(0,0,3,3) = Eigen::Quaterniond(cuboid_vec(6),cuboid_vec(3),cuboid_vec(4),cuboid_vec(5)).toRotationMatrix();
            cuboid_mat.col(3).head(3) = cuboid_vec.head(3);
	        Eigen::MatrixXd cuboid_corners = vCuboid.compute3D_BoxCorner();

            for (size_t k = 0; k < 6; k++)
            {
                // n123 = R(theta).transpose()
                float a = cuboid_mat(0, k%3);
                float b = cuboid_mat(1, k%3);
                float c = cuboid_mat(2, k%3);
                float v = cuboid_mat.col(k%3).norm();
                float d = a*cuboid_corners(0,0) + b*cuboid_corners(1,0)+ c*cuboid_corners(2,0);
                if(k >= 3) // use the first or last corner to calculate d
                    d = a*cuboid_corners(0,6) + b*cuboid_corners(1,6)+ c*cuboid_corners(2,6);
                // cv::Mat coef = (cv::Mat_<float>(4,1) << a/v, b/v, c/v, -d/v);
                // if(coef.at<float>(3) < 0)
                //     coef = -coef;
                // std::cout << "cuboidPlaneCoef: " << coef.t() << std::endl;
                // float d = coef.at<float>(3,0) - plane_coef.at<float>(3,0);
                // float angle = coef.at<float>(0,0) * plane_coef.at<float>(0,0) +
                //                 coef.at<float>(1,0) * plane_coef.at<float>(1,0) +
                //                 coef.at<float>(2,0) * plane_coef.at<float>(2,0);

                Eigen::Vector4d coef = Eigen::Vector4d(a/v, b/v, c/v, -d/v);
                if (coef(3)<0)
                    coef = -coef;
                g2o::Plane3D cuboid_coef(coef);

                double dist = coef(3) - plane_vec(3);
                double angle = coef(0)*plane_vec(0) + coef(1)*plane_vec(1) + coef(2)*plane_vec(2);

                if((dist < 0.2 && dist > -0.2) && (angle > 0.9397 || angle < -0.9397)) 
                {
                    // std::cout << "cuboid: " << cuboid_vec.transpose() << std::endl;
                    // std::cout << "plane: " << plane_vec.transpose() << " dist to cuboid: " << d << std::endl;
                    _error = vPlane.ominus(cuboid_coef);
                    return;
                }
            } // loop for k = 6
        } // function compute error

        void showError() // same as computeError(), but print error
        {
		    const VertexCuboid *cuboidVertex = static_cast<const VertexCuboid *>(_vertices[0]);	//  object pose to world
            const VertexPlane* planeVertex = static_cast<const VertexPlane*>(_vertices[1]); // plane in world

            g2o::cuboid  vCuboid = cuboidVertex->estimate();
            g2o::Plane3D vPlane = planeVertex->estimate();
            Eigen::Vector4d plane_vec = vPlane.toVector();
            std::cout << "plane_coef: " << plane_vec.transpose() << std::endl;
            // cv::Mat plane_coef = (cv::Mat_<float>(4,1) << plane_vec(0), plane_vec(1), plane_vec(2), plane_vec(3));

            Eigen::VectorXd cuboid_vec = vCuboid.pose.toVector(); // 7d, xyz, qwxyz
            Eigen::Matrix4d cuboid_mat;
            cuboid_mat.setIdentity();
            cuboid_mat.block(0,0,3,3) = Eigen::Quaterniond(cuboid_vec(6),cuboid_vec(3),cuboid_vec(4),cuboid_vec(5)).toRotationMatrix();
            cuboid_mat.col(3).head(3) = cuboid_vec.head(3);
	        Eigen::MatrixXd cuboid_corners = vCuboid.compute3D_BoxCorner();

            std::vector<Eigen::Vector4d> cuboid_coef;
            for (size_t k = 0; k < 6; k++)
            {
                // n123 = R(theta).transpose()
                float a = cuboid_mat(0, k%3);
                float b = cuboid_mat(1, k%3);
                float c = cuboid_mat(2, k%3);
                float v = cuboid_mat.col(k%3).norm();
                float d = a*cuboid_corners(0,0) + b*cuboid_corners(1,0)+ c*cuboid_corners(2,0);
                if(k >= 3) // use the first or last corner to calculate d
                    d = a*cuboid_corners(0,6) + b*cuboid_corners(1,6)+ c*cuboid_corners(2,6);
                // cv::Mat coef = (cv::Mat_<float>(4,1) << a/v, b/v, c/v, -d/v);
                // if(coef.at<float>(3) < 0)
                //     coef = -coef;
                // std::cout << "cuboidPlaneCoef: " << coef.t() << std::endl;
                // float d = coef.at<float>(3,0) - plane_coef.at<float>(3,0);
                // float angle = coef.at<float>(0,0) * plane_coef.at<float>(0,0) +
                //                 coef.at<float>(1,0) * plane_coef.at<float>(1,0) +
                //                 coef.at<float>(2,0) * plane_coef.at<float>(2,0);

                Eigen::Vector4d coef = Eigen::Vector4d(a/v, b/v, c/v, -d/v);
                if (coef(3)<0)
                    coef = -coef;
                cuboid_coef.push_back(coef);
            }

            for (size_t i = 0; i < cuboid_coef.size(); i++)
            {
                Eigen::Vector4d coef = cuboid_coef[i];

                double dist = coef(3) - plane_vec(3);
                double angle = coef(0)*plane_vec(0) + coef(1)*plane_vec(1) + coef(2)*plane_vec(2);
                std::cout << "cuboid_coef: " << coef.transpose() << " angle: " << angle << " dist to cuboid: " << dist << std::endl;
                if((dist < 0.2 && dist > -0.2) && (angle > 0.9397 || angle < -0.9397)) 
                {
                    g2o::Plane3D cuboid_coef(coef);
                    _error = vPlane.ominus(cuboid_coef);
                    return;
                }
                else
                    _error = Eigen::Vector3d(0,0,0);
            }
            

        } // function compute error

        void setMeasurement(const Vector3D& m){
            _measurement = m;
        }

        void computeError()
        {
            _error = _measurement;
        }
    }; // class edgecuboidplane


}


#endif //ORB_SLAM2_PLANE3D_H
