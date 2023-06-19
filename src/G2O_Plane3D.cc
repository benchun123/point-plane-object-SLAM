
#include "G2O_Plane3D.h"

namespace g2o
{
    VertexPlane::VertexPlane(){
    }

    bool VertexPlane::read(std::istream& is) {
        Vector4D lv;
        for (int i=0; i<4; i++)
            is >> lv[i];
        setEstimate(Plane3D(lv));
        return true;
    }

    bool VertexPlane::write(std::ostream& os) const {
        Vector4D lv=_estimate.toVector();
        for (int i=0; i<4; i++){
            os << lv[i] << " ";
        }
        return os.good();
    }


    EdgePlane::EdgePlane(){

    }

    bool EdgePlane::read(std::istream &is) {
        Vector4D v;
        is >> v(0) >> v(1) >> v(2) >> v(3);
        setMeasurement(Plane3D(v));
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgePlane::write(std::ostream &os) const {
        Vector4D v = _measurement.toVector();
        os << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " ";
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
                os << " " << information()(i, j);
        return os.good();
    }


    EdgeVerticalPlane::EdgeVerticalPlane(){

    }

    bool EdgeVerticalPlane::read(std::istream &is) {
        Vector4D v;
        is >> v(0) >> v(1) >> v(2) >> v(3);
        setMeasurement(Plane3D(v));
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeVerticalPlane::write(std::ostream &os) const {
        Vector4D v = _measurement.toVector();
        os << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " ";
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
                os << " " << information()(i, j);
        return os.good();
    }


    EdgeParallelPlane::EdgeParallelPlane(){

    }

    bool EdgeParallelPlane::read(std::istream &is) {
        Vector4D v;
        is >> v(0) >> v(1) >> v(2) >> v(3);
        setMeasurement(Plane3D(v));
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeParallelPlane::write(std::ostream &os) const {
        Vector4D v = _measurement.toVector();
        os << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " ";
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
                os << " " << information()(i, j);
        return os.good();
    }
}