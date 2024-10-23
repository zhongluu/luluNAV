#ifndef _BASEELEMENT_
#define _BASEELEMENT_

#include <Eigen/Dense>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Matrix;

namespace navGlv{
    extern const double Re;
    extern const double f;
    extern const double wie;
    extern const double g;
    extern const double deg2rad;
    extern const double rad2deg;
    extern const double navPI;
    extern const double dph;
    extern const double lightspeed;
    extern const double e;
    extern const double e2;
    extern const double Rp;
}

class Earth
{
public:
    double a, b;
    double f, e, e2;
    double wie;
    double sl, sl2, cl, tl, RMh, RNh, clRNh, f_RMh, f_RNh, f_clRNh;
    Vector3d dpos;
    Vector3d pos, vn, wnie, wnen, wnin, gn, gcc;
    Matrix3d Mpv;

public:
    Earth(double a0 = navGlv::Re, double f0 = navGlv::f);
    ~Earth();
    bool update(Vector3d &inPos, Vector3d &inVn);
    bool vnUpdateDpos(Vector3d &vn);
};

Matrix3d vector2skew(const Vector3d &v);

#endif