/*
LULUNAV
Copyright (C) {{ 2024 }}  {{ yulu_zhong }}

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "BASEElement.h"
#include <cmath>

namespace navGlv
{
    const double Re = 6378137.0;
    const double f = (1.0 / 298.257);
    const double wie = 7.2921151467e-5;
    const double g = 9.7803267714;
    const double navPI = 3.14159265358979;
    const double deg2rad = navPI / 180.0;   
    const double rad2deg = 180.0 / navPI; 
    const double dph = deg2rad/3600; 
    const double lightspeed = 299792458.0;
    const double Rp = (1 - f) * Re;
    const double e = std::sqrt(2 * f - (f * f));
    const double e2 = e * e;
    // const double g = 9.7803267714e-06;
} // namespacavGlv

Earth::Earth(double a0, double f0): a(a0), f(f0) 
{
    b = (1 - f) * a;
    e = std::sqrt(a * a - b * b) / a;
    e2 = e * e;
    wie = navGlv::wie;
    gn.setZero();
    vn.setZero();
    Mpv.setZero();
    Mpv(2,2) = 1.0;
    update(gn, vn);
}

Earth::~Earth()
{
}

bool Earth::vnUpdateDpos(Vector3d &vn) 
{
    dpos[0] = 0.5 * vn[1] * f_RMh;
    dpos[1] = 0.5 * vn[0] * f_clRNh;
    dpos[2] = 0.5 * vn[2];
    return true;
}

bool Earth::update(Vector3d &inPos, Vector3d &inVn)
{
    pos = inPos;
    vn = inVn;
    sl = std::sin(pos[0]); 
    cl = std::cos(pos[0]); 
    tl = sl / cl;
    double sq = 1 - e2 * sl * sl;
    double sq2 = std::sqrt(sq);
    RMh = a * (1 - e2) / sq / sq2 + pos[2];
    f_RMh = 1.0 / RMh;
    RNh = a / sq2 + pos[2];
    clRNh = cl * RNh; f_RNh = 1.0 / RNh; f_clRNh = 1.0 / clRNh;
    wnie[0] = 0; wnie[1] = wie * cl; wnie[2] = wie * sl;
    wnen[0] = -vn[1] * f_RMh; wnen[1] = vn[0] * f_RNh; wnen[2] = wnen[1] * tl;
    wnin = wnie + wnen;
    sl2 = sl * sl;
    double sl4 = sl2 * sl2;
    Mpv(0,1) = f_RMh; Mpv(1,0) = f_clRNh; 
    gn[2] = -(navGlv::g * (1 + 5.27094e-3 * sl2 + 2.32718e-5 * sl4)- 3.086e-6 * pos[2]);
    gcc = gn + vn.cross(wnie + wnin);
    return true;
}

Matrix3d vector2skew(const Vector3d &v)
{
    Matrix3d mat;
    mat << 0,      -v.z(),  v.y(),
           v.z(),    0,    -v.x(),
           -v.y(),  v.x(),  0;
    return mat;
}
