#include <nova_processing/nv_math.hpp>
#include <nova_processing/nv_types.hpp>

void nvNormalizeVector(double v[3]) {
    double norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    for (int i = 0; i < 3; ++i) v[i] /= norm;
}

void nvProjectOut(double target[3], const double ref[3]) {
    double dot = target[0]*ref[0] + target[1]*ref[1] + target[2]*ref[2];
    for (int i = 0; i < 3; ++i) target[i] -= dot * ref[i];
}

void nvOrthonormalizeMatrix(nv_mat<float64_t, 3, 3>& R) {

    double r1[3] = { R[0][0], R[1][0], R[2][0] };
    double r2[3] = { R[0][1], R[1][1], R[2][1] };
    
    nvNormalizeVector(r1);
    
    nvProjectOut(r2, r1);
    nvNormalizeVector(r2);
    
    double r3[3];
    // r3 = r1 x r2
    r3[0] = r1[1]*r2[2] - r1[2]*r2[1];
    r3[1] = r1[2]*r2[0] - r1[0]*r2[2];
    r3[2] = r1[0]*r2[1] - r1[1]*r2[0];
    nvNormalizeVector(r3);
    
    // Write back into R
    for (int i = 0; i < 3; ++i) {
        R[i][0] = r1[i];
        R[i][1] = r2[i];
        R[i][2] = r3[i];
    }
}

nv_vec<float64_t, 3> nvGetOrientationFromRotMat(nv_mat<float64_t, 3, 3> &R){
    nv_vec<float64_t, 3> ret;
    ret[0] = atan2(R[2][1], R[2][2]);
    if(R[2][0]<=-1)ret[1] = -asin(-1);
    else if(R[2][0]>=1)ret[1] = -asin(1);
    else ret[1] = -asin(R[2][0]);
    ret[2] = atan2(R[1][0], R[0][0]);
    return ret;
}
