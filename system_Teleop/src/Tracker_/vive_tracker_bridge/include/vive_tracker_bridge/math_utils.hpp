#pragma once

#include <array>
#include <cmath>

// 3x3 matrix type
using Mat3 = std::array<std::array<double, 3>, 3>;
using Vec3 = std::array<double, 3>;

// Matrix multiplication: C = A * B
static Mat3 matMul(const Mat3& A, const Mat3& B)
{
    Mat3 C{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) {
                s += A[i][k] * B[k][j];
            }
            C[i][j] = s;
        }
    }
    return C;
}

// Matrix transpose: AT = A^T
static Mat3 matTranspose(const Mat3& A)
{
    Mat3 AT{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            AT[i][j] = A[j][i];
        }
    }
    return AT;
}

// Vector transform: y = A * x
static Vec3 matVecMul(const Mat3& A, const Vec3& x)
{
    Vec3 y{};
    for (int i = 0; i < 3; ++i) {
        y[i] = A[i][0] * x[0] + A[i][1] * x[1] + A[i][2] * x[2];
    }
    return y;
}

// Build rotation matrix for Euler ZYX (intrinsic local axes)
// roll  = phi about X
// pitch = theta about Y
// yaw   = psi about Z
//
// Returns: R_BL = Rz(yaw) * Ry(pitch) * Rx(roll)
// Meaning: v_B = R_BL * v_L   (local vector -> base vector)
static Mat3 eulerZYX_LocalToBase(double roll, double pitch, double yaw)
{
    // Precompute sines/cosines for speed and consistency
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);

    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);

    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    // Rx(roll)
    Mat3 Rx{{
        {{1.0, 0.0, 0.0}},
        {{0.0, cr, -sr}},
        {{0.0, sr,  cr}}
    }};

    // Ry(pitch)
    Mat3 Ry{{
        {{ cp, 0.0, sp}},
        {{0.0, 1.0, 0.0}},
        {{-sp, 0.0, cp}}
    }};

    // Rz(yaw)
    Mat3 Rz{{
        {{cy, -sy, 0.0}},
        {{sy,  cy, 0.0}},
        {{0.0, 0.0, 1.0}}
    }};

    // R_BL = Rz * Ry * Rx
    return matMul(matMul(Rz, Ry), Rx);
}

// Base->Local is inverse of Local->Base for rotation matrices, i.e., transpose
// Returns: R_LB = (R_BL)^T
static Mat3 eulerZYX_BaseToLocal(double roll, double pitch, double yaw)
{
    const Mat3 R_BL = eulerZYX_LocalToBase(roll, pitch, yaw);
    return matTranspose(R_BL);
}

static inline double wrapToPi(double a)
{
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;
}
