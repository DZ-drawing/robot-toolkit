#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <cmath>
#include <vector>
#include <tuple>

namespace py = pybind11;

// ============================================================
// Math utilities
// ============================================================

inline void cross3(const double a[3], const double b[3], double c[3]) {
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

inline double dot3(const double a[3], const double b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// 4x4 DH transform
void dh_transform(double a, double alpha, double d, double theta, double T[16]) {
    double ct = cos(theta), st = sin(theta);
    double ca = cos(alpha), sa = sin(alpha);
    T[0] = ct;  T[1] = -st*ca; T[2] =  st*sa; T[3] = a*ct;
    T[4] = st;  T[5] =  ct*ca; T[6] = -ct*sa; T[7] = a*st;
    T[8] = 0;   T[9] =  sa;    T[10] = ca;    T[11] = d;
    T[12] = 0;  T[13] = 0;     T[14] = 0;     T[15] = 1;
}

void mat4_mul(const double A[16], const double B[16], double C[16]) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            C[i*4+j] = 0;
            for (int k = 0; k < 4; k++)
                C[i*4+j] += A[i*4+k] * B[k*4+j];
        }
}

// Multiply 3x3 rotation stored in 4x4 row-major (stride 4) by 3-vector
void mat3_mul_vec3(const double* R4x4, const double v[3], double out[3]) {
    out[0] = R4x4[0]*v[0] + R4x4[1]*v[1] + R4x4[2]*v[2];
    out[1] = R4x4[4]*v[0] + R4x4[5]*v[1] + R4x4[6]*v[2];
    out[2] = R4x4[8]*v[0] + R4x4[9]*v[1] + R4x4[10]*v[2];
}

void mat3T_mul_vec3(const double* R4x4, const double v[3], double out[3]) {
    out[0] = R4x4[0]*v[0] + R4x4[4]*v[1] + R4x4[8]*v[2];
    out[1] = R4x4[1]*v[0] + R4x4[5]*v[1] + R4x4[9]*v[2];
    out[2] = R4x4[2]*v[0] + R4x4[6]*v[1] + R4x4[10]*v[2];
}

// ============================================================
// Forward kinematics: return list of 4x4 transforms
// ============================================================

std::vector<std::vector<double>> forward_kinematics_all_cpp(
    py::array_t<double> dh_params,  // Nx4: (a, alpha, d, theta_offset=0)
    py::array_t<double> q)          // N joint angles
{
    auto dh = dh_params.unchecked<2>();
    auto q_arr = q.unchecked<1>();
    int n = dh.shape(0);

    std::vector<std::vector<double>> Ts(n + 1);
    double T[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

    // Store base transform
    Ts[0] = std::vector<double>(T, T + 16);

    for (int i = 0; i < n; i++) {
        double Ti[16];
        dh_transform(dh(i,0), dh(i,1), dh(i,2), dh(i,3) + q_arr(i), Ti);
        double T_new[16];
        mat4_mul(T, Ti, T_new);
        for (int j = 0; j < 16; j++) T[j] = T_new[j];
        Ts[i+1] = std::vector<double>(T, T + 16);
    }

    return Ts;
}

// ============================================================
// Inverse dynamics (RNEA) — verified base-frame algorithm
// ============================================================

py::array_t<double> inverse_dynamics_cpp(
    py::array_t<double> dh_params,     // Nx4: (a, alpha, d, 0)
    py::array_t<double> q,             // N
    py::array_t<double> qd,            // N
    py::array_t<double> qdd,           // N
    py::array_t<double> masses,        // N
    py::array_t<double> coms,          // Nx3: COM in link frame
    py::array_t<double> inertias,      // Nx9: row-major 3x3 inertia tensors
    py::array_t<double> damping,       // N
    py::array_t<double> gravity)       // 3
{
    auto dh = dh_params.unchecked<2>();
    auto q_arr = q.unchecked<1>();
    auto qd_arr = qd.unchecked<1>();
    auto qdd_arr = qdd.unchecked<1>();
    auto m_arr = masses.unchecked<1>();
    auto com_arr = coms.unchecked<2>();
    auto I_arr = inertias.unchecked<2>();
    auto damp_arr = damping.unchecked<1>();
    auto g_arr = gravity.unchecked<1>();
    int n = dh.shape(0);

    // === Forward kinematics ===
    std::vector<std::vector<double>> Ts(n + 1);
    double T[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    Ts[0] = std::vector<double>(T, T + 16);

    for (int i = 0; i < n; i++) {
        double Ti[16], T_new[16];
        dh_transform(dh(i,0), dh(i,1), dh(i,2), dh(i,3) + q_arr(i), Ti);
        mat4_mul(T, Ti, T_new);
        for (int j = 0; j < 16; j++) T[j] = T_new[j];
        Ts[i+1] = std::vector<double>(T, T + 16);
    }

    // === Forward recursion (base frame) ===
    std::vector<double> omega[7], alpha[7], a_origin[7], a_com[7];
    for (int i = 0; i <= n; i++) {
        omega[i] = {0,0,0};
        alpha[i] = {0,0,0};
        a_origin[i] = {0,0,0};
        a_com[i] = {0,0,0};
    }

    // Base acceleration = -gravity
    double g[3] = {g_arr(0), g_arr(1), g_arr(2)};
    a_origin[0] = {-g[0], -g[1], -g[2]};

    double z[3] = {0, 0, 1};

    for (int i = 0; i < n; i++) {
        double* R_i = Ts[i+1].data();  // rotation matrix (first 9 elements)

        // Joint axis in base frame
        double z_i[3];
        mat3_mul_vec3(R_i, z, z_i);

        // Angular velocity
        omega[i+1][0] = omega[i][0] + qd_arr(i) * z_i[0];
        omega[i+1][1] = omega[i][1] + qd_arr(i) * z_i[1];
        omega[i+1][2] = omega[i][2] + qd_arr(i) * z_i[2];

        // Angular acceleration
        double cross_w_qd_z[3];
        cross3(omega[i].data(), z_i, cross_w_qd_z);
        alpha[i+1][0] = alpha[i][0] + qdd_arr(i) * z_i[0] + qd_arr(i) * cross_w_qd_z[0];
        alpha[i+1][1] = alpha[i][1] + qdd_arr(i) * z_i[1] + qd_arr(i) * cross_w_qd_z[1];
        alpha[i+1][2] = alpha[i][2] + qdd_arr(i) * z_i[2] + qd_arr(i) * cross_w_qd_z[2];

        // r_i = origin_i+1 - origin_i
        double r_i[3] = {Ts[i+1][3] - Ts[i][3],
                         Ts[i+1][7] - Ts[i][7],
                         Ts[i+1][11] - Ts[i][11]};

        // Linear acceleration of origin (use alpha[i+1] and omega[i+1])
        double cross_a_r[3], cross_w_cross_w_r[3], cross_w_r[3];
        cross3(alpha[i+1].data(), r_i, cross_a_r);
        cross3(omega[i+1].data(), r_i, cross_w_r);
        cross3(omega[i+1].data(), cross_w_r, cross_w_cross_w_r);
        a_origin[i+1][0] = a_origin[i][0] + cross_a_r[0] + cross_w_cross_w_r[0];
        a_origin[i+1][1] = a_origin[i][1] + cross_a_r[1] + cross_w_cross_w_r[1];
        a_origin[i+1][2] = a_origin[i][2] + cross_a_r[2] + cross_w_cross_w_r[2];

        // COM acceleration
        double com_link[3] = {com_arr(i,0), com_arr(i,1), com_arr(i,2)};
        double com_b[3];
        mat3_mul_vec3(R_i, com_link, com_b);
        double r_com[3] = {com_b[0] - Ts[i][3],
                           com_b[1] - Ts[i][7],
                           com_b[2] - Ts[i][11]};

        double cross_a_rc[3], cross_w_rc[3], cross_w_w_rc[3];
        cross3(alpha[i+1].data(), r_com, cross_a_rc);
        cross3(omega[i+1].data(), r_com, cross_w_rc);
        cross3(omega[i+1].data(), cross_w_rc, cross_w_w_rc);
        a_com[i+1][0] = a_origin[i][0] + cross_a_rc[0] + cross_w_w_rc[0];
        a_com[i+1][1] = a_origin[i][1] + cross_a_rc[1] + cross_w_w_rc[1];
        a_com[i+1][2] = a_origin[i][2] + cross_a_rc[2] + cross_w_w_rc[2];
    }

    // === Backward recursion ===
    double f_next[3] = {0,0,0}, n_next[3] = {0,0,0};
    auto tau = py::array_t<double>(n);
    auto tau_arr = tau.mutable_unchecked<1>();

    for (int i = n - 1; i >= 0; i--) {
        double* R_i = Ts[i+1].data();
        double m_i = m_arr(i);

        // Inertia in base frame: I_b = R * I_local * R^T
        double I_local[9] = {I_arr(i,0), I_arr(i,1), I_arr(i,2),
                             I_arr(i,3), I_arr(i,4), I_arr(i,5),
                             I_arr(i,6), I_arr(i,7), I_arr(i,8)};
        // Extract rotation as proper 3x3 (stride 4)
        double R3x3[9] = {R_i[0], R_i[1], R_i[2],
                          R_i[4], R_i[5], R_i[6],
                          R_i[8], R_i[9], R_i[10]};
        // R3x3 * I_local (both proper 3x3)
        double RI[9];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) {
                RI[r*3+c] = 0;
                for (int k = 0; k < 3; k++)
                    RI[r*3+c] += R3x3[r*3+k] * I_local[k*3+c];
            }
        // RI * R3x3^T
        double I_b[9] = {0};
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                for (int k = 0; k < 3; k++)
                    I_b[r*3+c] += RI[r*3+k] * R3x3[c*3+k];

        // Inertial force and torque
        double F_i[3] = {m_i * a_com[i+1][0], m_i * a_com[i+1][1], m_i * a_com[i+1][2]};

        // Proper 3x3 matrix-vector multiply for inertia tensor
        auto mat3x3_mul_vec3 = [](const double M[9], const double v[3], double out[3]) {
            out[0] = M[0]*v[0] + M[1]*v[1] + M[2]*v[2];
            out[1] = M[3]*v[0] + M[4]*v[1] + M[5]*v[2];
            out[2] = M[6]*v[0] + M[7]*v[1] + M[8]*v[2];
        };
        double I_w[3], N_i[3];
        mat3x3_mul_vec3(I_b, omega[i+1].data(), I_w);
        cross3(omega[i+1].data(), I_w, N_i);
        mat3x3_mul_vec3(I_b, alpha[i+1].data(), I_w);  // reuse I_w as temp
        N_i[0] += I_w[0]; N_i[1] += I_w[1]; N_i[2] += I_w[2];

        // COM in base frame
        double com_link[3] = {com_arr(i,0), com_arr(i,1), com_arr(i,2)};
        double com_b[3];
        mat3_mul_vec3(R_i, com_link, com_b);
        double r_origin_to_com[3] = {com_b[0] - Ts[i][3],
                                      com_b[1] - Ts[i][7],
                                      com_b[2] - Ts[i][11]};

        // Force/moment balance
        double cross_r_F[3], f_i[3], n_i[3];
        cross3(r_origin_to_com, F_i, cross_r_F);
        f_i[0] = f_next[0] + F_i[0];
        f_i[1] = f_next[1] + F_i[1];
        f_i[2] = f_next[2] + F_i[2];
        n_i[0] = n_next[0] + cross_r_F[0] + N_i[0];
        n_i[1] = n_next[1] + cross_r_F[1] + N_i[1];
        n_i[2] = n_next[2] + cross_r_F[2] + N_i[2];

        if (i < n - 1) {
            double r_child[3] = {Ts[i+2][3] - Ts[i][3], Ts[i+2][7] - Ts[i][7], Ts[i+2][11] - Ts[i][11]};
            double cross_rc_f[3];
            cross3(r_child, f_next, cross_rc_f);
            n_i[0] += cross_rc_f[0];
            n_i[1] += cross_rc_f[1];
            n_i[2] += cross_rc_f[2];
        }

        f_next[0] = f_i[0]; f_next[1] = f_i[1]; f_next[2] = f_i[2];
        n_next[0] = n_i[0]; n_next[1] = n_i[1]; n_next[2] = n_i[2];

        // Joint torque: projection onto z_i
        double z_i[3];
        mat3_mul_vec3(R_i, z, z_i);
        tau_arr(i) = dot3(n_i, z_i) + damp_arr(i) * qd_arr(i);
    }

    return tau;
}


PYBIND11_MODULE(robot_dyn_fast, m) {
    m.doc() = "Fast rigid body dynamics in C++ (pybind11)";
    m.def("inverse_dynamics", &inverse_dynamics_cpp, "Recursive Newton-Euler inverse dynamics");
}
