#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <cmath>
#include <vector>
#include <tuple>

namespace py = pybind11;

// ============================================================
// Core math utilities
// ============================================================

inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// 4x4 homogeneous transform from DH parameters
void dh_transform(double a, double alpha, double d, double theta, double T[16]) {
    double ct = cos(theta), st = sin(theta);
    double ca = cos(alpha), sa = sin(alpha);
    T[0] = ct;  T[1] = -st*ca; T[2] =  st*sa; T[3] = a*ct;
    T[4] = st;  T[5] =  ct*ca; T[6] = -ct*sa; T[7] = a*st;
    T[8] = 0;   T[9] =  sa;    T[10] = ca;    T[11] = d;
    T[12] = 0;  T[13] = 0;     T[14] = 0;     T[15] = 1;
}

// Multiply two 4x4 matrices: C = A * B
void mat4_mul(const double A[16], const double B[16], double C[16]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i*4 + j] = 0;
            for (int k = 0; k < 4; k++)
                C[i*4 + j] += A[i*4 + k] * B[k*4 + j];
        }
    }
}

// Cross product: c = a x b
void cross3(const double a[3], const double b[3], double c[3]) {
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

double dot3(const double a[3], const double b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void vec3_sub(const double a[3], const double b[3], double c[3]) {
    c[0] = a[0] - b[0]; c[1] = a[1] - b[1]; c[2] = a[2] - b[2];
}

// ============================================================
// Forward kinematics: DH params -> end-effector pose
// ============================================================

// dh_params: Nx4 array (a, alpha, d, theta)
// joint_angles: N array
// Returns: 4x4 homogeneous transform (16 doubles, row-major)
py::array_t<double> forward_kinematics_cpp(
    py::array_t<double> dh_params,
    py::array_t<double> joint_angles)
{
    auto dh = dh_params.unchecked<2>();
    auto q = joint_angles.unchecked<1>();
    int n = dh.shape(0);

    double T[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

    for (int i = 0; i < n; i++) {
        double Ti[16];
        dh_transform(dh(i, 0), dh(i, 1), dh(i, 2), q(i), Ti);
        double T_new[16];
        mat4_mul(T, Ti, T_new);
        for (int j = 0; j < 16; j++) T[j] = T_new[j];
    }

    auto result = py::array_t<double>({4, 4});
    auto r = result.mutable_unchecked<2>();
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            r(i, j) = T[i*4 + j];

    return result;
}

// ============================================================
// Forward kinematics with all transforms (for Jacobian)
// ============================================================

// Returns list of N+1 4x4 transforms (base to each frame + EE)
std::vector<py::array_t<double>> forward_kinematics_all_cpp(
    py::array_t<double> dh_params,
    py::array_t<double> joint_angles)
{
    auto dh = dh_params.unchecked<2>();
    auto q = joint_angles.unchecked<1>();
    int n = dh.shape(0);

    std::vector<py::array_t<double>> transforms;
    double T[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

    // Add base frame
    auto T0 = py::array_t<double>({4, 4});
    auto r0 = T0.mutable_unchecked<2>();
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) r0(i, j) = T[i*4 + j];
    transforms.push_back(T0);

    for (int k = 0; k < n; k++) {
        double Ti[16];
        dh_transform(dh(k, 0), dh(k, 1), dh(k, 2), q(k), Ti);
        double T_new[16];
        mat4_mul(T, Ti, T_new);
        for (int j = 0; j < 16; j++) T[j] = T_new[j];

        auto Tk = py::array_t<double>({4, 4});
        auto rk = Tk.mutable_unchecked<2>();
        for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) rk(i, j) = T[i*4 + j];
        transforms.push_back(Tk);
    }

    return transforms;
}

// ============================================================
// Jacobian: 6xN geometric Jacobian
// ============================================================

py::array_t<double> compute_jacobian_cpp(
    py::array_t<double> dh_params,
    py::array_t<double> joint_angles)
{
    auto dh = dh_params.unchecked<2>();
    auto q = joint_angles.unchecked<1>();
    int n = dh.shape(0);

    // Compute all transforms first
    auto transforms = forward_kinematics_all_cpp(dh_params, joint_angles);
    auto T_ee = transforms.back().unchecked<2>();
    double p_ee[3] = {T_ee(0, 3), T_ee(1, 3), T_ee(2, 3)};

    auto J = py::array_t<double>({6, n});
    auto Jr = J.mutable_unchecked<2>();
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < n; j++)
            Jr(i, j) = 0.0;

    for (int k = 0; k < n; k++) {
        auto T_k = transforms[k].unchecked<2>();
        double z[3] = {T_k(0, 2), T_k(1, 2), T_k(2, 2)};
        double p[3] = {T_k(0, 3), T_k(1, 3), T_k(2, 3)};

        // p_ee - p_k
        double diff[3];
        vec3_sub(p_ee, p, diff);

        // Cross: z x (p_ee - p_k)
        double v[3];
        cross3(z, diff, v);

        Jr(0, k) = v[0];
        Jr(1, k) = v[1];
        Jr(2, k) = v[2];
        Jr(3, k) = z[0];
        Jr(4, k) = z[1];
        Jr(5, k) = z[2];
    }

    return J;
}

// ============================================================
// Full IK solve (DLS) — returns (success, joint_angles, iterations)
// ============================================================

// Simple linear system solver (Gaussian elimination for 6x6)
bool solve_6x6(const double A[36], const double b[6], double x[6]) {
    // Copy A and b for in-place elimination
    double M[6][7];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) M[i][j] = A[i*6 + j];
        M[i][6] = b[i];
    }

    for (int col = 0; col < 6; col++) {
        // Find pivot
        int pivot = col;
        double max_val = fabs(M[col][col]);
        for (int row = col + 1; row < 6; row++) {
            if (fabs(M[row][col]) > max_val) {
                max_val = fabs(M[row][col]);
                pivot = row;
            }
        }
        if (max_val < 1e-15) return false;

        // Swap rows
        if (pivot != col) {
            for (int j = col; j <= 6; j++) {
                double tmp = M[col][j];
                M[col][j] = M[pivot][j];
                M[pivot][j] = tmp;
            }
        }

        // Eliminate
        double piv = M[col][col];
        for (int j = col; j <= 6; j++) M[col][j] /= piv;
        for (int row = 0; row < 6; row++) {
            if (row != col) {
                double factor = M[row][col];
                for (int j = col; j <= 6; j++) M[row][j] -= factor * M[col][j];
            }
        }
    }

    for (int i = 0; i < 6; i++) x[i] = M[i][6];
    return true;
}

// Matrix multiply: C = A (6x6) * B (6x6) -> C (6x6)
void mat6_mul(const double A[36], const double B[36], double C[36]) {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            C[i*6 + j] = 0;
            for (int k = 0; k < 6; k++)
                C[i*6 + j] += A[i*6 + k] * B[k*6 + j];
        }
}

// Matrix transpose multiply: C = A^T (6xN) * B (Nx6) -> 6x6
void matT_mul_6xNx6(const double* A, int N, double C[36]) {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            C[i*6 + j] = 0;
            for (int k = 0; k < N; k++)
                C[i*6 + j] += A[k*6 + i] * A[k*6 + j];  // A^T * A
        }
}

py::tuple ik_solve_cpp(
    py::array_t<double> dh_params,
    py::array_t<double> target_pose,
    py::array_t<double> initial_guess,
    py::array_t<double> joint_limits,  // Nx2: (min, max) per joint
    int max_iterations = 200,
    double position_tolerance = 1e-4,
    double orientation_tolerance = 1e-3,
    double damping = 0.1)
{
    auto dh = dh_params.unchecked<2>();
    auto limits = joint_limits.unchecked<2>();
    int n = dh.shape(0);

    // Initial joint angles
    std::vector<double> q(n);
    auto ig = initial_guess.unchecked<1>();
    for (int i = 0; i < n; i++) q[i] = ig(i);

    // Target
    auto T_tgt = target_pose.unchecked<2>();
    double target_pos[3] = {T_tgt(0, 3), T_tgt(1, 3), T_tgt(2, 3)};
    // Extract target rotation matrix
    double target_rot[9] = {
        T_tgt(0,0), T_tgt(0,1), T_tgt(0,2),
        T_tgt(1,0), T_tgt(1,1), T_tgt(1,2),
        T_tgt(2,0), T_tgt(2,1), T_tgt(2,2),
    };

    std::vector<double> iter_errors;
    int iteration;
    for (iteration = 0; iteration < max_iterations; iteration++) {
        // Current FK
        auto q_arr = py::array_t<double>(n);
        auto q_buf = q_arr.mutable_unchecked<1>();
        for (int i = 0; i < n; i++) q_buf(i) = q[i];

        auto T_cur_arr = forward_kinematics_cpp(dh_params, q_arr);
        auto T_cur = T_cur_arr.unchecked<2>();

        // Position error
        double pos_err[3] = {
            target_pos[0] - T_cur(0, 3),
            target_pos[1] - T_cur(1, 3),
            target_pos[2] - T_cur(2, 3),
        };

        // Orientation error from rotation error matrix R_err = R_target * R_current^T
        double R_cur[9] = {
            T_cur(0,0), T_cur(0,1), T_cur(0,2),
            T_cur(1,0), T_cur(1,1), T_cur(1,2),
            T_cur(2,0), T_cur(2,1), T_cur(2,2),
        };
        // R_err = R_target * R_cur^T (R_cur^T since rotation matrices)
        double R_err[9] = {0};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    R_err[i*3 + j] += target_rot[i*3 + k] * R_cur[j*3 + k];  // R_cur^T

        double trace = R_err[0] + R_err[4] + R_err[8];
        double theta = acos(clamp((trace - 1.0) / 2.0, -1.0, 1.0));
        double orient_err[3] = {0, 0, 0};
        if (fabs(theta) > 1e-10) {
            double s = 2.0 * sin(theta);
            orient_err[0] = theta * (R_err[7] - R_err[5]) / s;
            orient_err[1] = theta * (R_err[2] - R_err[6]) / s;
            orient_err[2] = theta * (R_err[3] - R_err[1]) / s;
        }

        double error[6] = {pos_err[0], pos_err[1], pos_err[2],
                           orient_err[0], orient_err[1], orient_err[2]};
        double err_norm = sqrt(error[0]*error[0] + error[1]*error[1] + error[2]*error[2] +
                               orient_err[0]*orient_err[0] + orient_err[1]*orient_err[1] + orient_err[2]*orient_err[2]);
        iter_errors.push_back(err_norm);

        double pos_norm = sqrt(pos_err[0]*pos_err[0] + pos_err[1]*pos_err[1] + pos_err[2]*pos_err[2]);
        double orient_norm = sqrt(orient_err[0]*orient_err[0] + orient_err[1]*orient_err[1] + orient_err[2]*orient_err[2]);

        if (pos_norm < position_tolerance && orient_norm < orientation_tolerance)
            break;

        // Jacobian
        auto J_arr = compute_jacobian_cpp(dh_params, q_arr);
        auto J = J_arr.unchecked<2>();

        // Damped least-squares: delta_q = J^T (J J^T + lambda^2 I)^-1 error
        double JJT[36] = {0};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++) {
                JJT[i*6 + j] = 0;
                for (int k = 0; k < n; k++)
                    JJT[i*6 + j] += J(i, k) * J(j, k);
            }

        double lam2 = damping * damping;
        double damped[36];
        for (int i = 0; i < 36; i++) damped[i] = JJT[i];
        for (int i = 0; i < 6; i++) damped[i*6 + i] += lam2;

        double delta_q[6];
        if (!solve_6x6(damped, error, delta_q)) {
            // Fallback: use larger damping
            for (int i = 0; i < 36; i++) damped[i] = JJT[i];
            lam2 = 1.0;
            for (int i = 0; i < 6; i++) damped[i*6 + i] += lam2;
            if (!solve_6x6(damped, error, delta_q))
                break;
        }

        // Update: q += J^T @ delta_q
        for (int i = 0; i < n; i++) {
            double dq = 0;
            for (int j = 0; j < 6; j++)
                dq += J(j, i) * delta_q[j];
            q[i] += dq;
            // Clamp to joint limits
            q[i] = clamp(q[i], limits(i, 0), limits(i, 1));
        }
    }

    // Build return arrays
    auto q_out = py::array_t<double>(n);
    auto qo = q_out.mutable_unchecked<1>();
    for (int i = 0; i < n; i++) qo(i) = q[i];

    auto err_out = py::array_t<double>(iter_errors.size());
    auto eo = err_out.mutable_unchecked<1>();
    for (size_t i = 0; i < iter_errors.size(); i++) eo(i) = iter_errors[i];

    bool success = (iteration < max_iterations);
    return py::make_tuple(success, q_out, iteration + 1, err_out);
}


PYBIND11_MODULE(ik_fast, m) {
    m.doc() = "Fast 6-DOF IK solver in C++ (pybind11)";
    m.def("forward_kinematics", &forward_kinematics_cpp, "Forward kinematics from DH params");
    m.def("compute_jacobian", &compute_jacobian_cpp, "Geometric Jacobian");
    m.def("ik_solve", &ik_solve_cpp, "Damped least-squares IK solver",
          py::arg("dh_params"), py::arg("target_pose"), py::arg("initial_guess"),
          py::arg("joint_limits"), py::arg("max_iterations") = 200,
          py::arg("position_tolerance") = 1e-4, py::arg("orientation_tolerance") = 1e-3,
          py::arg("damping") = 0.1);
}
