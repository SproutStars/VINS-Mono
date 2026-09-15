#include "pose_local_parameterization.h"

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}
bool PoseLocalParameterization::PlusJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

bool PoseLocalParameterization::Minus(const double *y, const double *x, double *y_minus_x) const
{
    Eigen::Map<const Eigen::Vector3d> py(y);
    Eigen::Map<const Eigen::Vector3d> px(x);
    Eigen::Quaterniond qy(Eigen::Map<const Eigen::Vector4d>(y + 3));
    Eigen::Quaterniond qx(Eigen::Map<const Eigen::Vector4d>(x + 3));

    Eigen::Map<Eigen::Vector3d> dp(y_minus_x);
    Eigen::Map<Eigen::Vector3d> dq(y_minus_x + 3);

    dp = py - px;

    // Plus() applies the increment on the right of the quaternion, so the
    // inverse operation is the relative rotation expressed in the local frame.
    Eigen::Quaterniond dq_quat = qx.conjugate() * qy;
    dq_quat.normalize();
    if (dq_quat.w() < 0.0)
    {
        dq_quat.coeffs() = -dq_quat.coeffs();
    }
    dq = 2.0 * dq_quat.vec() / dq_quat.w();

    return true;
}

bool PoseLocalParameterization::MinusJacobian(const double *x, double *jacobian) const
{
    const Eigen::Quaterniond qx(Eigen::Map<const Eigen::Vector4d>(x + 3));
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);

    j.setZero();
    j.block<3, 3>(0, 0).setIdentity();

    // Derivative of Minus() with respect to the ambient quaternion at y == x.
    Eigen::Matrix3d skew;
    skew <<             0.0, -qx.z(),  qx.y(),
               qx.z(),            0.0, -qx.x(),
              -qx.y(),  qx.x(),            0.0;
    j.block<3, 3>(3, 3) = 2.0 * (qx.w() * Eigen::Matrix3d::Identity() - skew);
    j.block<3, 1>(3, 6) = -2.0 * qx.vec();

    return true;
}
