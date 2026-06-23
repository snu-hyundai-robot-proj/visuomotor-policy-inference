#include "utils/admittance.h"

void AdmittanceController::initialize(const Param& p)
{
    setParam(p);
    x_ddot_adm_.setZero();
    x_dot_adm_.setZero();
    x_adm_.setZero();
}

void AdmittanceController::setParam(const Param& p)
{
    p_ = p;
    defineGains();
}

void AdmittanceController::compute(const Eigen::Ref<const Eigen::VectorXd> &x, 
                                   const Eigen::Ref<const Eigen::VectorXd> &x_dot,
                                   const Eigen::Ref<const Eigen::VectorXd> &f_ext
                                   )
{
    Eigen::Vector6d f_select;
    Eigen::Vector6d x_ddot_thrs;
    double x_dot_thrs = 0.03;
    // Eigen::Vector6d x_dot_adm_sat;
    double w = 0.1;
    double dt = p_.dt;

    x_ddot_thrs << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    f_select = f_ext.cwiseProduct(axis_mask_);

    x_ddot_adm_ = inv_mass_.cwiseProduct(f_select - d_.cwiseProduct(x_dot) - p_.stiff.cwiseProduct(x));
    x_ddot_adm_ = noiseGate(x_ddot_adm_, x_ddot_thrs);

    x_dot_adm_ = (1 - w) * x_dot_adm_ + x_ddot_adm_ * dt; // leaky integration
    x_dot_adm_ = noiseGate(x_dot_adm_, 1e-4*Eigen::Vector6d::Ones());
    x_dot_adm_ = saturateTaskVelocity(x_dot_adm_, x_dot_thrs, 0.1); // limit within 30mm/s

    // std::cout<<"x_dot: "<< x_dot_adm_.transpose()<<std::endl;
    // std::cout<<"inv_mass: \n"<< inv_mass_<<std::endl;

    x_adm_ += x_dot_adm_ * dt;

    for(int i = 0; i < 6; i++){
        if(axis_mask_[i] == 0.0)
        {
            x_ddot_adm_[i] = 0.0;
            x_dot_adm_[i] = 0.0;
            x_adm_[i] = 0.0;
        }
    }
}

Eigen::Vector6d AdmittanceController::getAdmittance(const Eigen::Ref<const Eigen::MatrixXd>& R_ref)
{
    // If R_ref = I --> the output is reference to "the sensor frame"
    // R_ref --> rotation of "ft sensor" w.r.t reference frame

    Eigen::Vector6d x_dot_ref;
    x_dot_ref.head<3>() = R_ref*x_dot_adm_.head<3>();
    x_dot_ref.tail<3>() = R_ref*x_dot_adm_.tail<3>();

    debug_amit << x_dot_adm_.head<3>().transpose()<< " " <<x_dot_ref.head<3>().transpose()<<"\n";

    return x_dot_ref;
}

Eigen::Vector6d AdmittanceController::saturateTaskVelocity(Eigen::Vector6d& twist, double v_max, double w_max)
{
    Eigen::Vector6d x_dot;
    Eigen::Vector3d v, w;
    v = twist.head<3>();
    w = twist.tail<3>();

    // --- Translation saturation ---
    double v_norm = v.norm();
    if (v_norm >= v_max)
    {
        double scale = v_max / v_norm;
        v = scale*v;
    }

    // --- Orientation saturation ---
    double w_norm = w.norm();
    if (w_norm > w_max)
    {
        double scale = w_max / w_norm;
        w = scale*w;
    }

    x_dot.head<3>() = v;
    x_dot.tail<3>() = w;

    return x_dot;
}

Eigen::VectorXd AdmittanceController::noiseGate(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    const Eigen::Ref<const Eigen::VectorXd> &thres)
{
    Eigen::ArrayXd mask;
    mask = (x.array().abs() >= thres.array()).cast<double>();    
    return (x.array()*mask).matrix();
}


void AdmittanceController::defineGains()
{
    axis_mask_ = p_.adm_axis;

    inv_mass_.setZero();
    d_.setZero();

    for(int i = 0; i < 6; i++){
        if(axis_mask_[i] == 1.0){
            inv_mass_[i] = 1.0 / p_.mass[i];
            d_[i] = 2.0*p_.zeta[i]*std::sqrt(p_.mass[i]*p_.stiff[i]); // critical damping
        }
    }
}

double AdmittanceController::getDT()
{
    return p_.dt;
}
