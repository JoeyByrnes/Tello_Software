#include <Eigen/Dense>
#include <math.h>
#include "structs.h"
#include "utilities.h"


namespace  dash_dyn
{
    void SRB_Dyn(VectorXd& x_next, VectorXd& net_external_wrench, SRB_Params srb_params, VectorXd x, MatrixXd lfv, VectorXd u, VectorXd tau_ext);
    void HLIP_dyn_xk2x0(double& x0, double& dx0, VectorXd xk, double w, double Ts);
    void HLIP_S2S_Dyn(MatrixXd& A, VectorXd& B, double T_SSP, double T_DSP, double w);
    void HLIP_SSP_dyn(double& x, double& dx, double t, double w, double x0, double dx0);
    void HLIP_DSP_dyn(double& x, double& dx, double t, double x0, double dx0);
    void HLIP_Reset_Map_SSP_DSP(Vector2d& x_plus, Vector2d x_minus);
    void HLIP_Reset_Map_DSP_SSP(Vector2d& x_plus, Vector2d x_minus, double u);
}
