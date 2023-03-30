#include <Eigen/Dense>
#include <math.h>
#include "structs.h"
#include "utilities.h"


namespace  dash_dyn
{
    void SRB_Dyn(VectorXd& x_next, VectorXd& net_external_wrench, SRB_Params srb_params, VectorXd x, MatrixXd lfv, VectorXd u, VectorXd tau_ext);
}