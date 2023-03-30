#include <Eigen/Dense>
#include <math.h>
#include "structs.h"
#include "utilities.h"
#include "kinematics.h"

namespace dash_init
{
    void Human_Init(Human_params &Human_params, Human_dyn_data &Human_dyn_data);
    void SRB_Init(VectorXd& x0, MatrixXd& q0, MatrixXd& qd0, MatrixXd& lfv0, MatrixXd& lfdv0, VectorXd& u0, SRB_Params srb_params, Human_params human_params);
    void SRB_params_tello(SRB_Params& srb_params);
}