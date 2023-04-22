#include "mujoco_utilities.h"

// MuJoCo data structures
extern mjModel* m;                  // MuJoCo model
extern mjData* d;                   // MuJoCo data
extern mjvCamera cam;               // abstract camera
extern mjvOption opt;               // visualization options
extern mjvScene scn;                // abstract scene
extern mjrContext con;              // custom GPU context

extern double push_force_x;
extern double push_force_y;
extern double push_force_z;
extern bool pause_sim;

double rfz;
double rbz;
double lfz;
double lbz;

int rf_cnt = 0;
int rb_cnt = 0;
int lf_cnt = 0;
int lb_cnt = 0;

bool rf_seen = false;
bool rb_seen = false;
bool lf_seen = false;
bool lb_seen = false;


extern VectorXd gnd_contacts;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

extern int hip_motor1_r_idx ;
extern int hip_motor1_l_idx ;
extern int hip_motor2_r_idx ;
extern int hip_motor2_l_idx ;
extern int hip_motor3_r_idx ;
extern int hip_motor3_l_idx ;
extern int knee_motor_r_idx ;
extern int knee_motor_l_idx ;
extern int ankle_motor_r_idx;
extern int ankle_motor_l_idx;

// Callbacks

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        // mj_resetData(m, d);
        // sim_was_restarted = true;
        // mj_forward(m, d);
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_X)
    {
        push_force_x = 10;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_Y)
    {
        push_force_y = 10;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_Z)
    {
        push_force_z = 10;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_C)
    {
        mju_zero(d->qfrc_applied, m->nv);
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_ENTER)
    {
        pause_sim = !pause_sim;
    }
}

void window_close_callback(GLFWwindow* window)
{
    glfwSetWindowShouldClose(window, GLFW_TRUE);
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void contactforce(const mjModel* m, mjData* d,int FSM)
{
    mjMARKSTACK
    mjtNum* f0_contact_pos = mj_stackAlloc(d, 3);//contact position in unkown frame. check
    mjtNum* f0_contact_frame = mj_stackAlloc(d, 9);
    mjtNum* f0_contact_force = mj_stackAlloc(d, 6);//contact force in contact frame

    bool left_front_contact_detected = false;
    bool left_back_contact_detected = false;
    bool right_front_contact_detected = false;
    bool right_back_contact_detected = false;
    gnd_contacts.setZero();
    rf_seen = false;
    rb_seen = false;
    lf_seen = false;
    lb_seen = false;
    bool right_data_reliable;
    bool left_data_reliable;
    for (int i = 0; i < d->ncon; i++)
    {
        mjContact* cur_contact = &((d->contact)[i]);
        std::string geom1 = mj_id2name(m, mjOBJ_GEOM, cur_contact->geom1);
        std::string geom2 = mj_id2name(m, mjOBJ_GEOM, cur_contact->geom2);
        //std::cout << "contact point # " << i + 1 << std::endl;
        //std::cout << mj_id2name(m, mjOBJ_GEOM, cur_contact->geom2) << "  :"; // normal
        int ll = 0;
        int ul = 1000;
        right_data_reliable = (rfz >= ll && rbz >= ll) && (rfz <= ul && rbz <= ul);
        left_data_reliable = (lfz >= ll && lbz >= ll) && (lfz <= ul && lbz <= ul);
        if (geom1.compare("floor") == 0 || geom2.compare("floor") == 0)
        {
            if((geom1.compare("right_foot_toe") == 0 || geom2.compare("right_foot_toe") == 0) && right_data_reliable  )
            {
                gnd_contacts(0) = 1;
                rf_seen = true;
            }
            if((geom1.compare("right_foot_heel") == 0 || geom2.compare("right_foot_heel") == 0) && right_data_reliable  )
            {
                gnd_contacts(1) = 1;
                rb_seen = true;
            }
            if((geom1.compare("left_foot_toe") == 0 || geom2.compare("left_foot_toe") == 0) && left_data_reliable  )
            {
                gnd_contacts(2) = 1;
                lf_seen = true;
            }
            if((geom1.compare("left_foot_heel") == 0 || geom2.compare("left_foot_heel") == 0) && left_data_reliable  )
            {
                gnd_contacts(3) = 1;
                lb_seen = true;
            }
            //get robot's geom id
            int bodyid;
            if(geom1.compare("floor") == 0){ //return 0 if same string
                bodyid = cur_contact->geom1;
            }else{
                bodyid = cur_contact->geom2;
            }
            //std::cout << "Collision Point = " << mj_id2name(m, mjOBJ_GEOM, bodyid) << '\n';
            // calculate position/force/normal of contact points.
            mju_copy(f0_contact_pos, cur_contact->pos, 3);
            mju_copy(f0_contact_frame, cur_contact->frame, 9); //contact frame in world frame
            mj_contactForce(m, d, i, f0_contact_force); // torque in contact frame, stored in transposed form
            //Print contact frame
            // std::cout << "Contact Frame = " << '\n';
            // mju_printMat(f0_contact_frame, 3, 3);
            //Print contact force
            mjtNum fw_contact_force[9];
            // std::cout << "Contact Force in Contact Frame = " << '\n';
            mju_transpose(fw_contact_force, f0_contact_force, 3, 3); //
            // mju_printMat(fw_contact_force, 3, 3);
            //std::cout << "Contact Force in World Frame = " << '\n';
            mjtNum f_world[9];
            mju_mulMatMat(f_world, f0_contact_frame, fw_contact_force, 3, 3, 3);
            // mju_printMat(f_world, 3, 3);
            //std::cout << f_world[0] << ", " << f_world[3] << ", " << f_world[6] << endl;

        } // if one geom is object
    } // for i = 1:ncon

    if(!rf_seen) gnd_contacts(1) = 0;
    if(!rb_seen) gnd_contacts(0) = 0;
    if(!lf_seen) gnd_contacts(3) = 0;
    if(!lb_seen) gnd_contacts(2) = 0;

    if(FSM == 1)
    {
        gnd_contacts(0) = 0;
        gnd_contacts(1) = 0;
    }
    if(FSM == -1)
    {
        gnd_contacts(2) = 0;
        gnd_contacts(3) = 0;
    }

    // if(rf_cnt < 5) gnd_contacts(0) = 1;
    // if(rb_cnt < 5) gnd_contacts(1) = 1;
    // if(lf_cnt < 5) gnd_contacts(2) = 1;
    // if(lb_cnt < 5) gnd_contacts(3) = 1;
    // cout << rfz << ", \t" << rbz << ", \t" << lfz << ", \t" << lbz << endl;
    // cout << (int)right_data_reliable << ",\t" << (int)left_data_reliable << endl;


    mjFREESTACK
}

void applyJointTorquesMujoco(VectorXd torques)
{
    d->ctrl[hip_motor1_l_idx]  = torques(0);
    d->ctrl[hip_motor2_l_idx]  = torques(1);
    d->ctrl[hip_motor3_l_idx]  = torques(2);
    d->ctrl[knee_motor_l_idx]  = torques(3);
    d->ctrl[ankle_motor_l_idx] = torques(4);
    d->ctrl[hip_motor1_r_idx]  = torques(5);
    d->ctrl[hip_motor2_r_idx]  = torques(6);
    d->ctrl[hip_motor3_r_idx]  = torques(7);
    d->ctrl[knee_motor_r_idx]  = torques(8);
    d->ctrl[ankle_motor_r_idx] = torques(9);
}

double sigmoid(double x) { return 1 / (1 + exp(-x)); }

VectorXd mux_and_smooth(VectorXd initialOutput, VectorXd finalOutput, double start_time, double end_time, double time) 
{

    if( time < start_time ) return initialOutput;
    if( time > end_time ) return finalOutput;
    if(end_time - start_time <= 0) return finalOutput;

    // compute the sigmoid function input for each element in the vector
    double switchFactor = (time - start_time) / (end_time - start_time);  // Calculate the switch factor based on the current time step and direction flag

    if(switchFactor > 1) switchFactor = 1;

    double smoothVal = sigmoid((switchFactor - 0.5) * 14);  // Scale the switch factor to be between -5 and 5, and apply sigmoid function
    if(smoothVal < 0.001) smoothVal = 0;
    if(smoothVal > 0.999) smoothVal = 1;

    VectorXd out = ((1-smoothVal) * initialOutput) + (smoothVal * finalOutput);

    return out;
}