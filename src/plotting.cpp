#include "plotting.h"

namespace plt = matplotlibcpp;

void* plotting( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    std::vector<double> x, y1,y2,y3,y4,y5,y6,y7,y8,y9,y10,y11,y12;
    while(tello->controller->get_time() < 0.01){ usleep(1000); }

    int plotting_history = 50;
    double last_plot_time = 0;

    // Initialize the plot
    plt::backend("TkAgg");
    plt::figure_size(1200, 800);
    plt::rcparams({{"font.size", "20"}});
    plt::rcparams({{"lines.linewidth", "2"}});
    plt::subplots_adjust({{"wspace", 0.5}, {"hspace", 0.5}});
    plt::ion();
    plt::plot(x, y1);
    // Enable legend.
    plt::legend();

    plt::show();

    x.push_back(0);
    y1.push_back(0);
    y2.push_back(0);
    y3.push_back(0);
    y4.push_back(0);
    y5.push_back(0);
    y6.push_back(0);
    y7.push_back(0);
    y8.push_back(0);
    y9.push_back(0);
    y10.push_back(0);
    y11.push_back(0);
    y12.push_back(0);

    while(1)
    {
        handle_start_of_periodic_task(next);
    	// PLOTTING CODE HERE
        // Update the plot with new data
        
        if(tello->controller->get_time() > x[x.size()-1])
        {

            // VELOCITY and Position DATA ==================================================================================
            // x.push_back(tello->controller->get_time());
            // // y1.push_back(dpc_curr(0));
            // // y2.push_back(dpc_curr(1));
            // // y3.push_back(dpc_curr(2));

            // y4.push_back(CoM_vel(0));
            // y5.push_back(CoM_vel(1));
            // y6.push_back(CoM_vel(2));

            // // // y7.push_back(pc_curr(0));
            // // // y8.push_back(pc_curr(1));
            // // // y9.push_back(pc_curr(2));

            // // // y10.push_back(tello->get_filter_state().getPosition()(0));
            // // // y11.push_back(tello->get_filter_state().getPosition()(1));
            // // // y12.push_back(CoM_z_last);

            // // if(tello->controller->get_time() - last_plot_time > 0.1){
            // //     last_plot_time = tello->controller->get_time();
            //     plt::rcparams({{"legend.loc","lower left"}});
            //     plt::clf();
            // //     // plt::subplot(3, 2, 1);
            // //     // plt::title("CoM X Position True vs EKF");
            // //     // plt::named_plot("True X", x, y7, "r-");
            // //     // plt::named_plot("EKF X", x, y10, "b-");
            // //     // plt::legend();
            // //     // plt::subplot(3, 2, 3);
            // //     // plt::title("CoM Y Position True vs EKF");
            // //     // plt::named_plot("True Y", x, y8, "r-");
            // //     // plt::named_plot("EKF Y", x, y11, "b-");
            // //     // plt::legend();
            // //     // plt::subplot(3, 2, 5);
            // //     // plt::title("CoM Z Position True vs Kinematics");
            // //     // plt::named_plot("True Z", x, y9, "r-");
            // //     // plt::named_plot("Kin Z", x, y12, "b-");
            // //     // plt::legend();
            //     // plt::subplot(3, 1, 1);
            // //     plt::title("CoM X Velocity True vs Estimated");
            // //     plt::named_plot("True dX", x, y1, "r-");
            //     plt::named_plot("Est. dX", x, y4, "r-");
            // //     plt::legend();
            // //     plt::subplot(3, 1, 2);
            // //     plt::title("CoM X Velocity True vs Estimated");
            // //     plt::named_plot("True dY", x, y2, "r-");
            //     plt::named_plot("Est.  dY", x, y5, "g-");
            // //     plt::legend();
            // //     plt::subplot(3, 1, 3);
            // //     plt::title("CoM X Velocity True vs Estimated");
            // //     plt::named_plot("True dZ", x, y3, "r-");
            //     plt::named_plot("Est. dZ", x, y6, "b-");
            //     plt::legend();
            //     plt::pause(0.001);
            // // }
            

            // ACCELEROMETER DATA: ==============================================================================
            // x.push_back(tello->controller->get_time());
            // y1.push_back(tello->_acc(0));
            // y2.push_back(tello->_acc(1));
            // y3.push_back(tello->_acc(2));

            // // y4.push_back(tello->_acc(0));
            // // y5.push_back(tello->_acc(1));
            // // y6.push_back(tello->_acc(2));

            // plt::clf();
            // plt::named_plot("acc 0", x, y1, "r-");
            // plt::named_plot("acc 1", x, y2, "b-");
            // plt::named_plot("acc 2", x, y3, "g-");
            // plt::title("IMU Data");
            // plt::legend();
            // plt::pause(0.001);

            // GYRO DATA: =========================================================================================
            // x.push_back(tello->controller->get_time());
            // y1.push_back(tello->_gyro(0));
            // y2.push_back(tello->_gyro(1));
            // y3.push_back(tello->_gyro(2));

            // y4.push_back(tello->controller->get_dEA()(0));
            // y5.push_back(tello->controller->get_dEA()(1));
            // y6.push_back(tello->controller->get_dEA()(2));

            // plt::clf();
            // plt::named_plot("gyro 0", x, y1, "r-");
            // plt::named_plot("gyro 1", x, y2, "b-");
            // plt::named_plot("gyro 2", x, y3, "g-");
            
            // plt::named_plot("dEA 0", x, y4, "r--");
            // plt::named_plot("dEA 1", x, y5, "b--");
            // plt::named_plot("dEA 2", x, y6, "g--");

            // plt::title("IMU Data");
            // plt::legend();
            // plt::pause(0.001);


        }
        usleep(100);
		//handle_end_of_periodic_task(next,period);
	}
    return  0;
}