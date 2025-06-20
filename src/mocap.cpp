#include "mocap.h"

OWL::Context owl;

using namespace std;
using namespace Eigen;

extern Vector3d CoM_pos;
Vector3d CoM_pos_last;
Vector3d CoM_vel_last;
Vector3d CoM_rpy_last;
VectorXd CoM_rpy_rates;

VectorXd CoM_x_vels(100);
VectorXd CoM_y_vels(100);
VectorXd CoM_z_vels(100);
extern VectorXd CoM_quat;
extern Vector3d CoM_rpy;
extern Vector3d CoM_vel;
extern Vector3d CoM_rpy_vel;
extern Vector3d CoM_acc;

VectorXd CoM_x_accs(100);
VectorXd CoM_y_accs(100);
VectorXd CoM_z_accs(100);

VectorXd roll_rate(100);
VectorXd pitch_rate(100);
VectorXd yaw_rate(100);

double time_last;

int bad_data_counter = 0;

extern ctrlData cd_shared;

extern float roll_adjust;
extern float pitch_adjust;
extern float yaw_adjust;

Eigen::VectorXd quaternionToEuler(const Eigen::Quaterniond& quat) {
  Eigen::VectorXd euler(3);
  
  Eigen::Matrix3d rotation = quat.toRotationMatrix();
  
  double sy = sqrt(rotation(0, 0) * rotation(0, 0) + rotation(1, 0) * rotation(1, 0));
  
  bool singular = sy < 1e-6;
  
  if (!singular) {
    euler(0) = atan2(rotation(2, 1), rotation(2, 2));
    euler(1) = atan2(-rotation(2, 0), sy);
    euler(2) = atan2(rotation(1, 0), rotation(0, 0));
  } else {
    euler(0) = atan2(-rotation(1, 2), rotation(1, 1));
    euler(1) = atan2(-rotation(2, 0), sy);
    euler(2) = 0.0;
  }
  
  return euler;
}

Eigen::Quaterniond rotateQuaternion(const Eigen::Quaterniond& inputQuaternion, const Eigen::Vector3d& alignment_error) {
    // Create the rotation quaternions for each axis using axis-angle representation
    Eigen::Quaterniond qx(std::cos(alignment_error.x() / 2.0), std::sin(alignment_error.x() / 2.0), 0.0, 0.0);
    Eigen::Quaterniond qy(std::cos(alignment_error.y() / 2.0), 0.0, std::sin(alignment_error.y() / 2.0), 0.0);
    Eigen::Quaterniond qz(std::cos(alignment_error.z() / 2.0), 0.0, 0.0, std::sin(alignment_error.z() / 2.0));

    // Combine the rotations by multiplying the quaternions in the proper order
    Eigen::Quaterniond rotatedQuaternion = qz * qy * qx * inputQuaternion;

    return rotatedQuaternion;
}

double smoothMocapData(const Eigen::VectorXd& vel, double smoothingFactor) {
    int n = vel.size();
    Eigen::VectorXd smoothedVel(n);
    smoothedVel.setZero();

    if (n > 0) {
        smoothedVel(0) = vel(0);
        int maxSamples = std::min(n, 100);
        for (int i = 1; i < maxSamples; ++i) {
            double weight = smoothingFactor / (i + 1);
            smoothedVel(i) = (1 - weight) * smoothedVel(i - 1) + weight * vel(i);
        }
        for (int i = maxSamples; i < n; ++i) {
            double weight = smoothingFactor / maxSamples;
            smoothedVel(i) = (1 - weight) * smoothedVel(i - 1) + weight * vel(i);
        }
    }

    return smoothedVel(n - 1);
}

std::string formatPosition(float ypos, float zpos, float xpos) {
    std::stringstream ss;
    ss << "pos=" << std::fixed << std::setprecision(4) << ypos << "," << zpos << "," << xpos;
    return ss.str();
}

void* motion_capture( void * arg )
{

	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
  void* arg1 = std::get<1>(*arg_tuple_ptr);
	void* arg0 = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);
	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(arg0);

  string address = "192.168.1.20";
  OWL::Markers markers;
  OWL::Rigids rigids;

  if(owl.open(address) <= 0 || owl.initialize() <= 0) return 0;

  // create the rigid tracker
  uint32_t tracker_id = 0;
  owl.createTracker(tracker_id, "rigid", "CoM");

  float ypos_right = -121.8125;
  float ypos_left = 121.8125;
  float zpos_upper = 372.5;
  float zpos_lower = 264.5;
  float xpos = 94.4527;

  // Front of robot tracker
  // owl.assignMarker(tracker_id, 0, "m0", "pos=121.8125,176.5,94.4527");
  // owl.assignMarker(tracker_id, 1, "m1", "pos=121.8125,284.5,94.4527");
  // owl.assignMarker(tracker_id, 2, "m2", "pos=-121.8125,176.5,94.4527");
  // owl.assignMarker(tracker_id, 3, "m3", "pos=-121.8125,284.5,94.4527");

  float CoM2H_z_dist = tello->controller->get_SRB_params().CoM2H_z_dist*1000.0;
  float robot_LIP_height = tello->controller->get_SRB_params().hLIP;

  owl.assignMarker(tracker_id, 0, "m0", formatPosition(ypos_left,zpos_lower-CoM2H_z_dist,xpos));
  owl.assignMarker(tracker_id, 1, "m1", formatPosition(ypos_left,zpos_upper-CoM2H_z_dist,xpos));
  owl.assignMarker(tracker_id, 2, "m2", formatPosition(ypos_right,zpos_lower-CoM2H_z_dist,xpos));
  owl.assignMarker(tracker_id, 3, "m3", formatPosition(ypos_right,zpos_upper-CoM2H_z_dist,xpos));


  // start streaming
  owl.streaming(1);

  bool stream_started = false;

  auto now = std::chrono::system_clock::now();  // Get the current time
  auto micros = std::chrono::time_point_cast<std::chrono::microseconds>(now);  // Round down to nearest microsecond
  auto since_epoch = micros.time_since_epoch();  // Get duration since epoch
  time_last = std::chrono::duration_cast<std::chrono::microseconds>(since_epoch).count() / 1000000.0;  // Convert to double with resolution of microseconds
  usleep(1000);
  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);

  bool first_time = true;
  while(owl.isOpen() && owl.property<int>("initialized"))
  {
    const OWL::Event *event = owl.nextEvent(1000);
    if(!event) continue;

    if(event->type_id() == OWL::Type::ERROR)
      {
        cerr << event->name() << ": " << event->str() << endl;
      }
    else if(event->type_id() == OWL::Type::FRAME)
      {
      //   cout << "time=" << event->time() << " " << event->type_name() << " " << event->name() << "=" << event->size<OWL::Event>() << ":" << endl;
        if(event->find("markers", markers) > 0)
          {
          //   cout << " markers=" << markers.size() << ":" << endl;
          //   for(OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++)
          //     if(m->cond > 0)
          //       cout << "  " << m->id << ") " << m->x << "," << m->y << "," << m->z << endl;
          }
        if(event->find("rigids", rigids) > 0)
          {
          //   cout << " rigid=" << rigids.size() << ":" << endl;
            for(OWL::Rigids::iterator r = rigids.begin(); r != rigids.end(); r++)
            {
              if(r->id != tracker_id) continue;
              // if(r->cond > 0)
              //   cout << "  " << r->id << ") " << r->pose[0] << "," << r->pose[1] << "," << r->pose[2]
              //        << "," << r->pose[3] << "," << r->pose[4] << "," << r->pose[5] << "," << r->pose[6]
              //        << endl;
              Eigen::Quaterniond quaternion(r->pose[3], r->pose[6], r->pose[4], r->pose[5]);  // Quaternion (w, x, y, z)

              // Define the rotation angle in radians
              // Eigen::Vector3d alignment_error(0, 0, 0);

              // Eigen::Quaterniond rotatedQuaternion = rotateQuaternion(quaternion, alignment_error);
              
              CoM_rpy = quaternionToEuler(quaternion);

              if(CoM_rpy[0] == 0 && CoM_rpy[1] == 0 && CoM_rpy[2] == 0) continue;

              // Create the rotation matrix
              Eigen::Matrix3d rotationMatrix;
              rotationMatrix = Eigen::AngleAxisd(roll_adjust/*+0.02*/, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(pitch_adjust, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(yaw_adjust, Eigen::Vector3d::UnitZ());

              // Apply the rotation matrix to CoM_rpy
              CoM_rpy = rotationMatrix * CoM_rpy;
              CoM_quat << r->pose[3], r->pose[6], r->pose[4], r->pose[5];

              double alignment_z_offset = 0.002;
              double floor_mat_height = -0.001;
              double seesaw_offset = -0.076;
              CoM_pos = Vector3d(r->pose[2]/1000.0, r->pose[0]/1000.0, r->pose[1]/1000.0-robot_LIP_height + alignment_z_offset + floor_mat_height); //0.0272 //COM_HEIGHT
              CoM_pos = rotationMatrix * CoM_pos;
              if(!(r->pose[2] == 0.0 && r->pose[0] == 0.0 && r->pose[1] == 0.0))
              {
                stream_started = true;
              }
              if(stream_started && r->pose[2] == 0.0 && r->pose[0] == 0.0 && r->pose[1] == 0.0)
              {
                // bad_data_counter++;
                scheduleDisable();
                cout << "Lost Phasespace Tracking. Disabling Motors." << endl;
                stream_started = false;
              }
              // else{
              //   bad_data_counter = 0;
              // }
              // if(bad_data_counter > 4)
              // {
              //   scheduleDisable();
              //   cout << "Lost Phasespace Tracking. Disabling Motors." << endl;
              //   stream_started = false;
              // }
              if(first_time) 
              {
                CoM_pos_last = CoM_pos;
                first_time = false;
              }
              auto now = std::chrono::system_clock::now();  // Get the current time
              auto micros = std::chrono::time_point_cast<std::chrono::microseconds>(now);  // Round down to nearest microsecond
              auto since_epoch = micros.time_since_epoch();  // Get duration since epoch
              double time_now = std::chrono::duration_cast<std::chrono::microseconds>(since_epoch).count() / 1000000.0;  // Convert to double with resolution of microseconds
              double dt_check = time_now - time_last;
              if(dt_check < 0.001) continue;
              double dt = time_now - time_last;  // Convert to double with resolution of microseconds
              time_last = time_now;
              // cout << "dt: " << dt << endl;
              Vector3d CoM_vel_raw = (CoM_pos - CoM_pos_last)/dt;
              CoM_pos_last = CoM_pos;
              CoM_x_vels.tail(99) = CoM_x_vels.head(99).eval();
              CoM_x_vels[0] = CoM_vel_raw(0);
              CoM_y_vels.tail(99) = CoM_y_vels.head(99).eval();
              CoM_y_vels[0] = CoM_vel_raw(1);
              CoM_z_vels.tail(99) = CoM_z_vels.head(99).eval();
              CoM_z_vels[0] = CoM_vel_raw(2);

              double dx_smoothed = smoothMocapData(CoM_x_vels,0.1);
              double dy_smoothed = smoothMocapData(CoM_y_vels,0.1);
              double dz_smoothed = smoothMocapData(CoM_z_vels,0.1);

              CoM_vel = Vector3d(dx_smoothed,dy_smoothed,dz_smoothed);

              Vector3d CoM_acc_raw = (CoM_vel - CoM_vel_last) / dt;
              CoM_vel_last = CoM_vel;
              CoM_x_accs.tail(99) = CoM_x_accs.head(99).eval();
              CoM_x_accs[0] = CoM_acc_raw(0);
              CoM_y_accs.tail(99) = CoM_y_accs.head(99).eval();
              CoM_y_accs[0] = CoM_acc_raw(1);
              CoM_z_accs.tail(99) = CoM_z_accs.head(99).eval();
              CoM_z_accs[0] = CoM_acc_raw(2);

              double ddx_smoothed = smoothMocapData(CoM_x_accs, 0.1);
              double ddy_smoothed = smoothMocapData(CoM_y_accs, 0.1);
              double ddz_smoothed = smoothMocapData(CoM_z_accs, 0.1);

              CoM_acc = Vector3d(ddx_smoothed, ddy_smoothed, ddz_smoothed);

              VectorXd CoM_rpy_diff = (CoM_rpy - CoM_rpy_last);
              CoM_rpy_last = CoM_rpy;

              // // Ensure that angular differences are within the range [-pi, pi]
              // for (int i = 0; i < CoM_rpy_diff.size(); ++i) {
              //     if (CoM_rpy_diff(i) > M_PI)
              //         CoM_rpy_diff(i) -= 2 * M_PI;
              //     else if (CoM_rpy_diff(i) < -M_PI)
              //         CoM_rpy_diff(i) += 2 * M_PI;
              // }

              // Angular velocities (roll_rate, pitch_rate, yaw_rate)
              CoM_rpy_vel = CoM_rpy_diff / dt;

              roll_rate.tail(99) = roll_rate.head(99).eval();
              roll_rate[0] = CoM_rpy_vel(0);
              pitch_rate.tail(99) = pitch_rate.head(99).eval();
              pitch_rate[0] = CoM_rpy_vel(1);
              yaw_rate.tail(99) = yaw_rate.head(99).eval();
              yaw_rate[0] = CoM_rpy_vel(2);

              double roll_rate_smoothed = smoothMocapData(roll_rate, 0.3);
              double pitch_rate_smoothed = smoothMocapData(pitch_rate, 0.3);
              double yaw_rate_smoothed = smoothMocapData(yaw_rate, 0.3);

              CoM_rpy_rates = Vector3d(roll_rate_smoothed,pitch_rate_smoothed,yaw_rate_smoothed);

              // cout << CoM_rpy_rates[0] << ", " << CoM_rpy_rates[1] << ", " << CoM_rpy_rates[2] << endl;

              // code for populating ctrlData struct here:
              cd_shared.setTime(tello->controller->get_time());
              // From Mocap
              cd_shared.setCoMPos(CoM_pos);
              cd_shared.setCoMVel(CoM_vel);
              cd_shared.setCoMRPY(CoM_rpy);
              // From tello sensors
              cd_shared.setCoMAcc(CoM_acc);
              cd_shared.setCoMAngVel(CoM_rpy_rates);
              cd_shared.setJointPos(tello->getJointPositions());
              cd_shared.setJointVel(tello->getJointVelocities());

              // cout << "  " << r->id << ") " << r->pose[0] << ",     " << r->pose[1] << ",     " << r->pose[2]
              //        << ",     " << CoM_rpy(0)*180.0/M_PI << ",     " << CoM_rpy(1)*180.0/M_PI << ",     " << CoM_rpy(2)*180.0/M_PI << "                          \r";
              // cout.flush();
              // cout << CoM_rpy(0)-tello->_rpy(0) << ",     " << CoM_rpy(1)-tello->_rpy(1) << ",     " << CoM_rpy(2)-tello->_rpy(2) << "                          \r";
              // cout.flush();
            }
          }
      }
  } // while

  owl.done();
  owl.close();

}