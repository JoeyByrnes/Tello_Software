#include "mocap.h"

OWL::Context owl;

using namespace std;
using namespace Eigen;

extern Vector3d CoM_pos;
Vector3d CoM_pos_last;

VectorXd CoM_x_vels(100);
VectorXd CoM_y_vels(100);
VectorXd CoM_z_vels(100);
extern VectorXd CoM_quat;
extern Vector3d CoM_rpy;
extern Vector3d CoM_vel;

double time_last;

extern ctrlData cd_shared;

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

  // Front of robot tracker
  owl.assignMarker(tracker_id, 0, "m0", "pos=122.3860,177.9332,87.0012");
  owl.assignMarker(tracker_id, 1, "m1", "pos=121.5570,285.3265,90.6545");
  owl.assignMarker(tracker_id, 2, "m2", "pos=-121.9130,175.6818,98.1765");
  owl.assignMarker(tracker_id, 3, "m3", "pos=-122.0300,283.0585,101.9678");

  // Side of robot tracker
  // owl.assignMarker(tracker_id, 4, "m4", "pos=58.5059,288.5615,148.9263");
  // owl.assignMarker(tracker_id, 5, "m5", "pos=56.6612,190.9799,148.2066");
  // owl.assignMarker(tracker_id, 6, "m6", "pos=-56.6656,290.2258,150.8257");
  // owl.assignMarker(tracker_id, 7, "m7", "pos=-58.5014,192.2328,149.8414");

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
  // while(1)
  // {
  //     handle_start_of_periodic_task(next);
  //     // loop code starts here

  //     // loop code ends here
  //     handle_end_of_periodic_task(next,period);
  // }
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
              // if(r->cond > 0)
              //   cout << "  " << r->id << ") " << r->pose[0] << "," << r->pose[1] << "," << r->pose[2]
              //        << "," << r->pose[3] << "," << r->pose[4] << "," << r->pose[5] << "," << r->pose[6]
              //        << endl;
              Eigen::Quaterniond quaternion(r->pose[3], r->pose[6], r->pose[4], r->pose[5]);  // Quaternion (w, x, y, z)

              // Define the rotation angle in radians
              Eigen::Vector3d alignment_error(-0.0035, 0.0305, 0);

              Eigen::Quaterniond rotatedQuaternion = rotateQuaternion(quaternion, alignment_error);
              
              CoM_rpy = quaternionToEuler(rotatedQuaternion);
              CoM_quat << r->pose[3], r->pose[6], r->pose[4], r->pose[5];
              CoM_pos = Vector3d(r->pose[2]/1000.0, r->pose[0]/1000.0, r->pose[1]/1000.0-0.58+0.0212);
              if(!(r->pose[2] == 0.0 && r->pose[0] == 0.0 && r->pose[1] == 0.0))
              {
                stream_started = true;
              }
              if(stream_started && r->pose[2] == 0.0 && r->pose[0] == 0.0 && r->pose[1] == 0.0)
              {
                scheduleDisable();
                cout << "Lost Phasespace Tracking. Disabling Motors." << endl;
                stream_started = false;
              }
              if(first_time) 
              {
                CoM_pos_last = CoM_pos;
                first_time = false;
              }
              auto now = std::chrono::system_clock::now();  // Get the current time
              auto micros = std::chrono::time_point_cast<std::chrono::microseconds>(now);  // Round down to nearest microsecond
              auto since_epoch = micros.time_since_epoch();  // Get duration since epoch
              double dt = std::chrono::duration_cast<std::chrono::microseconds>(since_epoch).count() / 1000000.0 - time_last;  // Convert to double with resolution of microseconds

              Vector3d CoM_vel_raw = (CoM_pos - CoM_pos_last)/dt;
              CoM_pos_last = CoM_pos;
              CoM_x_vels.tail(99) = CoM_x_vels.head(99).eval();
              CoM_x_vels[0] = CoM_vel_raw(0);
              CoM_y_vels.tail(99) = CoM_y_vels.head(99).eval();
              CoM_y_vels[0] = CoM_vel_raw(1);
              CoM_z_vels.tail(99) = CoM_z_vels.head(99).eval();
              CoM_z_vels[0] = CoM_vel_raw(2);

              double dx_smoothed = smoothMocapData(CoM_x_vels,1.5);
              double dy_smoothed = smoothMocapData(CoM_y_vels,1.5);
              double dz_smoothed = smoothMocapData(CoM_z_vels,1.5);

              CoM_vel = Vector3d(dx_smoothed,dy_smoothed,dz_smoothed);

              // code for populating ctrlData struct here:
              cd_shared.setTime(tello->controller->get_time());
              // From Mocap
              cd_shared.setCoMPos(CoM_pos);
              cd_shared.setCoMVel(CoM_vel);
              cd_shared.setCoMRPY(CoM_rpy);
              // From tello sensors
              cd_shared.setCoMAcc(tello->_acc);
              cd_shared.setCoMAngVel(tello->_gyro);
              cd_shared.setJointPos(tello->getJointPositions());
              cd_shared.setJointVel(tello->getJointVelocities());

              // cout << "  " << r->id << ") " << r->pose[0] << ",     " << r->pose[1] << ",     " << r->pose[2]
              //        << ",     " << CoM_rpy(0)*180.0/M_PI << ",     " << CoM_rpy(1)*180.0/M_PI << ",     " << CoM_rpy(2)*180.0/M_PI << "                          \r";
              // cout.flush();
            }
          }
      }
  } // while

  owl.done();
  owl.close();

}