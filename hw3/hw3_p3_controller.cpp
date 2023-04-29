#include <Sai2Model.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}


// Convenience functions for converting degrees to radians and vice-versa
#define DEG2RAD(deg) ((double)(deg) * M_PI / 180.0)
#define RAD2DEG(rad) ((double)(rad) * 180.0 / M_PI)

// Location of URDF files specifying world and robot information
const string world_file = "./resources/world_hw3_p3.urdf";
const string robot_file = "../../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA"; 
const string ee_link_name = "link6";

// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - read (from simulation/robot):
const std::string JOINT_ANGLES_KEY  = "cs327a::hw3::robot::Kuka-IIWA::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs327a::hw3::robot::Kuka-IIWA::sensors::dq";

// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY  = "cs327a::hw3::robot::Kuka-IIWA::actuators::fgc";

int main(int argc, char** argv) {
	// Make sure redis-server is running at localhost with default port 6379
	// start redis client
	RedisClient redis_client = RedisClient();
	redis_client.connect();

    // set up redis read + write callbacks
    redis_client.createReadCallback(0);
    redis_client.createWriteCallback(0);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

    // load robots, read current state and update the model
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();

    // end effector tip position in link frame
    Eigen::Vector3d ee_pos_local;
    ee_pos_local << 0.0, 0.0, 0.0;

    // final control torques to send to the robot
    VectorXd command_torques = VectorXd::Zero(robot->dof());

    // Suggested controller variables
    Eigen::VectorXd g(robot->dof()); //joint space gravity vector
    Eigen::MatrixXd J0(6, robot->dof()); //end effector basic Jacobian
    Eigen::MatrixXd L0(6, 6); //Lambda_0 at end effector
    Eigen::VectorXd p(6); //gravity vector at end-effector
    const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(robot->dof(), robot->dof()); // n x n identity matrix
    Eigen::MatrixXd J_bar(robot->dof(), 6);  //A-weighted generalized inverse of J0
    Eigen::MatrixXd N_bar(robot->dof(), robot->dof()); //I - Jbar*J0, null space projection matrix for Jbar
    Eigen::Vector3d ee_pos; //end effector position
    Eigen::Matrix3d ee_rot_mat; //end effector rotation
    robot->rotation(ee_rot_mat, ee_link_name); // initialize
    Eigen::Quaterniond ee_rot_lambda(ee_rot_mat); // end effector rotation in quaternion form
    Eigen::VectorXd ee_error(6); //end effector operational space instantaneous error
    Eigen::VectorXd v0(6); //end effector velocity
    Eigen::VectorXd v0d(6); //end effector desired velocity
    Eigen::VectorXd dv0d(6); //end effector desired acceleration
    Eigen::MatrixXd select_motion(6,6); // selection matrix for motion, Omega
    Eigen::MatrixXd select_forces(6,6); // selection matrix for forces, Omega_bar
    Eigen::VectorXd F_d(6); // desired end effector force

    // suggested starting gains
    double op_task_kp = 50; //operational space task proportional gain
    double op_task_kv = 20; //operational space task velocity gain
    double joint_damping_kv = 20; // joint damping velocity gain

    // initialize redis keys

    // prepare redis read callback
    redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

    // prepare redis write callback
    redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a loop timer
    double control_freq = 200;
    LoopTimer timer;
    timer.setLoopFrequency(control_freq);
    double last_time = timer.elapsedTime(); //secs
    bool fTimerDidSleep = true;
    timer.initializeTimer(2000);
    double start_time = timer.elapsedTime();

	unsigned long long counter = 0;

    cout << "Starting control loop" << endl;

	runloop = true;
	while (runloop) 
	{ 
		fTimerDidSleep = timer.waitForNextLoop();
        double curr_time = timer.elapsedTime() - start_time;
        double loop_dt = curr_time - last_time;

        // read controller parameters from redis
        redis_client.executeReadCallback(0);

        // update robot model
        robot->updateModel();
        robot->gravityVector(g);

        // reset command torques
        command_torques.setZero();

		/* ------------------------------------------------------------------------------------
            FILL ME IN: set joint torques
        -------------------------------------------------------------------------------------*/

        // ---------------------------------------------------------------------------------
        // (1) Update current robot configuration parameters and desired trajectory values 
        //----------------------------------------------------------------------------------

        
        // --------------------------------------------------------------------
        // (2) Compute desired operational space trajectory values and errors 
        //---------------------------------------------------------------------
        

        // ---------------------------------------------------------------------------------
        // (3) Compute joint torques
        //----------------------------------------------------------------------------------

       
		/* ------------------------------------------------------------------------------------
            END OF FILL ME IN
        -------------------------------------------------------------------------------------*/

        // send torques to redis
        redis_client.executeWriteCallback(0);

		counter++;
		last_time = curr_time;
	}

    command_torques.setZero();
    redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	double end_time = timer.elapsedTime();
    cout << "\n";
    cout << "Control Loop run time  : " << end_time << " seconds\n";
    cout << "Control Loop updates   : " << timer.elapsedCycles() << "\n";
    cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;
}
