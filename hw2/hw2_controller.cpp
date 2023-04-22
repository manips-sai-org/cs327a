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
const string world_file = "./resources/world_hw2.urdf";
const string robot_file = "../../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA"; 
const string ee_link_name = "link6";

// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - read (from simulation/robot):
const std::string JOINT_ANGLES_KEY  = "cs327a::robot::Kuka-IIWA::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs327a::robot::Kuka-IIWA::sensors::dq";
// - read (from user defined task space control):
const std::string TASK_KP_GAINS_KEY = "cs327a::robot::Kuka-IIWA::inputs::op_task_kp";
const std::string TASK_KV_GAINS_KEY = "cs327a::robot::Kuka-IIWA::inputs::op_task_kv";

// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY = "cs327a::robot::Kuka-IIWA::actuators::fgc";
const std::string PLOT_CURRENT_EE_POSITION_KEY = "cs327a::plot::Kuka-IIWA::sensors::ee_pos";
const std::string PLOT_DESIRED_EE_POSITION_KEY = "cs327a::plot::Kuka-IIWA::inputs::ee_pos_des";

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

    // Suggested controller variables
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd g(robot->dof()); //joint space gravity vector
    MatrixXd J0(6, robot->dof()); //end effector Basic Jacobian,
    MatrixXd L_hat(6, 6); //Kinetic Energy Matrix at end effector (op. space)
    VectorXd p_hat(6); //gravity vector at end-effector (op. space)
    const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(robot->dof(), robot->dof()); // n x n identity matrix
    Eigen::MatrixXd J_pseudo(robot->dof(), 6);  //Right Psuedoinverse of J0
    Eigen::MatrixXd N_proj(robot->dof(), robot->dof()); //I - J_pseudo*J0, null space projection matrix
    Vector3d ee_pos; //end effector position
    Matrix3d ee_rot_mat; //end effector rotation, in matrix form
    robot->rotation(ee_rot_mat, ee_link_name); // initialize the end effector rotation
    Quaterniond ee_rot_lambda(ee_rot_mat); // end effector rotation in quaternion form
    VectorXd v0(6); //end effector velocity
    Vector3d ee_pos_des; // end effector desired position
    VectorXd v0_des(6); //end effector desired velocity
    VectorXd dv0_des(6); //end effector desired acceleration
    VectorXd ee_error(6); //end effector operational space instantaneous error (position and orientation)

    // suggested starting gains
    double op_task_kp = 50; //operational space task proportional gain
    double op_task_kv = 20; //operational space task velocity gain
    double damping_kv = 20; // joint damping velocity gain

    // initialize redis keys

    // op task
    // redis_client.setEigenMatrixJSON(DESIRED_TASK_POSITION_KEY, desired_position);
    redis_client.set(TASK_KP_GAINS_KEY, std::to_string(op_task_kp));
    redis_client.set(TASK_KV_GAINS_KEY, std::to_string(op_task_kv));

    // prepare redis read callback
    redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.addDoubleToReadCallback(0, TASK_KP_GAINS_KEY, op_task_kp);
    redis_client.addDoubleToReadCallback(0, TASK_KV_GAINS_KEY, op_task_kv);

    // prepare redis write callback
    redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
    redis_client.addEigenToWriteCallback(0, PLOT_CURRENT_EE_POSITION_KEY, ee_pos);
    redis_client.addEigenToWriteCallback(0, PLOT_DESIRED_EE_POSITION_KEY, ee_pos_des);

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	timer.initializeTimer(1000000); // 1 ms pause before starting loop
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
