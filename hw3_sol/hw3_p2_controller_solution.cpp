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
const string world_file = "./resources/world_hw3_p2.urdf";
const string robot_file = "../../resources/puma/puma.urdf";
const string robot_name = "Puma"; 
const string ee_link_name = "end-effector";

// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - read (from simulation/robot):
const std::string JOINT_ANGLES_KEY  = "cs327a::hw3::robot::Puma::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs327a::hw3::robot::Puma::sensors::dq";

// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY  = "cs327a::hw3::robot::Puma::actuators::fgc";

// problem part selection
enum PROBLEM_PART_TYPE {
    PART1=0,
    PART2,
    N_PARTS
};
PROBLEM_PART_TYPE enum_problem_part;
void selectProblemPart(char* input);

int main(int argc, char** argv) {
    // get problem part
    if (argc < 2) {
        cout << "Usage: ./hw3-p2-controller_solution <part: 1 or 2>" << endl;
        return 0;
    }
    selectProblemPart(argv[1]);


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

    // load robots, get desired position
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
    robot->_q << 90/180.0*M_PI,
            -22.5/180.0*M_PI,
            180/180.0*M_PI,
            90.0/180.0*M_PI,
            100/180.0*M_PI,
            180/180.0*M_PI;
    robot->updateModel();

    // end effector tip position in link frame
    Eigen::Vector3d ee_pos_local;
    ee_pos_local << -0.2, 0.0, 0.0;

    // end effector desired position is set to its starting position
    Eigen::Vector3d ee_des_pos;
    robot->position(ee_des_pos, ee_link_name, ee_pos_local);

    // update model to its current state
    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
    robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
    robot->updateModel();

    // final control torques to send to the robot
    VectorXd command_torques = VectorXd::Zero(robot->dof());

    // Suggested controller variables
    Eigen::VectorXd g(robot->dof()); //joint space gravity vector
    Eigen::MatrixXd Jv(3, robot->dof()); //end effector linear velocity Jacobian
    Eigen::MatrixXd Lv(3, 3); //Lambda_v at end effector
    const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(robot->dof(), robot->dof()); // n x n identity matrix
    Eigen::MatrixXd J_pseudo(robot->dof(), 3); //pseudo inverse of Jv
    Eigen::MatrixXd N_pseudo(robot->dof(), robot->dof()); //I - Jpseudo * Jv, null space projection matrix for pseudo-inverse
    Eigen::Vector3d p_pseudo; //gravity vector at end-effector computed with pseudoinverse
    Eigen::MatrixXd J_bar(robot->dof(), 3);  //A-weighted generalized inverse of Jv
    Eigen::MatrixXd N_bar(robot->dof(), robot->dof()); //I - Jbar*Jv, null space projection matrix for Jbar
    Eigen::Vector3d p_bar; //gravity vector at end-effector computed with pseudoinverse
    Eigen::Vector3d ee_pos; //end effector position
    Eigen::Vector3d v; //end effector velocity
    Eigen::VectorXd qd(robot->dof()); //desired joint positions

    // suggested starting gains
    double op_task_kp = 50; //operational space task proportional gain
    double op_task_kv = 20; //operational space task velocity gain
    double joint_task_kp = 50; // joint space task velocity gain
    double joint_task_kv = 20; // joint space task velocity gain

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

        // Update robot model parameters
        robot->position(ee_pos, ee_link_name, ee_pos_local);
        robot->Jv(Jv, ee_link_name, ee_pos_local);
        v = Jv*robot->_dq;
        Lv = (Jv * robot->_M_inv * Jv.transpose()).inverse(); // alternate formulation of Lambda matrix for redundant robots

        //pseudoinverse terms
        J_pseudo = Jv.transpose()*(Jv*Jv.transpose()).inverse();
        N_pseudo = (In - J_pseudo*Jv);
        p_pseudo = J_pseudo.transpose()*g;

        //dynamically consistent inverse terms
        J_bar = robot->_M_inv * Jv.transpose() * Lv;
        N_bar = In - J_bar*Jv;
        p_bar = J_bar.transpose()*g;

        //compute required null-space motion
        qd = robot->_q;
        qd[1] = -M_PI/8.0 + M_PI/8.0*sin(2.0*M_PI*curr_time/10.0);

        switch (enum_problem_part) {

            // (1) Pseudo-inverse decoupling
            //----------------------------------------------------
            case PART1:
                // control torques using pseudoinverse
                command_torques = Jv.transpose()*(Lv*(-op_task_kp*(ee_pos - ee_des_pos) - op_task_kv*(v)) + p_pseudo) +  // op. space trajectory tracking 
                        N_pseudo.transpose()*(robot->_M*(-joint_task_kp*(robot->_q - qd) - joint_task_kv*robot->_dq) + g); // null space joint trajectory tracking and joint damping
                // command_torques = Jv.transpose()*(Lv*(-op_task_kp*(ee_pos - ee_des_pos) - op_task_kv*(v)) + p_pseudo) +  // op. space trajectory tracking 
                        (robot->_M*(-joint_task_kv*robot->_dq)); // joint damping
                break;

            // (2) Null space dynamically consistent decoupling
            //----------------------------------------------------
            case PART2:
                // control torques using dynamically consistent inverse 
                command_torques = Jv.transpose()*(Lv*(-op_task_kp*(ee_pos - ee_des_pos) - op_task_kv*(v)) + p_bar) + 
                        N_bar.transpose()*(robot->_M*(-joint_task_kp*(robot->_q - qd) - joint_task_kv*robot->_dq) + g);
                break;

            default:
                command_torques.setZero();
                break;
        }

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

void selectProblemPart(char* input) {
    switch (input[0]) {
        case '1':
            enum_problem_part = PART1;
            break;
        case '2':
            enum_problem_part = PART2;
            break;
        default:
            cout << "Usage: ./hw3-p2-controller_solution <part: 1 or 2>" << endl;
            exit(0);
    }
}