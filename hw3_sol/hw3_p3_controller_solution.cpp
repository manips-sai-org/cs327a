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

        // Update joint space gravity
        robot->gravityVector(g);

        // Update current end effector position, orientation
        robot->position(ee_pos, ee_link_name, Eigen::Vector3d::Zero());
        robot->rotation(ee_rot_mat, ee_link_name);

        // Update operational space dynamics
        robot->J_0(J0, ee_link_name, Eigen::Vector3d::Zero());
        L0 = (J0 * robot->_M_inv * J0.transpose()).inverse();
        J_bar = robot->_M_inv * J0.transpose() * L0;
        N_bar = In - J_bar*J0;
        p = J_bar.transpose()*g;

        // Update current end effector velocity (v,w)
        v0 = J0*robot->_dq; 

        // Update desired end-effector position, velocity and acceleration
        double R = 0.1; //m
        double T = 5.0; //sec
        double xd = 0.0;
        double yd = 0.5 + R*cos(2.0*M_PI*curr_time/T);
        double zd = 0.6 + R/2.0 - R/2.0*cos(4.0*M_PI*curr_time/T);

        Eigen::Quaterniond lambdad (
            1/sqrt(2)*sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T)),
            1/sqrt(2)*cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T)),
            1/sqrt(2)*sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T)),
            1/sqrt(2)*cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T))
        );

        double dxd = 0.0;
        double dyd = -2.0*M_PI*R/T*sin(2.0*M_PI*curr_time/T);
        double dzd = 2.0*M_PI*R/T*sin(4.0*M_PI*curr_time/T);

        Eigen::Quaterniond dlambdad (
            -pow(M_PI, 2)/(2*T*sqrt(2.0))*cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T))*sin(2.0*M_PI*curr_time/T),
            pow(M_PI, 2)/(2*T*sqrt(2.0))*sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T))*sin(2.0*M_PI*curr_time/T),
            -pow(M_PI, 2)/(2*T*sqrt(2.0))*cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T))*sin(2.0*M_PI*curr_time/T),
            pow(M_PI, 2)/(2*T*sqrt(2.0))*sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T))*sin(2.0*M_PI*curr_time/T)
        );

        double ddxd = 0.0;
        double ddyd = -4.0*M_PI*M_PI/T*R/T*cos(2.0*M_PI*curr_time/T);
        double ddzd = 8.0*M_PI*M_PI/T*R/T*cos(4.0*M_PI*curr_time/T);

        Eigen::Quaterniond ddlambdad(
            1/sqrt(2)*(-sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(-pow(M_PI, 2)/(2.0*T)*sin(2.0*M_PI*curr_time/T), 2) - cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(M_PI, 3)/(T*T)*cos(2.0*M_PI*curr_time/T)),
            1/sqrt(2)*(-cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(-pow(M_PI, 2)/(2.0*T)*sin(2.0*M_PI*curr_time/T), 2) + sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(M_PI, 3)/(T*T)*cos(2.0*M_PI*curr_time/T)),
            1/sqrt(2)*(-sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(-pow(M_PI, 2)/(2.0*T)*sin(2.0*M_PI*curr_time/T), 2) - cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(M_PI, 3)/(T*T)*cos(2.0*M_PI*curr_time/T)),
            1/sqrt(2)*(-cos(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(-pow(M_PI, 2)/(2.0*T)*sin(2.0*M_PI*curr_time/T), 2) + sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T)) * pow(M_PI, 3)/(T*T)*cos(2.0*M_PI*curr_time/T))
        );

        // --------------------------------------------------------------------
        // (2) Compute desired operational space trajectory values and errors 
        //---------------------------------------------------------------------

        // - Convert end effector rotation matrix to quaternion for computing 
        //   the end effector orientation error and desired op space v, dv 

        //          *** Note *** They way eigen does this conversion leads to some sign issues (see pg.19-20 of the course reader)
        //          So, we have to check if the biggest element in the quaternion has changed sign to make 
        //          sure the conversion from rotation matrix to quaternion is correct

        Eigen::Quaterniond temp_ee_rot_lambda = Eigen::Quaterniond(ee_rot_mat); // new rotation quaternion
        Eigen::VectorXd::Index max_ind = 0;
        Eigen::VectorXd temp_vec = temp_ee_rot_lambda.coeffs().cwiseAbs(); // get absolute values
        temp_vec.cwiseMax(max_ind); // guaranteed to be > 0.5

        // ---------------------------------- sign check ---------------------------------//
        if ((temp_ee_rot_lambda.coeffs())[max_ind]*(ee_rot_lambda.coeffs())[max_ind] < 0) // Flip sign
        {   
            // ee_rot_lambda = -temp_ee_rot_lambda; <-- Eigen does not allow this unfortunately
            ee_rot_lambda = Eigen::Quaterniond(
                -temp_ee_rot_lambda.w(),
                -temp_ee_rot_lambda.x(),
                -temp_ee_rot_lambda.y(),
                -temp_ee_rot_lambda.z() );
        } 
        else // do NOT flip sign (eigen did it right)
        {
            ee_rot_lambda = temp_ee_rot_lambda;
        }
        // ---------------------------------- sign check end ---------------------------------//

        // - Compute desired operational space velocity and acceleration 
        v0d << dxd, dyd, dzd, 2*(dlambdad * ee_rot_lambda.conjugate()).vec();
        dv0d << ddxd, ddyd, ddzd, 2*(ddlambdad * ee_rot_lambda.conjugate()).vec();

        // - Compute current position and orientation error 
        //          *** Note *** For the orientation  --> delta_phi = -Er+*labmdad)
        //          remember from hw1: Er+*labmdad = 2 * labmdad * lambda.conjugate()
        ee_error << (ee_pos - Eigen::Vector3d(xd, yd, zd)), -2*(lambdad * ee_rot_lambda.conjugate()).vec();


        // ---------------------------------------------------------------------------------
        // (3) Compute joint torques
        //----------------------------------------------------------------------------------

        // Selection matrices
        select_motion = Eigen::MatrixXd::Identity(6,6);
        select_motion(0,0) = 0; // no motion control in global x position
        select_motion(4,4) = 0; // no rotation control about global y axis
        select_motion(5,5) = 0; // no rotation control about global z axis
        select_forces = Eigen::MatrixXd::Identity(6,6) - select_motion;

        // Desired forces/moments
        F_d << 10, 0, 0, 0, 0, 0;

        // Control torques
        command_torques = J0.transpose()*(
                L0*select_motion*(dv0d - op_task_kp*ee_error - op_task_kv*(v0 - v0d)) + // op. space F_motion for trajectory tracking
                select_forces*F_d + // Fdes feedfoward term 
                L0*select_forces*(- op_task_kv*(v0)) + // Op space damping: damps lin. vel. in x and angular vel. about y and z
                p ) + // gravity compensation in op. space
                N_bar.transpose()*(robot->_M*(-joint_damping_kv*robot->_dq) + g); // joint space damping and gravity comp. in the null space

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
