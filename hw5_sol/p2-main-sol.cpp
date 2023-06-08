/* --------------------------------------------------------------------------------------
   Include Required Libraries and Files
-----------------------------------------------------------------------------------------*/
#include <iostream>
#include <string>
#include <csignal>
#include <thread>
#include <chrono>
#include <math.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <dynamics3d.h>
#include "timer/LoopTimer.h"
#include <GLFW/glfw3.h> 

#include "keys.h"

using namespace std;

const double time_slowdown_factor = 10;

constexpr const char *sim_title = "SAI2.0 - CS327a HW5 P2 Solution";
const string world_fname = "./resources/world_hw5_p2.urdf";
const string robot_fname = "../../resources/puma/puma_gripper.urdf";
const string robot1_name = "Puma1";
const string robot2_name = "Puma2";
const string object_name = "CoordObject";
const string object_fname = "./resources/object.urdf";
const string object_link_name = "object";
const string camera_name = "camera_front";
const string ee_link_name = "end-effector";
const string gripper_joint_name = "gripper";

RedisClient redis_client;

// trajectory type selection
enum TRAJECTORY {
	TRAJECTORY1=0,
	TRAJECTORY2,
	TRAJECTORY3,
	N_TRAJECTORIES
};
TRAJECTORY enum_trajectory;
void selectTrajectory(char* input);

/* ----------------------------------------------------------------------------------
	Simulation and Control Loop Setup
-------------------------------------------------------------------------------------*/
// state machine setup
enum ControlMode {
	CONTROL_GRASP_STABILIZE = 0,
	CONTROL_AUGMENTED_OBJECT
};

// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* object_model, Simulation::Sai2Simulation* sim);
void simulation(Simulation::Sai2Simulation* sim);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

/* =======================================================================================
   MAIN LOOP
========================================================================================== */
int main (int argc, char** argv) {
	
	// get trajectory
	if (argc < 2) {
		cout << "Usage: ./hw5-p2-sol <trajectory: 1 or 2 or 3>" << endl;
		return 0;
	}
	selectTrajectory(argv[1]);

	cout << "Loading URDF world model file: " << world_fname << endl;

    // set up redis callbacks
    redis_client.connect();
    redis_client.createReadCallback(0);
    redis_client.createWriteCallback(0);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, false);

	// load robots
	auto robot1 = new Sai2Model::Sai2Model(robot_fname, false);
	auto robot2 = new Sai2Model::Sai2Model(robot_fname, false);

	// load object
	auto coobject = new Sai2Model::Sai2Model(object_fname, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);
	// set co-efficient of restition to zero to avoid bounce
    // see issue: https://github.com/manips-sai/sai2-simulation/issues/1
    sim->setCollisionRestitution(0.0);
    sim->setCoeffFrictionStatic(0.5);
    sim->setCoeffFrictionDynamic(0.5);

    // set joint damping on grippers: 
    auto base_1 = sim->_world->getBaseNode(robot1_name);
    auto gripper_1 = base_1->getJoint(gripper_joint_name);
    gripper_1->setDamping(10.0);
    gripper_1->setJointLimits(-0.005, 0.068, 0.005);
    auto base_2 = sim->_world->getBaseNode(robot2_name);
    auto gripper_2 = base_2->getJoint(gripper_joint_name);
    gripper_2->setDamping(10.0);
    gripper_2->setJointLimits(-0.005, 0.068, 0.005);

    // set initial conditions
	robot1->_q << 90/180.0*M_PI,
				-22.5/180.0*M_PI,
				212/180.0*M_PI,
				90.0/180.0*M_PI,
				100/180.0*M_PI,
				180/180.0*M_PI,
				0.0;
	sim->setJointPositions(robot1_name, robot1->_q);
	robot1->updateModel();
	robot2->_q << 90/180.0*M_PI,
				202.5/180.0*M_PI,
				-28/180.0*M_PI,
				-90.0/180.0*M_PI,
				97/180.0*M_PI,
				180/180.0*M_PI,
				0.0;
	sim->setJointPositions(robot2_name, robot2->_q);
	robot2->updateModel();
	Eigen::Affine3d ee_trans;

	// set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor *primary = glfwGetPrimaryMonitor();
    const GLFWvidmode *mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
    int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow *window = glfwCreateWindow(windowW, windowH, sim_title, NULL, NULL);
    glfwSetWindowPos(window, windowPosX, windowPosY);
    glfwShowWindow(window);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // set callbacks
    glfwSetKeyCallback(window, keySelect);

    // cache variables
    double last_cursorx, last_cursory;

	// start the simulation thread first
    fSimulationRunning = true;
	thread sim_thread(simulation, sim);

	// next start the control thread
	thread ctrl_thread(control, robot1, robot2, coobject, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot1_name, robot1);
		graphics->updateGraphics(robot2_name, robot2);
		graphics->updateGraphics(object_name, coobject);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

/* ----------------------------------------------------------------------------------
	Utility functions
-------------------------------------------------------------------------------------*/
// Calculate the cross product matrix
Eigen::Matrix3d getCrossProductMat(const Eigen::Vector3d& t) {
	Eigen::Matrix3d ret;
	ret <<  0, -t(2), t(1),
			t(2), 0, -t(0),
			-t(1), t(0), 0;
	return ret;
}

//------------------------------------------------------------------------------
void selectTrajectory(char* input) {
	switch (input[0]) {
		case '1':
			enum_trajectory = TRAJECTORY1;
			break;
		case '2':
			enum_trajectory = TRAJECTORY2;
			break;
		case '3':
			enum_trajectory = TRAJECTORY3;
			break;
		default:
			cout << "Usage: ./hw5-p2-sol <trajectory: 1 or 2 or 3>" << endl;
			exit(0);
	}
}

/* =======================================================================================
   CONTROL LOOP
========================================================================================== */
void control(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* object_model, Simulation::Sai2Simulation* sim) {
	
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1000Hz timer
	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs

	// load robot global frame to robot base transformations: todo: move out to a function
	Eigen::Affine3d robot1_base_frame = sim->getRobotBaseTransform(robot1_name);
	Eigen::Affine3d robot2_base_frame = sim->getRobotBaseTransform(robot2_name);

	// cache variables
	bool fTimerDidSleep = true;
	bool fTorqueUseDefaults = false; // set true when torques are overriden for the first time
	Eigen::VectorXd tau1 = Eigen::VectorXd::Zero(robot1->dof());
	Eigen::VectorXd tau2 = Eigen::VectorXd::Zero(robot2->dof());

	Eigen::Affine3d object_com_frame;
	Eigen::Vector3d object_current_pos;
	Eigen::MatrixXd object_inertia(6,6);
	Eigen::MatrixXd object_j(6,6);
	Eigen::VectorXd object_p(6);

	Eigen::VectorXd robot1_g(robot1->dof());
	Eigen::VectorXd robot2_g(robot2->dof());

	// **  Other sugested variables and gains **
	Eigen::Vector3d object_com_in_robot1_ee_frame;
	Eigen::Vector3d object_com_in_robot2_ee_frame;
	Eigen::MatrixXd robot1_j0_objcom(6, robot1->dof());
	Eigen::MatrixXd robot2_j0_objcom(6, robot2->dof());
	Eigen::MatrixXd robot1_j0_objectcom_bar(robot1->dof(), 6);
	Eigen::MatrixXd robot2_j0_objectcom_bar(robot2->dof(), 6);
	Eigen::MatrixXd robot1_objcom_inertia(6,6);
	Eigen::MatrixXd robot2_objcom_inertia(6,6);

	Eigen::MatrixXd augmented_object_inertia(6,6);
	Eigen::VectorXd augmented_object_p(6);

	Eigen::MatrixXd G(2*6, 2*6);
	Eigen::MatrixXd W(6, 2*6);

	Eigen::Vector3d obj_des_pos;
	Eigen::Vector3d obj_ori_error;
	Eigen::VectorXd obj_task_err(6);
	Eigen::VectorXd force_des_vec(12);
	Eigen::VectorXd force_ee_vec(12);

	double kp = 30;
	double kv = 10;

	// ** Control Mode **
	// 		0 = grasp stabilizing controller
	//		1 = augmented object controller
	ControlMode control_mode = CONTROL_GRASP_STABILIZE; 

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time;

        // read joint positions, velocities
        sim->getJointPositions(robot1_name, robot1->_q);
        sim->getJointVelocities(robot1_name, robot1->_dq);
        robot1->updateModel();
        sim->getJointPositions(robot2_name, robot2->_q);
        sim->getJointVelocities(robot2_name, robot2->_dq);
        robot2->updateModel();

        // read object position
        sim->getJointPositions(object_name, object_model->_q);
        sim->getJointVelocities(object_name, object_model->_dq);
        object_model->updateModel();

        // update object dynamics
		// - find object COM frame in global frame
		object_model->transform(object_com_frame, object_link_name);
		object_current_pos << object_com_frame.translation();
		redis_client.addEigenToWriteCallback(0, CURRENT_OBJECT_POSITION_KEY, object_current_pos);
		// - obtain object inertia matrix in world frame
		object_model->J_0(object_j, object_link_name, Eigen::Vector3d::Zero());
		object_inertia = (object_j*object_model->_M_inv*object_j.transpose()).inverse();
		// - obtain object p
		object_p << 0, 0, 9.8*0.5, 0, 0, 0;

        // ---------------------------------------------------------------------------------
        /* ---------------------- FILL ME IN ----------------------------------------------- */
		
		// --------------------------------------------------------------------
		// (1) Manipulator Jacobians
		//---------------------------------------------------------------------
		// ** FILL ME IN **

		// Obtain object COM location in each robot end effector frame
		// 		world frame -> base frame -> end effector
		//		- object_com_frame: from the world frame to the object_com_frame 
		//		- robot_base_frame: from the world frame to robot base frame 
		//		- robot_ee_frame: from the robot base frame to robot ee frame 
		Eigen::Affine3d robot1_ee_frame;
		robot1->transform(robot1_ee_frame, ee_link_name);
		object_com_in_robot1_ee_frame = ((robot1_base_frame * robot1_ee_frame).inverse() * object_com_frame).translation();
		
		Eigen::Affine3d robot2_ee_frame;
		robot2->transform(robot2_ee_frame, ee_link_name);
		object_com_in_robot2_ee_frame = ((robot2_base_frame * robot2_ee_frame).inverse() * object_com_frame).translation();
		
		// J0 is the jacobian at the point of interest (in this case object com) measured in base frame
		robot1->J_0(robot1_j0_objcom, ee_link_name, object_com_in_robot1_ee_frame);
		robot2->J_0(robot2_j0_objcom, ee_link_name, object_com_in_robot2_ee_frame);

		// --------------------------------------------------------------------
		// (2) Augmented Object Model
		//---------------------------------------------------------------------
		// ** FILL ME IN **

		// Lambda at the object COM for each robot and augmented object lambda
		robot1_objcom_inertia = (robot1_j0_objcom*robot1->_M_inv*robot1_j0_objcom.transpose()).inverse();
		robot2_objcom_inertia = (robot2_j0_objcom*robot2->_M_inv*robot2_j0_objcom.transpose()).inverse();
		augmented_object_inertia = object_inertia + robot1_objcom_inertia + robot2_objcom_inertia;

		// Jbar for each robot at the object COM
		robot1_j0_objectcom_bar = robot1->_M_inv*robot1_j0_objcom.transpose()*robot1_objcom_inertia;	
		robot2_j0_objectcom_bar = robot2->_M_inv*robot2_j0_objcom.transpose()*robot2_objcom_inertia;

		// Gravity reflected at the point of interest
		robot1->gravityVector(robot1_g);
		robot2->gravityVector(robot2_g);
		augmented_object_p = object_p + robot1_j0_objectcom_bar.transpose()*robot1_g + robot2_j0_objectcom_bar.transpose()*robot2_g;


		// --------------------------------------------------------------------
		// (3) Grasp Matrix
		//---------------------------------------------------------------------
		// ** FILL ME IN **

		// Compute the different blocks that form G: 
		// 				  Wf        Wm
		// 			G =	  Ebar      Zero_1x6
		// 				  Zero_5x6  I_tilda

		// Remember: 
		//		- object_com_frame: from the world frame to the object_com_frame 
		//		- robot_base_frame: from the world frame to robot base frame 
		//		- robot_ee_frame: from the robot base frame to robot ee frame 

		// - W: relates the external forces and moments to the resulting forces and moments (fr, mr) 
		Eigen::Matrix3d I3 = Eigen::MatrixXd::Identity(3,3); 
		Eigen::Matrix3d Zero_3x3 = Eigen::MatrixXd::Zero(3,3); 
		Eigen::Vector3d r1, r2; // distance vector FROM object com TO grasping point IN world frame 
		r1 = (robot1_base_frame * robot1_ee_frame).translation() - object_com_frame.translation();
		r2 = (robot2_base_frame * robot2_ee_frame).translation() - object_com_frame.translation();
		W << I3, I3, Zero_3x3, Zero_3x3, 
			getCrossProductMat(r1), getCrossProductMat(r2), I3, I3;
		
		// - Ebar: relates applied force to resulting internal tension
		Eigen::MatrixXd Ebar(1, 6);
		Eigen::Vector3d e12; // UNIT vector in the direction FROM one grasp point TO the other IN world frame
		e12 = (robot2_base_frame * robot2_ee_frame).translation() - (robot1_base_frame * robot1_ee_frame).translation();
		e12.normalize();
		Ebar << -0.5*e12.transpose(), 0.5*e12.transpose();

		// - I_tilda: relates the 5 internal moments (tau) to the external applied moments (m)
		//		note: 
		//		* tau are in a frame that connects the two end effectors (which is the Y-axis if you don't drop the object)
		//		* m are in world frame 
		//		* we want tau to be zero
		//		* If we had more than 2 manipulators I_tilda would simply be Identity
		Eigen::MatrixXd I_tilda(5, 6);
		Eigen::Vector3d e1x, e1y, e1z, e2x, e2y, e2z;   
		e1x = e12; //we define the frame X-axis to be along the connecting line between the 2 end-effectors
		e2x = -e12;
		e1z = robot1_ee_frame.linear().col(2);
		e1y = e1z.cross(e1x);
		e2z = robot2_ee_frame.linear().col(2);
		e2y = e2z.cross(e2x);
		I_tilda <<	e1x.transpose(), e2x.transpose(),
		  			e1y.transpose(), 0, 0, 0,
					e1z.transpose(), 0, 0, 0,
					0, 0, 0, 		 e2y.transpose(),
					0, 0, 0, 		 e2z.transpose();
		
		// - complete G
		Eigen::MatrixXd Zero_1x6 = Eigen::MatrixXd::Zero(1, 6);
		Eigen::MatrixXd Zero_5x6 = Eigen::MatrixXd::Zero(5, 6);
		
		G << W,
			Ebar,     Zero_1x6,
			Zero_5x6, I_tilda;

		// --------------------------------------------------------------------
		// (4) Force Control 
		//---------------------------------------------------------------------
		
		/* ----------------------------------------------------------------------------------
			Augmented object control torques   ** FILL ME IN **
		-------------------------------------------------------------------------------------*/ 

		if (control_mode == CONTROL_AUGMENTED_OBJECT) { 

			// desired object com position
			switch (enum_trajectory) {
			case TRAJECTORY1: //y-axis
				obj_des_pos << 0.0, 0.15*sin(2.0*M_PI/3.0*curr_time), 0.4;
				break;
			case TRAJECTORY2: //xy circle
				obj_des_pos << 0.15*cos(5.0*M_PI/3.0*curr_time), 0.15*sin(5.0*M_PI/3.0*curr_time), 0.4;
				break;
			case TRAJECTORY3: //yz circle
				obj_des_pos << 0.0, 0.15*sin(5.0*M_PI/3.0*curr_time), 0.4 + 0.15*cos(5.0*M_PI/3.0*curr_time);
				break;
			default:
				obj_des_pos << 0.0, 0.0, 0.4;
				break; }

			// object com task error
			Sai2Model::orientationError(obj_ori_error, Eigen::Matrix3d::Identity(), object_com_frame.linear());
			obj_task_err << (object_com_frame.translation() - obj_des_pos), obj_ori_error;
			
			// desired external wrench and internal forces/moments on the object
			Eigen::VectorXd Fapplied(6);
			Fapplied = augmented_object_inertia*(-kv * object_j * object_model->_dq - kp * obj_task_err) + object_p;
			force_des_vec << Fapplied, -15, 0, 0, 0, 0, 0; //made the tension -15N because students were having bus error with -20
			
			// required end-effector forces to generate the desired object dynamics
			force_ee_vec  = G.lu().solve(force_des_vec);

			// from end-effector forces to required joint torques for each robot
			Eigen::VectorXd force_ee_1(6), force_ee_2(6);
			force_ee_1 << force_ee_vec(0), force_ee_vec(1), force_ee_vec(2), force_ee_vec(6), force_ee_vec(7), force_ee_vec(8); 
			force_ee_2 << force_ee_vec(3), force_ee_vec(4), force_ee_vec(5), force_ee_vec(9), force_ee_vec(10), force_ee_vec(11); 
			
			Eigen::MatrixXd robot1_j0_ee(6, robot1->dof());
			Eigen::MatrixXd robot2_j0_ee(6, robot2->dof());
			robot1->J_0(robot1_j0_ee, ee_link_name, Eigen::Vector3d::Zero());
			robot2->J_0(robot2_j0_ee, ee_link_name, Eigen::Vector3d::Zero());

			tau1 = robot1_j0_ee.transpose()*force_ee_1 + robot1_g;
			tau2 = robot2_j0_ee.transpose()*force_ee_2 + robot2_g;
		
		/* ----------------------------------------------------------------------------------
			Initial grasp stabilization
		-------------------------------------------------------------------------------------*/ 
		} else if (control_mode == CONTROL_GRASP_STABILIZE) { 
			
			Eigen::MatrixXd robot1_j0_ee(6, robot1->dof());
			Eigen::MatrixXd robot2_j0_ee(6, robot2->dof());
			robot1->J_0(robot1_j0_ee, ee_link_name, Eigen::Vector3d::Zero());
			robot2->J_0(robot2_j0_ee, ee_link_name, Eigen::Vector3d::Zero());

			// Joint Torques
			tau1 = robot1_j0_ee.transpose()*(object_p/2) + robot1->_M*(-10.0*robot1->_dq) + robot1_g;
			tau2 = robot2_j0_ee.transpose()*(object_p/2) + robot2->_M*(-10.0*robot2->_dq) + robot2_g;
			
			// Grasp stabilization
			static uint grasp1Counter = 0;
			static uint grasp2Counter = 0;
			if (robot1->_dq[6] < 0.1) {
				grasp1Counter += 1;
			} else {
				grasp1Counter = 0;
			}
			if (robot2->_dq[6] < 0.1) {
				grasp2Counter += 1;
			} else {
				grasp2Counter = 0;
			}
			if (grasp1Counter > 40 && grasp2Counter > 40) {
				cout << " ** Switch Control Mode to Augmented Object Model ** " << endl;
				control_mode = CONTROL_AUGMENTED_OBJECT;
			}
		}

		/* ----------------------------------------------------------------------------------
			Safety torques 
		-------------------------------------------------------------------------------------*/ 
		
		// Set constant gripper forces
		tau1[6] = 15;
		tau2[6] = 15;

        // Default values if torques are exceeded:
        bool fTorqueOverride = false; // to avoid robot blow-ups
        const double tau1_max = 200;
        const double tau2_max = 200;
        if (!fTorqueUseDefaults) {
        	if (tau1.cwiseAbs().maxCoeff() > tau1_max || tau2.cwiseAbs().maxCoeff() > tau2_max) {
	        	fTorqueOverride = true;
	        	cerr << "Torque overriden. User asked torques beyond safe limit: \n";
	        	cerr << "Robot 1: " << tau1.transpose() << "\n";
	        	cerr << "Robot 2: " << tau2.transpose() << "\n";
	        	fTorqueUseDefaults = true;
	        }
	        // Also default values if object is dropped
	        const double object_thickness = 0.05;
	        bool fRobot1DroppedObject = robot1->_q[6] > object_thickness/2;
	        bool fRobot2DroppedObject = robot2->_q[6] > object_thickness/2;
	        if (fRobot1DroppedObject || fRobot2DroppedObject) {
	        	cerr << "Torque overriden. Robot 1 or 2 dropped object. \n";
	        	fTorqueUseDefaults = true;
	        }
        }
        else {
        	robot1->gravityVector(tau1);
			tau1 = tau1 + robot1->_M*(-10.0*robot1->_dq);
			tau1 = (tau1.array() >= tau1_max).select(tau1_max*Eigen::VectorXd::Ones(robot1->dof()), tau1);
			tau1 = (tau1.array() <= -tau1_max).select(-tau1_max*Eigen::VectorXd::Ones(robot1->dof()), tau1);
			robot2->gravityVector(tau2);
			tau2 = tau2 + robot2->_M*(-10.0*robot2->_dq);
			tau2 = (tau2.array() >= tau2_max).select(tau2_max*Eigen::VectorXd::Ones(robot2->dof()), tau2);
			tau2 = (tau2.array() <= -tau2_max).select(-tau2_max*Eigen::VectorXd::Ones(robot2->dof()), tau2);
        }

        /* ----------------------------------------------------------------------------------
			Send robot torques from controller
		-------------------------------------------------------------------------------------*/ 
		sim->setJointTorques(robot1_name, tau1);
		sim->setJointTorques(robot2_name, tau2);
		
		// write object location key out to redis
		redis_client.executeWriteCallback(0);

		// update last time
		last_time = curr_time;
	}
}

/* =======================================================================================
   SIMULATION SETUP
   -----------------------
   * Simulation loop
   * Select trajectory
   * Window initialization
   * Window error
   * Mouse click commands
========================================================================================== */
void simulation(Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(10000); //10000Hz timer

	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs
	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();
		// integrate forward
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time; 
		// sim->integrate(loop_dt);
		sim->integrate(loop_dt);
		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
}
