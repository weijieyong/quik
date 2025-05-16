/**
 * @file sample_cpp_usage.cpp
 * @author Steffan Lloyd (steffan.lloyd@nibio.no)
 * @brief Demo code on how to define a robot, and call the forward and inverse kinematics functions
 * in pure C++ (outside of ros). This code can be compiled in ROS but does not depend on it in
 * any way.
 * @date 2024-11-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <iostream>
#include <chrono>
#include "Eigen/Dense"
#include <math.h>
#include "quik/geometry.hpp"
#include "quik/Robot.hpp"
#include "quik/IKSolver.hpp"

using namespace std;
using namespace Eigen;

// Define manipulator.
// This is the DH parameters for the KUKA KR6 robot
// auto R = std::make_shared<quik::Robot<6>>(
// 	// Given as DOFx4 table, in the following order: a_i, alpha_i, d_i, theta_i.
// 	(Matrix<double, 6, 4>() <<
// 		0.025,    -M_PI/2,   0.183,       0,
// 		-0.315,   0,         0,           0,
// 		-0.035,   M_PI/2,    0,           0,
// 		0,        -M_PI/2,   0.365,       0,
// 		0,        M_PI/2,    0,           0,
// 		0,        0,         0.08,        0).finished(),

// 	// Second argument is a list of joint types
// 	// true is prismatic, false is revolute
// 	// KUKA KR6 only has revolute joints
// 	(Vector<quik::JOINTTYPE_t,6>() <<
//         quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE
//     ).finished(),

// 	// Third agument is a list of joint directions
// 	// Allows you to change the sign (direction) of
// 	// the joints.
// 	(Vector<double,6>(6) << 1, 1, 1, 1, 1, 1).finished(),

// 	// Fourth and fifth arguments are the base and tool transforms, respectively
// 	Matrix4d::Identity(4,4),
// 	Matrix4d::Identity(4,4)
// );

// Define UR5 manipulator with calibration deltas applied
// Base parameters from urcontrol.conf.UR5 + deltas from calibration.conf
auto R = std::make_shared<quik::Robot<6>>(
    // Given as DOFx4 table, in the following order: a_i, alpha_i, d_i, theta_i.
    (Matrix<double, 6, 4>() <<
        // a + delta_a,             alpha + delta_alpha,                 d + delta_d,                              theta + delta_theta
        0.0 + 0.000143412481775751, 1.570796327 + 0.000261142842141071,  0.1625 + 0.000229096222625319035,        0.0 + 2.63213121159955321e-08,
        -0.425 + 0.0111645466532434, 0.0 + 0.000478915073877239075,      0.0 - 204.077105406530421,               0.0 - 0.231923745541985582,
        -0.3922 - 7.29306429188004e-05, 0.0 + 0.00738088439942797669,    0.0 + 203.392323630684871,               0.0 + 6.50222213863359411,
        0.0 + 0.00019069275578740042, 1.570796327 + 0.000252176196238407, 0.1333 + 0.685273448748187963,           0.0 + 0.0128800872653601212,
        0.0 + 1.53107557356551335e-06, -1.570796327 + 0.00184534917359324702, 0.0997 - 0.000125249700421090071,    0.0 - 6.75796482454971148e-08,
        0.0 + 0.0,                    0.0 + 0.0,                         0.0996 - 0.000130827480128062335,         0.0 - 7.09899853634093914e-07
    ).finished(),

    // Second argument is a list of joint types
    // true is prismatic, false is revolute
    // UR5 only has revolute joints
    (Vector<quik::JOINTTYPE_t,6>() <<
        quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE
    ).finished(),

    // Third argument is a list of joint directions from urcontrol.conf.UR5
    (Vector<double,6>(6) << 1, 1, 1, 1, 1, 1).finished(),

    // Fourth and fifth arguments are the base and tool transforms, respectively
    Matrix4d::Identity(4,4),
    Matrix4d::Identity(4,4)
);

// Define the IK options
const quik::IKSolver<6> IKS(
    R, // The robot object (pointer)
    200, // max number of iterations
    quik::ALGORITHM_QUIK, // algorithm (ALGORITHM_QUIK, ALGORITHM_NR or ALGORITHM_BFGS)
    1e-12, // Exit tolerance
    1e-14, // Minimum step tolerance
    0.05, // iteration-to-iteration improvement tolerance (0.05 = 5% relative improvement)
    10, // max consequitive gradient fails
    80, // Max gradient fails
    1e-10, // lambda2 (lambda^2, the damping parameter for DQuIK and DNR)
    0.34, // Max linear error step
    1 // Max angular error step
);

int main()
{
	
	// Initilize variables
	int N = 10; // Number of poses to generate
	int DOF = R->dof;
	Matrix<double,6,Dynamic>    Q(DOF, N),	    // True joint angles
                                Q0(DOF, N),	    // Initial guess of joint angles
                                Q_star(DOF, N);	// Solver's solution
	Matrix<double,6,Dynamic>    e_star(6,N);	// Error at solver pose
    std::vector<int>            iter(N);	    // Store number of iterations of algorithm
	std::vector<quik::BREAKREASON_t>  breakReason(N);	// Store break out reason
	Matrix4d                    T,		        // True forward kinematics transform
		                        T_star;	        // Forward kinematics at solver solution
	
	Matrix<double,Dynamic,4>    Tn(N*4,4); 	    // 4N * 4 matrix of vertically stacked transforms to be solved.
							                    // This is just a convenient way of sending in an array of transforms.
	
	// Generate some random joint configurations for the robot
	Q.setRandom(DOF, N);
	
	// Log out the random joint angles
	cout << "\n=== Random Joint Angles Generated ===\n";
	for (int i = 0; i < N; i++) {
		cout << "Config " << i+1 << ": [";
		for (int j = 0; j < DOF; j++) {
			cout << Q(j, i) * 180.0 / M_PI;
			if (j < DOF-1) cout << ", ";
		}
		cout << "]\n";
	}
	cout << "===================================\n\n";

	// Perturb true answers slightly to get initial "guess" (knock over by 0.1 radians)
	// Q0 = Q.array() + 0.1;
	// Generate random initial guesses instead of fixed offsets
	Q0.setRandom(DOF, N); // Completely random initial guesses
	
	// Log out the initial guesses
	cout << "\n=== Random Initial Guesses ===\n";
	for (int i = 0; i < N; i++) {
		cout << "Initial guess " << i+1 << ": [";
		for (int j = 0; j < DOF; j++) {
			cout << Q0(j, i) * 180.0 / M_PI;
			if (j < DOF-1) cout << ", ";
		}
		cout << "]\n";
	}
	cout << "===================================\n\n";
		
	// Do forward kinematics of each Q sample and store in the "tall" matrix
	// of transforms, Tn
	cout << "\n=== Forward Kinematics Poses ===\n";
	for (int i = 0; i < N; i++){
		R->FKn(Q.col(i), T);
		Tn.middleRows<4>(i*4) = T;
		
		// Log out the pose
		cout << "Pose " << i+1 << " from config " << i+1 << ":\n";
		cout << "Position: [" << T(0,3) << ", " << T(1,3) << ", " << T(2,3) << "]\n";
		
		// Extract rotation in a readable format (rotation vector)
		AngleAxisd axisAngle(T.block<3,3>(0,0));
		Vector3d rotVector = axisAngle.axis() * axisAngle.angle();
		cout << "Orientation (rotation vector): [" << rotVector(0) << ", " 
		     << rotVector(1) << ", " << rotVector(2) << "] "
		     << "(magnitude: " << rotVector.norm() << " rad = " 
		     << rotVector.norm() * 180.0 / M_PI << " deg)\n";
		cout << "Transform Matrix:\n" << T << "\n\n";
	}
	cout << "==============================\n\n";

	R->print();

	cout << "==============================\n\n";

	// Start a timer, to get time
	auto startTime = chrono::high_resolution_clock::now();

	// Solve using the IK function, store results in Q_star, e_star, iter and breakreason
	IKS.IK(Tn, Q0, Q_star, e_star, iter, breakReason);

	// Get the time after calling the IK function
    chrono::duration<double, std::micro> elapsed = chrono::high_resolution_clock::now() - startTime;
	
	// Print out results
	cout << "Joint angles (start):" << endl;
	cout << Q0 << endl << endl;
	cout << "Joint angles (true):" << endl;
	cout << Q << endl << endl;
	cout << "The final joint angles are: " << endl;
	cout << Q_star << endl << endl;
	cout << "Final normed error is: " << endl << e_star.array().square().colwise().sum().sqrt() << endl << endl;
	cout << "Break reason is: " << endl;
    for (const auto& reason : breakReason) cout << reason << ' ';
    cout << endl;
	cout << "Number of iterations: " << endl;
    for (const auto& iter_i : iter) cout << iter_i << ' ';
    cout << endl;
	cout << "Time elapsed for IK function: " << elapsed.count() << " us, " << elapsed.count()/N << " us per sample." << endl;
	cout << "Program finished!" << endl;
	
	return 0;
}