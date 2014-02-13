// BETA VERSION, STILL BEING TESTED, BUT APPARENTLY ROBUST AND EFFICIENT.

#include "kuka_ikac.h"
using namespace Eigen;
using namespace std;

KukaIKAC::KukaIKAC() {
  //init class attributes if necessary
  //this->loop_rate = 2;//in [Hz]

  this->currentjoints_.resize(7);

  //string for port names
  std::string port_name;

  // [init publishers]
  port_name = ros::names::append(ros::this_node::getName(), "ikjoints"); 
  this->ik_joints_publisher_ = this->nh_.advertise<sensor_msgs::JointState>(port_name, 5);
  this->ik_joints_msg.position.resize(7);
  
  // [init subscribers]
  port_name = ros::names::append(ros::this_node::getName(), "joint_states"); 
  this->joint_states_subscriber = this->nh_.subscribe(port_name, 5, &KukaIKAC::joint_states_callback, this);
  
  // [init services]
  port_name = ros::names::append(ros::this_node::getName(), "wamik"); 
  this->kukaik_server = this->nh_.advertiseService(port_name, &KukaIKAC::kukaikCallback, this);
  
  port_name = ros::names::append(ros::this_node::getName(), "wamik_from_pose");
  this->kukaik_server_fromPose = this->nh_.advertiseService(port_name, &KukaIKAC::kukaikCallbackFromPose, this);

  // [init clients]
  port_name = ros::names::append(ros::this_node::getName(), "joints_move"); 
  joint_move_client = this->nh_.serviceClient<iri_wam_common_msgs::joints_move>(port_name);
  
  // [init action servers]
  
  // [init action clients] 
 
  // [init variables]

/* KUKA LIGHTWEIGHT ROBOT INVERSE KINEMATICS. This code works apparently correct without crashes. But please test it carefully to ensure 
its robustness and also that the parameters introduced are correct (if not, the solution given by this algorithm would not be correct.*/

  double PI=3.14159265;
  // links offsets. d7 is the TCP offset. Arbitrarly set to 0.1.
  d1=0.3100;
  d3=0.4000;
  d5=0.3900;
  d7=0.0785; // Tool center point offset. This should be modified with the correct value!!!
  
  // for debugging purposes, if checksol_=1, the forward kinematics is computed to check if the solution has 0 error.
  checksol_=1;
  
  //DH PARAMETERS - STANDARD CONVENTION (not CRAIG's convention!!)
  DHparam_.resize(7,3);  
  DHparam_ << 0.0,  PI/2.0, d1,
	            0.0, -PI/2.0, 0.0,
	            0.0, -PI/2.0, d3,
	            0.0,  PI/2.0, 0.0,
	            0.0,  PI/2.0, d5,
	            0.0, -PI/2.0, 0.0,
	            0.0, 0.0, d7; 
	    
  // JOINT LIMITS---> TO BE MODIFIED!! note that if joints 3,5 or 7 have their limits out of the [-pi,pi] range, the function pi2pi should be modified.
  Qlim_.resize(7,2);
  Qlim_ << -2.97, 2.97,
	  -0.47, 3.49,
	  -2.97, 2.97,
	  -2.09, 2.09,
	  -2.97, 2.97,
	  -2.09, 2.09,
	  -2.97, 2.97;
	  
  // STEP INDICATES THE VARIATION MAGNITUDE APPLIED TO JOINT 1 TO FIND THE FIRST SOLUTION
  step_ = 0.0005;
  
  // when a first solution is found, the elbow is rotated around the wrist-shoulder axis so as to obtain more solutions. These rotations
  //have a magnitude of 2*pi/IMAX_.
  IMAX_ = 100;
  
}

void KukaIKAC::ikPub(void)
{
  for (int i=0;i<7;i++){
    this->ik_joints_msg.position[i]=joints_(i);
  }
    this->ik_joints_publisher_.publish(this->ik_joints_msg);
}

/*  [subscriber callbacks] */
void KukaIKAC::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg) { 

  for(int i=0;i<7;i++)
    currentjoints_[i] = msg->position[i]; 

}

/*  [service callbacks] */
bool KukaIKAC::kukaikCallbackFromPose(iri_wam_common_msgs::wamInverseKinematicsFromPose::Request &req, iri_wam_common_msgs::wamInverseKinematicsFromPose::Response &res){

  ROS_INFO("[KukaIKAC] User Given Current Joints %s (j1, j2, j3, j4, j5, j6, j7): [ %f, %f, %f, %f, %f, %f, %f ]",
            req.current_joints.header.frame_id.c_str(), 
            req.current_joints.position[0], 
            req.current_joints.position[1],
            req.current_joints.position[2],
            req.current_joints.position[3],
            req.current_joints.position[4],
            req.current_joints.position[5],
            req.current_joints.position[6]);
  
  ROS_INFO("[KukaIKAC] Received Pose from frame_id %s (x, y, z, qx, qy, qz, qw): [ %f, %f, %f, %f, %f, %f, %f ]",
            req.desired_pose.header.frame_id.c_str(), 
            req.desired_pose.pose.position.x, 
            req.desired_pose.pose.position.y,
            req.desired_pose.pose.position.z,
            req.desired_pose.pose.orientation.x,
            req.desired_pose.pose.orientation.y,
            req.desired_pose.pose.orientation.z,
            req.desired_pose.pose.orientation.w);
  
  // User defined current pose
  std::vector<double> currentjoints;
  for(int ii=0; ii<7; ii++)
    currentjoints[ii] = req.current_joints.position[ii]; 

  Quaternion<float> quat( req.desired_pose.pose.orientation.w, req.desired_pose.pose.orientation.x, req.desired_pose.pose.orientation.y, req.desired_pose.pose.orientation.z);
  Matrix3f mat = quat.toRotationMatrix();

  std::vector <double> desired_pose(16,0);
  std::vector <double> desired_joints(7,0);

  // Desired pose as a Homogeneous Transformation
  desired_pose[3] = req.desired_pose.pose.position.x;
  desired_pose[7] = req.desired_pose.pose.position.y;
  desired_pose[11] = req.desired_pose.pose.position.z;
  desired_pose[15] = 1;
  for(int i=0; i<12; i++){
   if(i%4 != 3){
     desired_pose[i] = mat(i/4,i%4);
   }
  }
  ROS_INFO("[KukaIKAC] Received HRt:\n [ \t%f %f %f %f\n\t %f %f %f %f\n\t %f %f %f %f\n\t %f %f %f %f\n",
            desired_pose[0], desired_pose[1], desired_pose[2], desired_pose[3],
            desired_pose[4], desired_pose[5], desired_pose[6], desired_pose[7],
            desired_pose[8], desired_pose[9], desired_pose[10], desired_pose[11],
            desired_pose[12], desired_pose[13], desired_pose[14], desired_pose[15]);

  if(!KukaIKAC::ik(desired_pose, currentjoints, desired_joints)){
      ROS_ERROR("[KukaIKAC] IK solution not found. Is pose out of configuration space?");
      return false;
  }else{

      ROS_INFO("[KukaIKAC] Service computed joints:\n %f %f %f %f %f %f %f\n", desired_joints.at(0), desired_joints.at(1), desired_joints.at(2), desired_joints.at(3), desired_joints.at(4), desired_joints.at(5), desired_joints.at(6));
    
      res.desired_joints.position.resize(7);
      joints_.resize(7);
      for(int i=0; i<7; i++){
        res.desired_joints.position[i] = desired_joints.at(i);
        joints_(i) = desired_joints.at(i);
      }
  }
  return true;
}

bool KukaIKAC::kukaikCallback(iri_wam_common_msgs::wamInverseKinematics::Request &req, iri_wam_common_msgs::wamInverseKinematics::Response &res){

  ROS_INFO("[KukaIKAC] Received Pose from frame_id %s (x, y, z, qx, qy, qz, qw): [ %f, %f, %f, %f, %f, %f, %f ]",
            req.pose.header.frame_id.c_str(), 
            req.pose.pose.position.x, 
            req.pose.pose.position.y,
            req.pose.pose.position.z,
            req.pose.pose.orientation.x,
            req.pose.pose.orientation.y,
            req.pose.pose.orientation.z,
            req.pose.pose.orientation.w);
  
  Quaternion<float> quat( req.pose.pose.orientation.w, req.pose.pose.orientation.x, req.pose.pose.orientation.y, req.pose.pose.orientation.z);
  Matrix3f mat = quat.toRotationMatrix();

  std::vector <double> pose(16,0);
  std::vector <double> joints(7,0);

  pose[3] = req.pose.pose.position.x;
  pose[7] = req.pose.pose.position.y;
  pose[11] = req.pose.pose.position.z;
  pose[15] = 1;
  for(int i=0; i<12; i++){
   if(i%4 != 3){
     pose[i] = mat(i/4,i%4);
   }
  }
  ROS_INFO("[KukaIKAC] Received HRt:\n [ \t%f %f %f %f\n\t %f %f %f %f\n\t %f %f %f %f\n\t %f %f %f %f\n",
            pose[0],pose[1],pose[2],pose[3],
            pose[4],pose[5],pose[6],pose[7],
            pose[8],pose[9],pose[10],pose[11],
            pose[12],pose[13],pose[14],pose[15]);

  if(!KukaIKAC::ik(pose, currentjoints_, joints)){
      ROS_ERROR("[KukaIKAC] IK solution not found. Is pose out of configuration space?");
      return false;
  }else{

      ROS_INFO("[KukaIKAC] Service computed joints:\n %f %f %f %f %f %f %f\n", joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5), joints.at(6));
    
      res.joints.position.resize(7);
      joints_.resize(7);
      for(int i=0;i<7;i++){
        res.joints.position[i] = joints.at(i);
        joints_(i)=joints.at(i);
      }
      res.joints.position[1] = joints.at(1)+1.57079;
      joints_(1) = joints.at(1)+1.57079;
  }
  return true;
}

/*  [action callbacks] */

/*  [action requests] */


bool KukaIKAC::ik(vector<double> pose, vector<double> currentjoints, vector<double>& joints){
    //ik solver: pose is the desired homogeneous transformation, formated as a 16x1 vector:[1st row + 2nd row + 3rd row + 4th row]
    //currentjoints is the current joints vector, and joints is the output of the inverse kinematics.
    // joints is the output vector, the solution chosen

/* Convert current joints and goal to eigen vector structure*/
  MatrixXd T0(4,4);
  VectorXd cjoints(7), nextangles(7), qoptim(7);
  for(int i=0;i<16;i++)
     T0(i/4,i%4) = pose.at(i); 

  for(int i=0;i<7;i++)
     cjoints(i) = currentjoints.at(i);

	joints.resize(7);
	MatrixXd A(4,4);
	// first solution loop variables
	double i=0;
	bool initfound=false;
	bool breakloop=false;
	// variables to store solutions and auxiliar variables
	MatrixXd qsol(8,7);
	VectorXd sol(8);	
	qsol.fill(0.0);
	sol.fill(0.0);
	double q10=cjoints(0);
	MatrixXd qaux;
	Vector3f Xw;
	// we initialise the potential value of the solutions with a very high value
	double potq=100000.;
	
	printf("Solution found is that which minimizes a weighted norm of the joints variation, to insert another criterion, modify the function KukaIKAC::potentialfunction \n");
	printf("Variation on q0 at each step for obtaining first solution : %e", step_);
	printf(" \n Initial joints:\n %f %f %f %f %f %f %f\n", currentjoints.at(0), currentjoints.at(1), currentjoints.at(2), currentjoints.at(3), currentjoints.at(4), currentjoints.at(5), currentjoints.at(6));

	
	/* Look for the initial solution */
	while (!initfound & !breakloop){
		/* q10 is the joint 1 value used to find a solution. Its first value is the current joint 1 value.*/
    KukaIKAC::getq1(cjoints,i,q10);
		
		if (q10<Qlim_(0,1) && q10>Qlim_(0,0)){
		/* qsol is the solution matrix (up to 8 solutions) found with q10, sol is a vector indicating which solutions are valid
		T0 is the objective homogeneous transformation*/
		      KukaIKAC::exactikine(T0,q10,cjoints,qsol,sol);
		      /* we eliminate solutions which are not valid*/
		      KukaIKAC::filtersols(qsol,sol);
		      if (sol.sum()!=0.){ /* if there is a valid solution, we break the loop, and we take representatives of each elbow-shoulder
			 configuration (what we call basic solutions). These solutions are then to be rotated around the wrist-shoulder axis*/
			      KukaIKAC::basicsols(qsol,qaux,sol);
			      initfound=true;
		      }
		}else{
		  if (i*step_+cjoints(0)>Qlim_(0,1) && -i*step_+cjoints(0)<Qlim_(0,0)){
		    breakloop=true; // if we get out of joint 1 range without having found a solution, we stop to avoid infinite loop
		    printf("LOOP BREAK,q1 steps OUT OF RANGE \n");
		  }
		}	
		i=i+1.;
	}



	if (initfound){

		/* If we found a first solution, we perform optimization with the above mentioned rotations*/
		KukaIKAC::optimizesol(qaux,T0,qoptim,potq,cjoints);	
		  joints.clear();
		  joints.resize(7);
		  for(int i=0;i<7;i++)
		     joints[i] = qoptim(i);
		  
		  std::cout<<"\n best solution is: \n"<<qoptim.transpose();


		if (checksol_==1){
		  MatrixXd T01,T12,T23,T34,T45,T56,T67,T07;
		  KukaIKAC::DHmatrix(DHparam_(0,1),DHparam_(0,0),DHparam_(0,2),qoptim(0),T01),
		  KukaIKAC::DHmatrix(DHparam_(1,1),DHparam_(1,0),DHparam_(1,2),qoptim(1),T12),
		  KukaIKAC::DHmatrix(DHparam_(2,1),DHparam_(2,0),DHparam_(2,2),qoptim(2),T23),
		  KukaIKAC::DHmatrix(DHparam_(3,1),DHparam_(3,0),DHparam_(3,2),qoptim(3),T34);
		  KukaIKAC::DHmatrix(DHparam_(4,1),DHparam_(4,0),DHparam_(4,2),qoptim(4),T45);
		  KukaIKAC::DHmatrix(DHparam_(5,1),DHparam_(5,0),DHparam_(5,2),qoptim(5),T56);
		  KukaIKAC::DHmatrix(DHparam_(6,1),DHparam_(6,0),DHparam_(6,2),qoptim(6),T67);
		  T07=T01*T12*T23*T34*T45*T56*T67;
		  std::cout<<"\n Result matrix error T0-Tout= \n"<<T0-T07<<std::endl;
		}


		return true;
	} else{
		  for(int i=0;i<7;i++)
		     joints[i] = cjoints(i);
		printf("SOLUTION NOT FOUND, robot not moving \n ");
		return false;
	}    


}





void KukaIKAC::filtersols(MatrixXd qsol,VectorXd& sol){
// INPUT: qsol, Nx7 MatrixXd, where each row is a solution of the WAM inverse kinematics, not concerning joint limits
// OUPUT: the same Nx7 matrix, plus a N-dimentional VectorXd "sol", whose ith value is 1 if the ith row of qsol respects joint limits, 0 otherwise
	sol.resize(qsol.rows());
	sol.fill(1.0);
	for (int i=0;i<qsol.rows();i++){
		for (int j=0;j<7;j++){
			if ((qsol(i,j)<Qlim_(j,0))|(qsol(i,j)>Qlim_(j,1))){
				sol(i)=0.0;
			}	
		}
	}
}

void KukaIKAC::DHmatrix(double alpha, double a, double d,double& theta, MatrixXd& T){
// INPUT: alpha, a, d Denavit Hartenberg parameters, and theta, the rotation angle of a wam link (standard notation, not craig!!).
// OUTPUT: T, the D-H matrix (MatrixXd format) calculated with those parameters and theta.
T.resize(4,4);
T.fill(0.0);
T(0,0)= cos(theta); 
T(0,1)=-sin(theta)*cos(alpha);
T(0,2)=sin(theta)*sin(alpha);
T(0,3)=a*cos(theta);
T(1,0)=sin(theta);
T(1,1)=cos(theta)*cos(alpha); 
T(1,2)=-cos(theta)*sin(alpha); 
T(1,3)=a*sin(theta); 
T(2,1)=sin(alpha); 
T(2,2)=cos(alpha);
T(2,3)=d; 
T(3,3)=1;

}



void KukaIKAC::exactikine(MatrixXd T0, double q10, VectorXd cjoints, MatrixXd& qsol,VectorXd&sol){
/* this function computes analitically the 8 solutions of the inverse kinematics of the robot, with a fixed value of the first joint (treating the
robot as a 6-dof robot.*/
// INPUT: T0=MatrixXd, the desired homogeneous transformation.
//	  q10=double, a fixed value for the first joint.
// OUTPUT: qsol=MatrixXd, a 8x7 matrix, each row corresponds to a solution of the inverse kinematics for the desired goal.
//	   sol=vector, indicating wether solution has been found or not for the fixed q1. Note it does not consider joint limits!

	double q30=cjoints(2);
	double c4,aux1,aux2,aux3,signq4;
	double PI=3.14159265;
	qsol.resize(8,7);
	MatrixXd q4sol4(2,2);
	MatrixXd q2sol2(2,2);
	MatrixXd Pw(4,1),Pwaux(3,1);
	MatrixXd T12,T23,T34,T01,T04;
	VectorXd q3(2);
	Vector2d q5;
	Vector2d q6;
	Vector2d q7;

	MatrixXd qout(8,7);
	qout.fill(0.0);
	int k;

	/* We compute the wrist position, and an auxiliar wrist position, due to the offset d1*/
	Pw=T0.block<4,1>(0,3)-d7*T0.block<4,1>(0,2);
	Pwaux=Pw.block<3,1>(0,0);
	Pwaux(2,0)=Pw(2,0)-d1;
	
	/*Computing elbow joint with cosinus theorem*/
	c4=(Pwaux.squaredNorm()-pow(d3,2.0)-pow(d5,2.0))/(2*d3*d5);
	if (abs(c4)<=1.0){
	  q4sol4(0,1)=1.0;
	  q4sol4(1,1)=1.0;
	  q4sol4(0,0)=acos(c4);
	  q4sol4(1,0)=-acos(c4);
	}else{
	  q4sol4.fill(0.0);
	  q4sol4(0,0)=-99.9999;
	  q4sol4(1,0)=-99.9999;
	}
	
	KukaIKAC::DHmatrix(DHparam_(0,1),DHparam_(0,0),DHparam_(0,2),q10,T01);
		for (int i4=0;i4<2;i4++){ /*for each joint 4 solution, we compute the rest of the angles. To do so, we use the equations
		  obtained by writing T12^(-1)·T01^(-1)·Pw=T23·T34·Pw4 (Pw4=wrist in frame 4: [0 0 d5 1]')*/

			aux1=c4*d5+d3;
			aux2=-cos(q10)*Pw(0,0)-sin(q10)*Pw(1,0);
			aux3=Pw(2,0)-d1;	
			KukaIKAC::soltrig(aux1, aux2, aux3, q2sol2);
			KukaIKAC::DHmatrix(DHparam_(3,1),DHparam_(3,0),DHparam_(3,2),q4sol4(i4,0),T34);
			signq4=q4sol4(i4,0)/abs(q4sol4(i4,0));
				for(int i2=0;i2<2;i2++){
					if (abs(cos(q4sol4(i4,0))-1.0)>0.000001){
					  q3(i2)=pi2pi(0.5*PI*(-signq4+1.0)+atan2(-sin(q10)*Pw(0,0)+cos(q10)*Pw(1,0),sin(q10)*cos(q2sol2(i2,0))*Pw(1,0)+cos(q10)*cos(q2sol2(i2,0))*Pw(0,0)+sin(q2sol2(i2,0))*(Pw(2,0)-d1)));
					}else{ //then the elbow is in a straight line, we do not vary the joint 3 configuration
					  q3(i2)=q30;		  
					}
					KukaIKAC::DHmatrix(DHparam_(1,1),DHparam_(1,0),DHparam_(1,2),q2sol2(i2,0),T12);
					KukaIKAC::DHmatrix(DHparam_(2,1),DHparam_(2,0),DHparam_(2,2),q3(i2),T23);
					T04=T01*T12*T23*T34;
					KukaIKAC::sphericalikine(T0,T04,cjoints,q5,q6,q7);
					/*We write all the posible solutions in a 8x7 matrix, each row of it is a solution set. the vector sol indicates wether solutions have been found*/
					for (int i5=0;i5<2;i5++){
						k=i4*4+i2*2+i5;
						qsol(k,0)=q10;
						qsol(k,1)=q2sol2(i2,0);
						qsol(k,2)=q3(i2);
						qsol(k,3)=q4sol4(i4,0);
						qsol(k,4)=q5(i5);
						qsol(k,5)=q6(i5);
						qsol(k,6)=q7(i5);
						sol(k)=q4sol4(i4,1)*q2sol2(i2,1);
					}
				}
		}
}



void KukaIKAC::basicsols(MatrixXd qsol,MatrixXd& qaux,VectorXd sol){
// INPUT:  qsol=a 8x7 matrix (MatrixXd), each row corresponds to a solution of the inverse kinematics for the desired goal.
//	   sol=vector, indicating wether solution has been found or not for the fixed q1. Suposed to consider joint limits.
	   
// OUTPUT: qaux=a Nx7 matrix (MatrixXd), each row corresponds to a solution of the inverse kinematics for the desired goal of those in qsol. With the 
// 	consideration that qaux only has one row for each pair of q2, q4 which gives a solution of the inverse kinematics for a given q1 
// 	(note there will usualy be 2 solutions for the spherical wrist for each of these couples {q2,q4}

	MatrixXd qauxiliar2;
	qauxiliar2.resize(4,7);
	qauxiliar2.fill(-99.9999);
	int nbasics=0;
	for (int i=0;i<qsol.rows()/2;i++){
		if (sol(2*i)>0.0){
			qauxiliar2.row(nbasics)=qsol.row(2*i);
			nbasics=nbasics+1;

		}else if (sol(2*i+1)>0.0){
			qauxiliar2.row(nbasics)=qsol.row(2*i+1);
			nbasics=nbasics+1;
		}
	}
	qaux.resize(nbasics,7);

	for (int j=0;j<nbasics;j++){
		qaux.row(j)=qauxiliar2.row(j);
	}
}





void KukaIKAC::optimizesol(MatrixXd q, MatrixXd T0, VectorXd& qoutput,double& potact,VectorXd qref){
// INPUT q=qaux in basicsol function.
//	 T0=desired goal
// OUPUT qoutput=the best solution, according to a desired optimization function, obtained by rotating the basic solutions around Origin-Wrist axis.
// 	 potact=the value of the potential function for the solution chosen.
	/* q is the initial guess*/
	VectorXd PJa,qelbows;
	MatrixXd PJ(3,q.rows()); //PJ is the elbow cartesian point. each column corresponds to one basic solution

	printf("Rotations for each solution: %d",IMAX_);

	for (int ia=0;ia<q.rows();ia++){
		qelbows=q.row(ia);
		KukaIKAC::initialelbows(qelbows,T0,PJa); // we compute the initial elbows values
		PJ.col(ia)=PJa;
	}
	
	double L=d7;
	MatrixXd T04;
	Matrix3d Mrot,Rn,I3;
	double phi,qn4,qn3,qn2,qn1,eps1,eps3,potq;
	Vector2d qn5,qn6,qn7;
	double PI=3.14159265;
	Vector3d v,CJ;
	VectorXd qn4b,sol,qbest;
	MatrixXd Pw,Pw2,Paux,T01,T12,T23,T34,Qaux;
	MatrixXd Qglobal(4*q.rows(),7);	
	qoutput=q.row(0);
	VectorXd offshoulder(3,1);
	offshoulder << 0.0,0.0,d1;
	Pw=T0.block<3,1>(0,3)-L*T0.block<3,1>(0,2);
	Pw2=T0.block<3,1>(0,3)-L*T0.block<3,1>(0,2)-offshoulder;
	double aux0=Pw2.norm();
	v=Pw2/aux0;	
	/*Compute q4*/
	KukaIKAC::skewop(Mrot,v);
	I3.setIdentity();
	double signq4;
	// we perform rotations around the wrist-shoulder(intersection of joints 2 and 3 axis) line
	for (int i=0;i<IMAX_;i++){
		phi=2*PI*double(i)/double(IMAX_)*pow(-1,double(i));
		Rn=I3+sin(phi)*Mrot+(1-cos(phi))*Mrot*Mrot;

		Qglobal.fill(-9.9);
		for (int nconfig=0;nconfig<q.rows();nconfig++){
			qn4=q(nconfig,3);
			Paux=PJ.block<3,1>(0,nconfig);
			Paux(2)=Paux(2)-d1;
			CJ=Rn*Paux;
			CJ(2)=CJ(2)+d1;
			Qaux.resize(4,7);
			Qaux.fill(-9.9);

				/*Calculacte q2*/
				qn2=atan2(CJ(2)-d1,sqrt(pow(CJ(0),2.0)+pow(CJ(1),2.0)))-PI/2;
				/*Calculate q1*/
				qn1=atan2(CJ(1),CJ(0));
				/*Calculate q3*/
				signq4=qn4/abs(qn4);
				qn3=pi2pi(0.5*PI*(-signq4+1.0)+atan2(-sin(qn1)*Pw(0,0)+cos(qn1)*Pw(1,0),sin(qn1)*cos(qn2)*Pw(1,0)+cos(qn1)*cos(qn2)*Pw(0,0)+sin(qn2)*(Pw(2,0)-d1)));
				KukaIKAC::DHmatrix(DHparam_(0,1),DHparam_(0,0),DHparam_(0,2),qn1,T01);
				KukaIKAC::DHmatrix(DHparam_(1,1),DHparam_(1,0),DHparam_(1,2),qn2,T12);
				KukaIKAC::DHmatrix(DHparam_(2,1),DHparam_(2,0),DHparam_(2,2),qn3,T23);
				KukaIKAC::DHmatrix(DHparam_(3,1),DHparam_(3,0),DHparam_(3,2),qn4,T34);
				T04=T01*T12*T23*T34;
				
				KukaIKAC::sphericalikine(T0,T04,qref,qn5,qn6,qn7);
				eps1 = copysign(1.0, qn1);
				eps3 = copysign(1.0, qn3);			
				Qaux<<qn1,qn2,qn3,qn4,qn5(0),qn6(0),qn7(0), //we obtain up to 4 solutions for each rotated point, which we add to the Qglobal matrix
					qn1,qn2,qn3,qn4,qn5(1),qn6(1),qn7(1),
					qn1-eps1*PI,-qn2,qn3-eps3*PI,qn4,qn5(0),qn6(0),qn7(0),
					qn1-eps1*PI,-qn2,qn3-eps3*PI,qn4,qn5(1),qn6(1),qn7(1);


			Qglobal.row(4*nconfig)=Qaux.row(0);
			Qglobal.row(4*nconfig+1)=Qaux.row(1);
			Qglobal.row(4*nconfig+2)=Qaux.row(2);
			Qglobal.row(4*nconfig+3)=Qaux.row(3);
		}
		// when we have all the possible solutions obtained from the basic solutions, we filter those which are valid
		KukaIKAC::filtersols(Qglobal,sol);
		// and we choose the best one according to the potential defined
		KukaIKAC::bestsol(Qglobal,sol,qref,qbest,potq);

		// if the potential found is better than the best we had, we update the solution:
		if (potq<potact){
			qoutput=qbest;
			potact=potq;
//			std::cout<<"\n i="<<i<<", potact="<<potq<<"\n qoutput="<<qbest.transpose();
		}
	}
}




void KukaIKAC::skewop(Matrix3d& M,Vector3d v){ //skew-simetric matrix of a vector
M.fill(0.0);
M(0,1)=-v(2);
M(1,0)=v(2);
M(0,2)=v(1);
M(2,0)=-v(1);
M(1,2)=-v(0);
M(2,1)=v(0);
}

void KukaIKAC::getq1(VectorXd cjoints,float i,double& q10){
q10=cjoints(0)+i*step_*pow(-1,i); 	
}

double KukaIKAC::pi2pi(double x){ //[-Pi,Pi] centering value of an angle
  double PI=3.14159265; 
 while (x>PI || x<-PI){
    if (x>PI){
      x=x-2*PI;
    }
    if (x<-PI){
      x=x+2*PI;
    } 
  }
return x;
}



void KukaIKAC::bestsol(MatrixXd qsol,VectorXd sol,VectorXd qref,VectorXd& q,double& potbest){
  //returns the best solution of a matrix of solutions (each row being a solution), according to the potentialfunction.
	potbest=100000; /*high initial potential value*/
	double potq;
	VectorXd qact;
	for(int i=0;i<qsol.rows();i++){
		if (sol(i)!=0.0){
			qact=qsol.block<1,7>(i,0);
			KukaIKAC::potentialfunction(qref,qact,potq);
			if (potq<potbest){
				q=qact;
				potbest=potq;
			}
		}
	}
}




void KukaIKAC::potentialfunction(VectorXd qref,VectorXd& q, double& potq){
  
// ------------------IT IS VERY RECOMMENDED TO MODIFY THIS FUNCTION ACCORDING TO THE NEEDS OF THE USER---------------------
// if you need help at making this function, do not hesitate to ask us!
// INPUT: qref: taken as the current joint state of the robot
//	  q: the joint state q to be evaluated	
// OUTPUT:   potq: the potential of the joint position q




	MatrixXd W(7,7);
	W.fill(0.0);
	W(0,0)=2.5;
	W(1,1)=2.0;
	W(2,2)=2.0;
	W(3,3)=1.5;
	W(4,4)=1.0;
	W(5,5)=1.0;
	W(6,6)=1.0;

	VectorXd qdif(7);
	
	qdif=q-qref;
	potq=qdif.transpose()*W*qdif;

}





void KukaIKAC::sphericalikine(MatrixXd T0,MatrixXd T4, VectorXd cjoints, Vector2d& q5,Vector2d& q6,Vector2d& q7){
	// This function solves the inverse kinematics of a spherical wrist like the kuka lightweight arm.
  
  MatrixXd Taux;
	Taux=T4.inverse()*T0;
	
	double c6,s6,q57;
	c6=Taux(2,2);
	double	pi=3.14159265;
	if (abs(c6)<0.99999){
	 q7(0)=atan2(-Taux(2,1),Taux(2,0));
	 q5(0)=atan2(-Taux(1,2),-Taux(0,2));
	 s6=Taux(0,3)/(-cos(q5(0))*d7);
	 q6(0)=atan2(s6,c6);
	 q5(1)=pi2pi(q5(0)+pi);
	 q6(1)=-q6(0);	
	 q7(1)=pi2pi(q7(0)+pi);
	}else if(abs(c6<1.0) && abs(c6)>0.99999){
	 //WRIST SINGULARITY - we keep q5 constant and we move q7.
	 q57=atan2(Taux(1,0),Taux(0,0));
	 q5(0)=cjoints(4);
	 q7(0)=q57-q5(0);	
	 q6(0)=0.0;
	 q5(1)=pi2pi(q5(0)+pi);
	 q6(1)=0.0;	
	 q7(1)=pi2pi(q7(0)+pi);
	}else{
	 q5(0)=-99.9999;
	 q5(1)=-99.9999;
	 q6(0)=-99.9999;
	 q6(1)=-99.9999;
	 q7(0)=-99.9999;
	 q7(1)=-99.9999;
	 std::cout<<"\n no solution spherical---------------------\n";
	}

}



void KukaIKAC::initialelbows(VectorXd q,MatrixXd T0,VectorXd& PJ){
	MatrixXd T01,T12,T23,T02,T03;
	KukaIKAC::DHmatrix(DHparam_(0,1),DHparam_(0,0),DHparam_(0,2),q(0),T01);
	KukaIKAC::DHmatrix(DHparam_(1,1),DHparam_(1,0),DHparam_(1,2),q(1),T12);
	KukaIKAC::DHmatrix(DHparam_(2,1),DHparam_(2,0),DHparam_(2,2),q(2),T23);
	T02=T01*T12;
	T03=T02*T23;
	VectorXd aux(4),PLaux(4);
	aux(0)=0.0;
	aux(1)=0.0;
	aux(2)=d3;
	aux(3)=1.0;
	PLaux=T02*aux;
	PJ=PLaux.block<3,1>(0,0);

}



void KukaIKAC::soltrig(double a, double b, double c, MatrixXd& q2sol2){
/*solves the equation a = b*sin(x)+c*cos(x), the output is a 2x2 matrix, with its first column containing the 2 solutions and the second column
an indicator of valid solution or not*/
	if (abs(b)<0.00000001 && abs(a/c)<1.0){
		q2sol2(0,1)=1.;
		q2sol2(1,1)=1.;
		q2sol2(0,0)=acos(a/c);
		q2sol2(1,0)=-acos(a/c);
    

	}else if (pow(c,2.)*pow(b,2)-pow(b,2.)*pow(a,2.)+pow(b,4.)<=0.00000001){
	  
		q2sol2(0,1)=0.;
		q2sol2(1,1)=0.;
		q2sol2(0,0)=-99.9999;
		q2sol2(1,0)=-99.9999;

	        
	}else{
	
		q2sol2(0,1)=1.;
		q2sol2(1,1)=1.;
		q2sol2(0,0)=atan2(-(-a+c*(a*c+sqrt(pow(c,2.)*pow(b,2.)-pow(b,2.)*pow(a,2.)+pow(b,4.)))/(pow(c,2.)+pow(b,2.)))/b,(a*c+sqrt(pow(c,2.)*pow(b,2.)-pow(b,2.)*pow(a,2.)+pow(b,4.)))/(pow(c,2.)+pow(b,2.)));
		q2sol2(1,0)=atan2(-(-a-c*(-a*c+sqrt(pow(c,2.)*pow(b,2.)-pow(b,2.)*pow(a,2.)+pow(b,4.)))/(pow(c,2.)+pow(b,2.)))/b,-(-a*c+sqrt(pow(c,2.)*pow(b,2.)-pow(b,2.)*pow(a,2.)+pow(b,4.)))/(pow(c,2.)+pow(b,2.)));
	}
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "wam_ik");
    KukaIKAC kuka_ikac;
    ros::Rate loop_rate(10);
    while(ros::ok()){
      ros::spinOnce();
      loop_rate.sleep(); 
    }

}
