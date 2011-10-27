




// UNCOMPLETE CODE!!!! DO NOT USE!!!





#include "wam_ikj.h"
using namespace Eigen;
using namespace std;

WamIKJ::WamIKJ() {
  //init class attributes if necessary
  //this->loop_rate = 2;//in [Hz]

  this->currentjoints.resize(7);

  //string for port names
  std::string port_name;

  // [init publishers]
  
  // [init subscribers]
  port_name = ros::names::append(ros::this_node::getName(), "joint_states"); 
  this->joint_states_subscriber = this->nh_.subscribe(port_name, 5, &WamIKJ::joint_states_callback, this);
  
  // [init services]
  port_name = ros::names::append(ros::this_node::getName(), "pose_move"); 
  this->pose_move_server = this->nh_.advertiseService(port_name, &WamIKJ::pose_moveCallback, this);
  port_name = ros::names::append(ros::this_node::getName(), "wamik"); 
  this->wamik_server = this->nh_.advertiseService(port_name, &WamIKJ::wamikCallback, this);
  
  // [init clients]
  port_name = ros::names::append(ros::this_node::getName(), "joint_move"); 
  joint_move_client = this->nh_.serviceClient<iri_wam_common_msgs::joints_move>(port_name);
  

  // [init action servers]
  
  // [init action clients]
}

/*  [subscriber callbacks] */
void WamIKJ::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg) { 

  for(int i=0;i<7;i++)
    currentjoints[i] = msg->position[i]; 

}

/*  [service callbacks] */
bool WamIKJ::pose_moveCallback(iri_wam_common_msgs::pose_move::Request &req, iri_wam_common_msgs::pose_move::Response &res) 
{ 

  bool result;
  Quaternion<float> quat( req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z);
  Matrix3f mat = quat.toRotationMatrix();
      ROS_INFO("Received Quat: %f %f %f %f %f %f %f\n",
                req.pose.position.x, req.pose.position.y, req.pose.position.z, 
                req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w);

  std::vector <double> pose(16,0);
  std::vector <double> joints(7,0);

  pose[3] = req.pose.position.x;
  pose[7] = req.pose.position.y;
  pose[11] = req.pose.position.z;
  pose[15] = 1;
  for(int i=0; i<12; i++){
   if(i%4 != 3){
     pose[i] = mat(i/4,i%4);
   }
  }
  ROS_INFO("wamik Service Received Pose:\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n",
        pose[0],pose[1],pose[2],pose[3],
        pose[4],pose[5],pose[6],pose[7],
        pose[8],pose[9],pose[10],pose[11],
        pose[12],pose[13],pose[14],pose[15]);

  if(!WamIKJ::ik(pose, currentjoints, joints)){
      ROS_ERROR("IK solution not found. Requested pose:\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n",
        pose[0],pose[1],pose[2],pose[3],
        pose[4],pose[5],pose[6],pose[7],
        pose[8],pose[9],pose[10],pose[11],
        pose[12],pose[13],pose[14],pose[15]);
      result = false;
  }else{

      ROS_INFO("wamik Service computed joints:\n %f %f %f %f %f %f %f\n", joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5), joints.at(6));
    
      joint_move_srv.request.joints.resize(7);
      for(int i=0;i<7;i++)
        joint_move_srv.request.joints[i] = joints.at(i);
    
      if (this->joint_move_client.call(joint_move_srv)) 
      { 
        ROS_INFO(" %d\n",joint_move_srv.response.success); 
        result = true;
      } 
      else 
      { 
        ROS_ERROR("Failed to call service joint_move"); 
        result = false;
      }
  }
  res.success = result;
  return result;
}

bool WamIKJ::wamikCallback(iri_wam_common_msgs::wamInverseKinematics::Request &req, iri_wam_common_msgs::wamInverseKinematics::Response &res){
  bool result;
  Quaternion<float> quat(req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w);
  Matrix3f mat = quat.toRotationMatrix();

  std::vector <double> pose(16,0);
  std::vector <double> joints(7,0);

  pose[3] = req.pose.position.x;
  pose[7] = req.pose.position.y;
  pose[11] = req.pose.position.z;
  pose[15] = 1;
  for(int i=0; i<12; i++){
   if(i%4 != 3){
     pose[i] = mat(i/4,i%4);
   }
  }
  ROS_INFO("wamik Service Received Pose:\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n",
        pose[0],pose[1],pose[2],pose[3],
        pose[4],pose[5],pose[6],pose[7],
        pose[8],pose[9],pose[10],pose[11],
        pose[12],pose[13],pose[14],pose[15]);

  if(!WamIKJ::ik(pose, currentjoints, joints)){
      ROS_ERROR("IK solution not found");
      result = false;
  }else{

      ROS_INFO("wamik Service computed joints:\n %f %f %f %f %f %f %f\n", joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5), joints.at(6));
    
      res.joints.position.resize(7);
      for(int i=0;i<7;i++)
        res.joints.position[i] = joints.at(i);
    
  }
  return result;
}

/*  [action callbacks] */

/*  [action requests] */





bool WamIKJ::ik(vector<double> pose, vector<double> currentjoints, vector<double>& joints){
    //ik solver




	MatrixXd Qlim(7,2);
	Qlim<<-2.6,2.6,-2.0,2.0,-2.8,2.8,-0.9,3.1,-4.8,1.3,-1.6,1.6,-2.2,2.2;

	//PARAMETERS
	int itmax=1000;
	double emax=0.0001;

	//INITIAL CONDITIONS
	  MatrixXd T0(4,4);
	  VectorXd cjoints(7),nextangles(7);

	  for(int i=0;i<7;i++)
	     cjoints(i) = currentjoints.at(i);

	  for(int i=0;i<16;i++)
	     T0(i/4,i%4) = pose.at(i); 

	//METHOD SELECTION
	int method=2;
	
	// STEP SELECTION
//by now, step is alpha=1;
	//VARIABLE INITIALIZATION
	MatrixXd U,V,S;
	int m=7;
        int n=6;
	int it=0;
	bool final=false;
	double mu=0.2;
	MatrixXd Tf,Ji,J;
	//Wam_IKAC::fkine(cjoints,Tf,Jn);
	VectorXd errc,aux,qnew(7);
	VectorXd q(7);
	q=cjoints;
	double alpha,alpha2;

	WamIKJ::fkine(q,Tf,J);
	WamIKJ::errortrans(Tf, T0,errc);
// std::cout<<"-----------------------------------------CHECKPOINT 3\n"<<std::endl;
	ROS_INFO("STARTING");

	while(!final){
		switch (method) {

			  case 1 : 
				// Jacobian Transpose
				Ji=J.transpose();
				aux=J*Ji*errc;
				WamIKJ::dotprod(aux,errc,alpha);
				WamIKJ::dotprod(aux,aux,alpha2);
				alpha=alpha/alpha2;
				qnew=q+alpha*Ji*errc;	

			  case 2 : 
				// Jacobian pseudoinverse
/*				std::cout<<"J=\n"<<J<<std::endl<<"U=\n"<<U<<std::endl<<"V=\n"<<V<<std::endl<<"S=\n"<<S<<std::endl;*/
				WamIKJ::pseudoinverse(J,Ji);
				qnew=q+Ji*errc;	

			  case 3 :
				// Gradient projection
				WamIKJ::pseudoinverse(J,Ji);
				
				
				
			  default : 
				// Jacobian Transpose
				Ji=J.transpose();
				aux=J*Ji*errc;
				WamIKJ::dotprod(aux,errc,alpha);
				WamIKJ::dotprod(aux,aux,alpha2);
				alpha=alpha/alpha2;
				qnew=q+alpha*Ji*errc;	
		}
	WamIKJ::fkine(qnew,Tf,J);
	WamIKJ::errortrans(Tf,T0,errc);
	q=qnew;
	std::cout<<"enorm="<<errc.norm()<<std::endl;

	it++;
  
	if (errc.norm()<emax || it>itmax){
		final=true;
	}


	}
	
	
	
for(int i=0;i<7;i++)
 joints.at(i) = q(i);
return true;
}



void WamIKJ::manipulability(MatrixXd J,double& man){
MatrixXd JJt;
  JJt=J*J.transpose();
  man=sqrt(JJt.determinant());
}


void WamIKJ::manipgrad(MatrixXd J,double& man){
MatrixXd JJt;
  JJt=J*J.transpose();
  man=sqrt(JJt.determinant());
}



void WamIKJ::pseudoinverse(MatrixXd J,MatrixXd& Ji){
  int n=J.rows();
  int m=J.cols();
  MatrixXd U,V,S,Si(m,n); 
  Si.fill(0.0);
  WamIKJ::svdJ(J,U,V,S);
  for (int i=0;i<n;i++){
   Si(i,i)=1/S(i,i); 
  }
  Ji=V*Si*U.transpose();
}




void WamIKJ::svdJ(MatrixXd J, MatrixXd& U,MatrixXd& V,MatrixXd& S){
 JacobiSVD<MatrixXd> svd(J,ComputeFullU | ComputeFullV);
VectorXd vaps;
 int n=J.rows();
 int m=J.cols();
 
 U=svd.matrixU();
 V=svd.matrixV();
 vaps=svd.singularValues();
 S.resize(n,m);
 S.fill(0.0);
 
 for (int i=0;i<n;i++){
  if (abs(vaps(i))<0.00000001){
    S(i,i)=0.0; 
  }else{
    S(i,i)=vaps(i); 
 }
  
  
}

void WamIKJ::fkine(VectorXd theta,MatrixXd& Tfk,MatrixXd& Jn){
	int n=6;
	int m=7;	
	MatrixXd T_aux(4,4),T_aux2(4,4),R(3,3),J(n,m),MJ(n,n);
	
	Tfk.resize(4,4);
	Tfk.setIdentity();

	MatrixXd DHcoeffs(7,3);
	DHcoeffs<<-M_PI/2,0.,0.,M_PI/2,0.,0.,-M_PI/2,0.045,0.55,M_PI/2,-0.045,0.,-M_PI/2,0.,0.3,M_PI/2,0.,0.,0.,0.,0.06;
	for (int j=m-1;j>=0;j--){
		DHmatrix(DHcoeffs(j,0), DHcoeffs(j,1), DHcoeffs(j,2),theta(j),T_aux);
		T_aux2=T_aux*Tfk;
		Tfk = T_aux2;
		J(0,j)=-Tfk(0,0)*Tfk(1,3)+Tfk(1,0)*Tfk(0,3);
		J(1,j)=-Tfk(0,1)*Tfk(1,3)+Tfk(1,1)*Tfk(0,3);
		J(2,j)=-Tfk(0,2)*Tfk(1,3)+Tfk(1,2)*Tfk(0,3);
		J(3,j)=Tfk(2,0);
		J(4,j)=Tfk(2,1);
		J(5,j)=Tfk(2,2);

	}
	R.resize(3,3);
	R=Tfk.block<3,3>(0,0);
	MJ.resize(n,n);
	MJ.fill(0.0);
	MJ.block<3,3>(0,0)=R;
	MJ.block<3,3>(3,3)=R;
	Jn=MJ*J;
}

void WamIKJ::dotprod(MatrixXd u,MatrixXd v,double& p){
	p=0.0;
	if ((v.rows()==1 && u.rows()==1) && u.cols()==v.cols()){
		for(int i=0;i<u.cols();i++){
			p+=u(0,i)*v(0,i);	
		}
	}else if((v.cols()==1 && u.cols()==1) && u.rows()==v.rows()){
		for(int i=0;i<u.rows();i++){
			p+=u(i,0)*v(i,0);	
		}
	}else{
	std::cout<<"Error when calculating dot product"<<std::endl;
	}
}



MatrixXd WamIKJ::crossprod(MatrixXd u,MatrixXd v){

MatrixXd w(3,1);
w(0,0)=u(1,0)*v(2,0)-u(2,0)*v(1,0);
w(1,0)=u(2,0)*v(0,0)-u(0,0)*v(2,0);
w(2,0)=u(0,0)*v(1,0)-u(1,0)*v(0,0);
return w;
}


void WamIKJ::errortrans(MatrixXd Tcurrent, MatrixXd Tdesired, VectorXd& errc){
  
	errc.resize(6);
	MatrixXd d1(3,1),d2(3,1),d3(3,1),c1(3,1),c2(3,1),c3(3,1),xd(3,1),xc(3,1),aux(3,1);
	
	d1=Tdesired.block<3,1>(0,0);
	d2=Tdesired.block<3,1>(0,1);
	d3=Tdesired.block<3,1>(0,2);
	xd=Tdesired.block<3,1>(0,3);

	c1=Tcurrent.block<3,1>(0,0);
	c2=Tcurrent.block<3,1>(0,1);
	c3=Tcurrent.block<3,1>(0,2);
	xc=Tcurrent.block<3,1>(0,3);

	errc(0)=xd(0)-xc(0);
	errc(1)=xd(1)-xc(1);
	errc(2)=xd(2)-xc(2);
	
	aux=0.5*(crossprod(c1,d1)+crossprod(c2,d2)+crossprod(c3,d3));

	errc(3)=aux(0,0);
	errc(4)=aux(1,0);
	errc(5)=aux(2,0);
}


void WamIKJ::filtersols(MatrixXd qsol,VectorXd& sol){
// INPUT: qsol, Nx7 MatrixXd, where each row is a solution of the WAM inverse kinematics, not concerning joint limits
// OUPUT: the same Nx7 matrix, plus a N-dimentional VectorXd "sol", whose ith value is 1 if the ith row of qsol respects joint limits, 0 otherwise
	MatrixXd Qlim(7,2);
	Qlim << -2.6, 2.6,
		-2.0, 2.0,
		-2.8, 2.8,
		-0.9, 3.1,
		-4.76,1.24,
		-1.6, 1.6,
		-2.2, 2.2;
	sol.resize(qsol.rows());
	sol.fill(1.0);

	for (int i=0;i<qsol.rows();i++){
		for (int j=0;j<7;j++){
			if ((qsol(i,j)<Qlim(j,0))|(qsol(i,j)>Qlim(j,1))){
				sol(i)=0.0;
	
			}	
		}

	}
	
	
}

void WamIKJ::DHmatrix(double alpha, double a, double d,double& theta, MatrixXd& T){
// INPUT: alpha, a, d Denavit Hartenberg parameters, and theta, the rotation angle of a wam link (standard notation).
// OUTPUT: T, the D-H matrix (MatrixXd) calculated with those parameters and theta.
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



void WamIKJ::skewop(Matrix3d& M,Vector3d v){
M.fill(0.0);
M(0,1)=-v(2);
M(1,0)=v(2);
M(0,2)=v(1);
M(2,0)=-v(1);
M(1,2)=-v(0);
M(2,1)=v(0);
}



void WamIKJ::potentialfunction(VectorXd qref,VectorXd& q, double& potq){
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





void WamIKJ::sphericalikine(MatrixXd T0,MatrixXd T4, Vector2d& q5,Vector2d& q6,Vector2d& q7){
	MatrixXd Taux;
	Taux=T4.inverse()*T0;
	q5(0)=atan(Taux(1,2)/Taux(0,2));

	if (q5(0)>1.3-3.14159265){
	q5(1)=q5(0)-3.14159265;
	}else{
	q5(1)=q5(0)+3.14159265;
	}

	for (int i5=0;i5<2;i5++){
	q6(i5)=atan2(Taux(0,2)*cos(q5(i5))+Taux(1,2)*sin(q5(i5)),Taux(2,2));
	q7(i5)=atan2(-Taux(0,0)*sin(q5(i5))+Taux(1,0)*cos(q5(i5)),-Taux(0,1)*sin(q5(i5))+Taux(1,1)*cos(q5(i5)));
	}
}




void WamIKJ::soltrig(double a, double b, double c, MatrixXd& q2sol2){
/*solves the equation a = b*sin(x)+c*cos(x) */
//std::cout<<"b"<<b<<std::endl;
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
    WamIKJ wam_ikj;
    ros::spin();
}
