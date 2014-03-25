




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
    port_name = ros::names::append(ros::this_node::getName(), "ikjoints"); 
  this->ik_joints_publisher_ = this->nh_.advertise<sensor_msgs::JointState>(port_name, 5);
  this->ik_joints_msg.position.resize(7);
  
  // [init subscribers]
  port_name = ros::names::append(ros::this_node::getName(), "joint_states"); 
  this->joint_states_subscriber = this->nh_.subscribe(port_name, 5, &WamIKJ::joint_states_callback, this);
  
  // [init services]
  port_name = ros::names::append(ros::this_node::getName(), "pose_move"); 
  this->pose_move_server = this->nh_.advertiseService(port_name, &WamIKJ::pose_moveCallback, this);
  port_name = ros::names::append(ros::this_node::getName(), "wamik"); 
  this->wamik_server = this->nh_.advertiseService(port_name, &WamIKJ::wamikCallback, this);
  
  // [init clients]
  port_name = ros::names::append(ros::this_node::getName(), "joints_move"); 
  joint_move_client = this->nh_.serviceClient<iri_wam_common_msgs::joints_move>(port_name);
  

  // [init action servers]
  
  // [init action clients]
}

void WamIKJ::ikPub(void)
{
  for (int i=0;i<7;i++){
    this->ik_joints_msg.position[i]=joints_(i);
  }
    this->ik_joints_publisher_.publish(this->ik_joints_msg);
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
	joints_<<-99.9,-99.9,-99.9,-99.9,-99.9,-99.9,-99.9;
      result = false;
  }else{

      ROS_INFO("wamik Service computed joints:\n %f %f %f %f %f %f %f\n", joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5), joints.at(6));
          joints_.resize(7);
	  
      joint_move_srv.request.joints.resize(7);
      for(int i=0;i<7;i++){
        joint_move_srv.request.joints[i] = joints.at(i);
    	joints_(i)=joints.at(i);
      }
      ikPub();

	
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
            joints_.resize(7);
      for(int i=0;i<7;i++){
        res.joints.position[i] = joints.at(i);
	joints_(i)=joints.at(i);
      }
    
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
	int itmax=500;
	double emax=0.0001,lambda=0.00001,eps=0.01;
	

	//INITIAL CONDITIONS
	  MatrixXd T0(4,4);
	  VectorXd cjoints(7),nextangles(7);

	  for(int i=0;i<7;i++)
	    cjoints(i) = currentjoints.at(i);
// 		
// cjoints<<0.8147,0.9058,0.1270,0.9134,0.6324,0.0975,0.2785;
// 

	  for(int i=0;i<16;i++)
	     T0(i/4,i%4) = pose.at(i); 

	//METHOD SELECTION
	int method=1;
	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// 	std::cout<<"method=";
// 	std::cin>>method;
	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	
	
	// STEP SELECTION
//by now, step is alpha=1;
	//VARIABLE INITIALIZATION
	MatrixXd U,V,S;
	int m=7;
        int n=6;
	int it=0;
	bool final=false;
	double mu=0.2,beta0=0.2,nu=10,s0=0.01,gmax=M_PI/5.0,gi,Ni,Mi,sig,lambdajl=0.2;
	MatrixXd Tf,J,I7(7,7);
	I7.setIdentity();

	//Wam_IKAC::fkine(cjoints,Tf,Jn);
	VectorXd errc,aux,gradman,ejl,qnew(7),dq2(7);
	MatrixXd Vi(m,1),Ui(n,1),Ji(n,1);

	VectorXd q(7),dH(7),dH0(7),difH(7),wi(7),w(7),thi,dq,dq1;
	MatrixXd H,Wi(7,7),J0,J1i,JP1i,JP1iaux,J1,P1,Haux(7,7),Jf(n,m),S2(n,m),Ub,Vb,Sb;
	Wi.fill(0.0);
	S2.fill(0.0);

	
	q=cjoints;
	double alpha,alpha2,man;

	dhfunct(q,dH0);

	
	WamIKJ::fkine(q,Tf,J);
	WamIKJ::errortrans(Tf, T0,errc);

	ROS_INFO("STARTING");

std::cout<<"METHOD="<<method<<std::endl;

	while(!final){
	  			 dq.fill(0.0);
				 dq1.fill(0.0);
			  if (method==1){  // JACOBIAN TRANSPOSE
/*			    std::cout<<"-----------------------------------------CHECKPOINT m1\n"<<std::endl;*/
				Ji=J.transpose();
				aux=J*Ji*errc;
				WamIKJ::dotprod(aux,errc,alpha);
				WamIKJ::dotprod(aux,aux,alpha2);
				alpha=alpha/alpha2;
				qnew=q+alpha*Ji*errc;	
			  }else if(method==2){  // JACOBIAN PSEUDOINVERSE
			
/*				std::cout<<"-----------------------------------------CHECKPOINT m2\n"<<std::endl;*/
/*				std::cout<<"J=\n"<<J<<std::endl<<"U=\n"<<U<<std::endl<<"V=\n"<<V<<std::endl<<"S=\n"<<S<<std::endl;*/
				WamIKJ::pseudoinverse(J,Ji,U,V,S);
				qnew=q+Ji*errc;	
			  }else if(method==3){ // MANIPULABILITY GRADIENT PROJECTION
/*				std::cout<<"-----------------------------------------CHECKPOINT m3\n"<<std::endl;*/
				WamIKJ::pseudoinverse(J,Ji,U,V,S);
				WamIKJ::manipgrad(q,man,gradman);
				
				qnew=q+Ji*errc+mu*(I7-Ji*J)*gradman;	
			 }else if(method==4){ // DAMPED JACOBIAN
/*			    std::cout<<"-----------------------------------------CHECKPOINT m4\n"<<std::endl;*/
				WamIKJ::Dpseudoinverse(J,Ji,lambda);
				qnew=q+Ji*errc;
			  }else if(method==5){ // FILTERED JACOBIAN
/*			    std::cout<<"-----------------------------------------CHECKPOINT m5\n"<<std::endl;*/
				WamIKJ::Fpseudoinverse(J,Ji,lambda,eps);
				
				qnew=q+Ji*errc;
			  }else if(method==6){ //WEIGTHED JACOBIAN
/*			      std::cout<<"-----------------------------------------CHECKPOINT m6\n"<<std::endl;*/
				dhfunct(q,dH);
				difH=dH.cwiseAbs()-dH0.cwiseAbs();
				
				for (int ij=1;ij<m;ij++){
				 if (difH(ij)>0){
				  Wi(ij,ij)=sqrt(1.0/(1.0+abs(dH(ij))));
				 }else{
				   Wi(ij,ij)=1.0;
				 }
				}
				 WamIKJ::pseudoinverse(J*Wi,Ji,U,V,S);
				 qnew=q+Wi*Ji*errc;
				
				dH0=dH;
			  }else if(method==7){ // JOINT CLAMPING
/*std::cout<<"-----------------------------------------CHECKPOINT m7\n"<<std::endl;*/
			    WamIKJ::clampjoints(q,H);
			    WamIKJ::pseudoinverse(J*H,Ji,U,V,S);
			    qnew=WamIKJ::pi2piwam(q+H*Ji*errc);
			   

				
			  }else if(method==8){ //JOINT CLAMPING WITH CONTINUOUS ACTIVATION
/*std::cout<<"-----------------------------------------CHECKPOINT m8\n"<<std::endl;*/
			    WamIKJ::HjlCONT(q,H,beta0);
			    WamIKJ::pseudoinverse(J*H,Ji,U,V,S);		    
			    qnew=q+H*Ji*errc;
	
			  }else if(method==9){ // JACOBIAN CONTINUOUS FILTERING
/*std::cout<<"-----------------------------------------CHECKPOINT m9\n"<<std::endl;*/
			    WamIKJ::CFpseudoinverse(J,Ji,nu,s0);
			    qnew=q+Ji*errc;	
			    
			  }else if(method==10){ //SELECTIVELY DAMPED
/*std::cout<<"-----------------------------------------CHECKPOINT m10\n"<<std::endl;*/
			       wi.resize(7);
			       WamIKJ::svdJ(J,U,V,S);
				w.fill(0.0);    
			    for (int i=0;i<m;i++){
					if (i>n-1){
					  wi.fill(0.0);
					  Ni=0.0;
					  sig=0.0;
					}else if (abs(S(i,i))<0.000001){
					  wi.fill(0.0);
					  Ni=0.0;
					  sig=0.0;  
					}else{
					  //define Vi
					  for (int ii=0;ii<m;ii++){
					  Vi(ii,0)=V(ii,i);
					  }  
					  //define Ji and Ui
					  for (int ii2=0;ii2<n;ii2++){
					    Ji(ii2,0)=J(ii2,i);
					    Ui(ii2,0)=U(ii2,i);
					  }
					  // 1/si
					  sig=1/S(i,i);
					  // wi value
					    wi=sig*Vi*Ui.transpose()*errc;
					    Ni=Ui.norm();
					    Mi=0.0;
					    for (int j=0;j<m;j++){
						Mi=Mi+sig*abs(V(j,i))*Ji.norm();			  
					    }
					    // gamma i
					    gi=gmax*min(1.0,Ni/Mi);
					  if (wi.maxCoeff()>gi){
					    thi=gi*wi/wi.maxCoeff(); 
					  }else{
					    thi=wi; 
					  }
					  w=w+thi;
					}
			      }
			      
			      	if (w.maxCoeff()>gmax){
				      dq=gmax*w/w.maxCoeff(); 
				}else{
				      dq=w; 
				}

				    qnew=q+dq;
// 				      std::cout<<"\n w="<<w<<"\n wi="<<wi<<"\n thi"<<thi<<std::endl;
// 				      std::cout<<"-----------------------------------------CHECKPOINT SD4d\n"<<std::endl;
				}else if(method==11){ // TAS PRIORITY CLAMPING
// 				      std::cout<<"-----------------------------------------CHECKPOINT m11\n"<<std::endl;

				    WamIKJ::HjlCONT(q,H,beta0);
				    J0.resize(7,7);
				    J1=I7-H;
				    ejl=-lambdajl*q;
				    WamIKJ::pseudoinverse(J1,J1i,U,V,S);		    
				    P1=I7-J1i*J1;
				    WamIKJ::pseudoinverse(J*P1,JP1i,U,V,S);		    
				    dq=J1i*ejl+JP1i*(errc+J*J1i*ejl);
				    qnew=q+dq;

				  
				  
				}else if(method==12){ // CONTINUOUS TAS PRIORITY CLAMPING
/*std::cout<<"-----------------------------------------CHECKPOINT m12\n"<<std::endl;*/
 
				    WamIKJ::HjlCONT(q,Haux,beta0);
				    J1=I7;
				    H=I7-Haux;
				    J0.resize(7,7);
				    ejl=-lambdajl*q;
				    J1i=H;
				    P1=I7-J1i*J1;
				    JP1iaux=WamIKJ::cpinv(J.transpose(),P1);   
				    JP1i=JP1iaux.transpose();
				    dq1=J1i*ejl;
				    dq=dq1+JP1i*errc-JP1i*J*dq1;
				    qnew=pi2piwam(q+dq);
				  
				}else if(method==13){ // CONTINUOUS TAS PRIORITY CLAMPING and smooth filtering
// std::cout<<"-----------------------------------------CHECKPOINT m13\n"<<std::endl;

				    WamIKJ::svdJ(J,U,V,S);
				      for (int i=0;i<n;i++){
					S2(i,i)=(2*s0+S(i,i)*(2+S(i,i)*(nu+S(i,i))))/(2+S(i,i)*(nu+S(i,i)));
				      }
				    Jf=U*S2*V.transpose();
				    WamIKJ::HjlCONT(q,Haux,beta0);
				    J1=I7;
				    H=I7-Haux;
				    J0.resize(7,7);
				    ejl=-lambdajl*q;
// 				    J1i=WamIKJ::rcpinv(J1,H);
				    J1i=H;
				    P1=I7-J1i*J1;
 				    
				    JP1iaux=WamIKJ::cpinv(J.transpose(),P1);   
// 				    std::cout<<"Jfresca\n"<<JP1iaux<<std::endl;
				    JP1i=JP1iaux.transpose();
// 				    std::cout<<"Jfresca2\n"<<JP1i<<std::endl;
//  				    std::cout<<"-----------------------------------------CHECKPOINT 4\n"<<std::endl;
//     				    std::cout<<"\nejl \n"<< ejl<<"\n errc \n"<< errc <<std::endl;
				    dq1=J1i*ejl;
				    dq=dq1+JP1i*errc-JP1i*J*dq1;
				    
//     				    std::cout<<"qinicial \n"<<q<<"\n H=\n"<<H<<"\n J= \n"<<J<<"\n J1i= \n"<<J1i<<"\n P1= \n"<<P1<<"\n JP1i= \n"<<JP1i<<"\n dq \n"<<dq<<std::endl;
// 				    std::cout<<"-----------------------------------------CHECKPOINT 5\n"<<std::endl;
//    				    std::cout<<"dq1 \n"<<dq1<<"\n JP1i*J*dq1= \n"<<JP1i*J*dq1<<"\n JP1i*errc\n "<<JP1i*errc<<std::endl;

				     
				    qnew=pi2piwam(q+dq);
				  
				}else if(method==14){ // CONTINUOUS TAS PRIORITY CLAMPING selectively damped and smooth filtering
/*std::cout<<"-----------------------------------------CHECKPOINT m14\n"<<std::endl;*/
				    // compute P1
				    WamIKJ::HjlCONT(q,Haux,beta0);
				    J1=I7;
				    H=I7-Haux;
				    J0.resize(7,7);
				    ejl=-lambdajl*q;
				    J1i=H;
				    P1=I7-J1i*J1;
  
				    // compute filtered jacobian: Jf
				    WamIKJ::svdJ(J,U,V,S);
				      for (int i=0;i<n;i++){
					S2(i,i)=(2*s0+S(i,i)*(2+S(i,i)*(nu+S(i,i))))/(2+S(i,i)*(nu+S(i,i)));
				      }
				    Jf=U*S2*V.transpose();

				    // compute J2P1
				    JP1iaux=WamIKJ::cpinv(Jf.transpose(),P1);   
				    JP1i=JP1iaux.transpose();
				    
				    // svd of J2P1
				    WamIKJ::svdJ(JP1i,Vb,Ub,Sb);
				    
				    wi.resize(7);			    
				    w.fill(0.0);    
				    
				    for (int i=0;i<n;i++){
					if (abs(S(i,i))<0.000001){
					  wi.fill(0.0);
					  Ni=1.0;
					  sig=0.0;  
					}else{
					  //define Vi
					  for (int ii=0;ii<m;ii++){
					  Vi(ii,0)=Vb(ii,i);
					  }  
					  //define Ji and Ui
					  for (int ii2=0;ii2<n;ii2++){
					    Ji(ii2,0)=Jf(ii2,i);
					    Ui(ii2,0)=Ub(ii2,i);
					  }
					  // si
					  sig=Sb(i,i);
					  // wi value
					  wi=sig*Vi*Ui.transpose()*errc;
					  Ni=Ui.norm();
					  Mi=0.0;
					  for (int j=0;j<m;j++){
						Mi=Mi+sig*abs(Vb(j,i))*Ji.norm();			  
					  }
					    // gamma i
					  gi=gmax*min(1.0,Ni/Mi);
					  if (wi.maxCoeff()>gi){
					    thi=gi*wi/wi.maxCoeff(); 
					  }else{
					    thi=wi; 
					  }
					  w=w+thi;
					}
				  }
				dq1=J1i*ejl-JP1i*Jf*J1i*ejl;
				dq2=dq1+w;
			      	if (dq2.maxCoeff()>gmax){
				      dq=gmax*dq2/dq2.maxCoeff(); 
				}else{
				      dq=dq2; 
				}
				    
// 				    std::cout<<"Jfresca2\n"<<JP1i<<std::endl;
//  				    std::cout<<"-----------------------------------------CHECKPOINT 4\n"<<std::endl;
//     				    std::cout<<"\nejl \n"<< ejl<<"\n errc \n"<< errc <<std::endl;

				    
//     				    std::cout<<"qinicial \n"<<q<<"\n H=\n"<<H<<"\n J= \n"<<J<<"\n J1i= \n"<<J1i<<"\n P1= \n"<<P1<<"\n JP1i= \n"<<JP1i<<"\n dq \n"<<dq<<std::endl;
// 				    std::cout<<"-----------------------------------------CHECKPOINT 5\n"<<std::endl;
//    				    std::cout<<"dq1 \n"<<dq1<<"\n JP1i*J*dq1= \n"<<JP1i*J*dq1<<"\n JP1i*errc\n "<<JP1i*errc<<std::endl;

				     
				    qnew=pi2piwam(q+dq);
				  
				}
			
	    

	    WamIKJ::fkine(qnew,Tf,J);
	    WamIKJ::errortrans(Tf,T0,errc);
	    q=pi2piwam(qnew);
	    
	    if (std::isnan(errc.norm())){
	    }else{
// 	    std::cout<<"enorm="<<errc.norm()<<std::endl;
	    }
	    it++;
//   std::cout<<"-----------------------------------------CHECKPOINT 3\n"<<std::endl;		    

	    if (errc.norm()<emax || it>itmax){
		    final=true;
	    }


	}
	
std::cout<<"end error= "<<errc.norm()<<", iterations="<< it-1<<std::endl;
// FILE *hola;
/*
  if((hola=fopen("dataM1.txt","w")) == NULL) {
    printf("Cannot open file.\n");
  }*/

// fprintf(hola,"err=%e, joints=",errc.norm());
for(int i=0;i<7;i++){
  joints.at(i) = q(i);
//   fprintf(hola," %e",q(i));
}
// fprintf(hola,"\n");
  
//   return errc.norm()<emax;
  return true;
}




MatrixXd WamIKJ::rcpinv(MatrixXd Q,MatrixXd W){
  MatrixXd Qu,J1,U,V,S;
  WamIKJ::svdJ(W,U,V,S);
  Qu=U.transpose()*Q;
      
  J1=cpinv(Qu,S)*U.transpose(); 
  
// std::cout<<"Q= \n"<<Q<<"\nW\n"<<W<<"\n U\n"<<U<<"\n V \n"<<V<<"\n S \n"<<S<<"\n Qu \n "<<Qu<<"\n W \n"<<W<<"\n cpinv(Qu,S)=\n"<<cpinv(Qu,S)<<"\n Joutrcp \n "<<J1<<std::endl;

  return J1;
}


MatrixXd WamIKJ::lcpinv(MatrixXd Q,MatrixXd W){
  MatrixXd J1;
//   std::cout<<"-----------------------------------------CHECKPOINT a\n"<<std::endl;
  J1=rcpinv(Q.transpose(),W);
// std::cout<<"Qt \n "<<Q.transpose()<<"\n W \n"<<W<<"\n JLout=\n"<<J1<<std::endl;

//   std::cout<<"-----------------------------------------CHECKPOINT b\n"<<std::endl;
  return J1.transpose();
}


MatrixXd WamIKJ::cpinv(MatrixXd J,MatrixXd H){
     int m=J.cols();
     int n=J.rows();
      MatrixXd U,V,S,Ji,J1(m,n),H0(7,7);
      J1.fill(0.0);
      H0.fill(0.0);
      int val;
      MatrixXd hbin(n,1),hi(n,1);
      double produ;
      
      	for (int j=0;j<n;j++){
      	  hi(j,0)=H(j,j);
	}

//       std::cout<<"-----------------------------------------CHECKPOINT cp1\n"<<std::endl;
//  std::cout<<"H=\n" <<H<<"\n hi=["<<hi.transpose()<<"]\n"<<"n, total"<<n<<pow(2,m)<<std::endl;
    for (int i1=0;i1<pow(2,n);i1++){
	hbin.fill(0.0);
	val=i1;
	for (int iaux=n-1;iaux>=0;iaux--){
	    if (val>pow(2,iaux)-1){
		hbin(iaux,0)=1.0;
		val=val-pow(2,iaux);
	    } 
	}
    
	for (int j=0;j<n;j++){
	  H0(j,j)=hbin(j,0); 
	}
    
	produ=1.0;
	for(int i2=0;i2<n;i2++){
	    if (hbin(i2,0)==1.0){
	      produ=produ*hi(i2,0);
	    }else {
	      produ=produ*(1.0-hi(i2,0));
	    }
	}
// 	std::cout<<"-----------------------------------------CHECKPOINT cp2\n"<<"HJ\n"<<H0*J<<std::endl;
	WamIKJ::pseudoinverse(H0*J,Ji,U,V,S);
// 	std::cout<<"-----------------------------------------CHECKPOINT cp3\n"<<std::endl;
	J1=J1+produ*Ji;
// std::cout<<"i1="<<i1<<", hbin= ["<<hbin.transpose()<<"], produ= "<<produ<<std::endl;
 	if (produ>0.0000001){
//   	  std::cout<<"\n H0=\n"<<H0<<"\n J \n"<<J<<"\nH0*J=\n"<<H0*J<<"\n Ji\n"<<Ji<<std::endl;
 	}


    }
//    std::cout<<"JP1i\n"<<J1<<std::endl;
  return J1;
  
}

VectorXd WamIKJ::pi2piwam(VectorXd q){
  MatrixXd Qlim(7,7);
  VectorXd qout(7),q2(7);
  Qlim.resize(7,2);
  Qlim<<-2.6,2.6,-2.0,2.0,-2.8,2.8,-0.9,3.1,-4.8,1.3,-1.6,1.6,-2.2,2.2;

  for (int i=0;i<7;i++){
    q2(i)=min(q(i),Qlim(i,1))-0.00000001;
    qout(i)=max(q2(i),Qlim(i,0))+0.00000001;
  }
  return qout;
}


void WamIKJ::HjlCONT(VectorXd q,MatrixXd& H,double b){
  MatrixXd Qlim,I7(7,7);
  VectorXd beta(7);
  Qlim.resize(7,2);
  Qlim<<-2.6,2.6,-2.0,2.0,-2.8,2.8,-0.9,3.1,-4.8,1.3,-1.6,1.6,-2.2,2.2;
  VectorXd hb(7);
  hb.fill(0.0);
  H.resize(7,7);
  H.fill(0.0);
  I7.fill(0.0);
  for (int i=0;i<7;i++){
	beta(i)=(Qlim(i,1)-Qlim(i,0))*b;

        if ((q(i)<Qlim(i,0)) || (q(i)>Qlim(i,1)) ){
           hb(i)=1.0;
	}else if(q(i)>Qlim(i,0) && q(i)<Qlim(i,0)+beta(i)){
 	    hb(i)=fubeta(beta(i)+Qlim(i,0)-q(i),beta(i));
	}else if(q(i)<Qlim(i,1) && q(i)>Qlim(i,1)-beta(i)){
	    hb(i)=fubeta(beta(i)-Qlim(i,1)+q(i),beta(i));
	}else{
	    hb(i)=0.0;
	}
	H(i,i)=1.0-hb(i);
//  std::cout<<"i="<<i<<"\n min,max= "<<Qlim(i,0)+beta(i)<<" , "<<Qlim(i,1)-beta(i)<<std::endl;
  }
//   std::cout<<"b="<<b<<"\n beta=\n"<<beta<<"\n q=\n"<<q<<"\n hb=\n "<<hb<<std::endl;
}


double WamIKJ::fubeta(double x,double beta){
  double f;
  f=0.5*(1.0+tanh(1.0/(1.0-x/beta)-beta/x)); 
 return f; 
 }

void WamIKJ::dhfunct(VectorXd q,VectorXd& dH){
  MatrixXd Qlim;
  Qlim.resize(7,2);
  Qlim<<-2.6,2.6,-2.0,2.0,-2.8,2.8,-0.9,3.1,-4.8,1.3,-1.6,1.6,-2.2,2.2;
  for (int i=0;i<7;i++){
    dH(i)=1/14*(pow((Qlim(i,1)-Qlim(i,0)),2)*(2*q(i)-Qlim(i,1)-Qlim(i,0))/(pow((Qlim(i,1)-q(i)),2.0)*pow((q(i)-Qlim(i,0)),2.0)));
  }

}


void WamIKJ::clampjoints(VectorXd q,MatrixXd& H){
  MatrixXd Qlim;
  H.resize(7,7);
  H.setIdentity();
  Qlim.resize(7,2);
  Qlim<<-2.6,2.6,-2.0,2.0,-2.8,2.8,-0.9,3.1,-4.8,1.3,-1.6,1.6,-2.2,2.2;
  for (int i=0;i<7;i++){
    if ((q(i)<Qlim(i,0)) || (q(i)>Qlim(i,1)) ){
            H(i,i)=0;
    }
  }

}

void WamIKJ::manipulability(MatrixXd J,double& man){
MatrixXd JJt;
  JJt=J*J.transpose();
  man=sqrt(JJt.determinant());
}

void WamIKJ::manipgrad(VectorXd q,double& man,VectorXd& gradman){
  VectorXd qaux; 
    MatrixXd Tfk,Jn;
    double man0,mani;
    WamIKJ::fkine(q,Tfk,Jn);
    WamIKJ::manipulability(Jn,man0);
  gradman.resize(7);
  gradman.fill(0);
  double h=0.000001;
  for (int i=0;i<7;i++){
  qaux=q;
  qaux(i)+=h;
  WamIKJ::fkine(q,Tfk,Jn);
    WamIKJ::manipulability(Jn,mani);
  gradman(i)=mani-man0;
  
}

}



void WamIKJ::pseudoinverse(MatrixXd J,MatrixXd& Ji,MatrixXd& U,MatrixXd& V,MatrixXd& S){
  int n=J.rows();
  int m=J.cols();
  MatrixXd Si(m,n); 
  Si.fill(0.0);
//  std::cout<<"Si=\n"<<Si<<std::endl<<"files,cols="<<n<<m<<std::endl;

  WamIKJ::svdJ(J,U,V,S);
//std::cout<<"-----------------------------------------fent pinv!\n"<<std::endl;


  for (int i=0;i<min(n,m);i++){
      if (abs(S(i,i))<0.00000001){
// 	      Si(i,i)=0.0; 
// 	     std::cout<<"vap i= "<<i<<" null "<<std::endl;

	}else{
	  Si(i,i)=1/S(i,i); 
// 	  std::cout<<"vap i= "<<i<<"valid= \n"<<Si(i,i)<<std::endl;

	}
      
  }
//   std::cout<<"Si=\n"<<Si<<std::endl;
  Ji=V*Si*U.transpose();
//   std::cout<<"Ji=\n"<<Ji<<std::endl;
}


void WamIKJ::Dpseudoinverse(MatrixXd J,MatrixXd& Ji,double lambda){
  int n=J.rows();
  int m=J.cols();
  MatrixXd U,V,S,Si(m,n); 
  Si.fill(0.0);
  WamIKJ::svdJ(J,U,V,S);
  for (int i=0;i<n;i++){
    if (abs(S(i,i))<0.00000001){
	  Si(i,i)=0.0; 
    }else{
	Si(i,i)=S(i,i)/( pow(S(i,i),2.0)+pow(lambda,2.0)); 
  }
  Ji=V*Si*U.transpose();
  }
}


void WamIKJ::CFpseudoinverse(MatrixXd J,MatrixXd& Ji,double nu,double s0){
  int n=J.rows();
  int m=J.cols();
  MatrixXd U,V,S,S2(n,m),Si(m,n); 
  Si.fill(0.0);
  S2.fill(0.0);
  WamIKJ::svdJ(J,U,V,S);
  for (int i=0;i<n;i++){
    S2(i,i)=(2*s0+S(i,i)*(2+S(i,i)*(nu+S(i,i))))/(2+S(i,i)*(nu+S(i,i)));
    if (abs(S(i,i))>0.00000001){
	Si(i,i)=1/S2(i,i); 	
  }
  Ji=V*Si*U.transpose();
  }
}

void WamIKJ::Fpseudoinverse(MatrixXd J,MatrixXd& Ji,double lambda,double eps){
  int n=J.rows();
  int m=J.cols();
  double lambda2;
  MatrixXd U,V,S,Si(m,n); 
  Si.fill(0.0);
  WamIKJ::svdJ(J,U,V,S);
//     std::cout<<"------------------------checkpoint f0-----------------------"<<std::endl;

  if (S(n-1,n-1)<eps){
      lambda2=lambda*sqrt(1.0-pow(S(n-1,n-1)/eps,2.0));
  }else{
      lambda2=0.0;
  }
    
/*     std::cout<<"------------------------checkpoint f1-----------------------"<<std::endl;*/
 
  for (int i=0;i<n-1;i++){
    if (abs(S(i,i))<0.00000001){
	  Si(i,i)=0.0; 
    }else{
	Si(i,i)=1/S(i,i); 
    }
  }
  
  
//     std::cout<<"------------------------checkpoint f2-----------------------"<<std::endl;

   if (abs(S(n-1,n-1))<0.00000001){
	  Si(n-1,n-1)=0.0; 
    }else{
      Si(n-1,n-1)=S(n-1,n-1)/(pow(S(n-1,n-1),2.0)+pow(lambda2,2.0));
    }

//   std::cout<<"Oldvap="<<S(n-1,n-1)<<"\n Newvap="<<Si(n-1,n-1)<<"\n lambda2="<<lambda2<<"\n aux="<<pow(S(n-1,n-1),2.0)<<std::endl;

  Ji=V*Si*U.transpose();
}


void WamIKJ::svdJ(MatrixXd J, MatrixXd& U,MatrixXd& V,MatrixXd& S){
/*  std::cout<<"Jsvd\n"<<J<<std::endl;*/
// std::cout<<"------------------------COMPUTING SVD-----------------------\n J=\n"<<J<<std::endl;

  int n=J.rows();
  int m=J.cols();
  U.resize(n,n);
  V.resize(m,m);
  S.resize(n,m);
  double maxim=J.maxCoeff();
  double minim=J.minCoeff();
  int maux=min(n,m);
  
  if (abs(maxim)<0.00000001 && abs(minim)<0.00000001){
    U.setIdentity();
    V.setIdentity();
    S.fill(0.0);
  }else{
//   std::cout<<"------------------------checkpoint svd1-----------------------"<<std::endl;

    JacobiSVD<MatrixXd> svd(J,ComputeFullU | ComputeFullV);
// std::cout<<"------------------------checkpoint svd2-----------------------"<<std::endl;
    /*      std::cout<<"S\n"<<S<<std::endl;*/
    VectorXd vaps;
    int n=J.rows();
    int m=J.cols();
    U=svd.matrixU();
    V=svd.matrixV();
    vaps=svd.singularValues();
    S.resize(n,m);
    S.fill(0.0);
   
    for (int i=0;i<maux;i++){
//               std::cout<<"------------------------checkpoint svd8-----------------------\n i "<<i<<std::endl;
	  S(i,i)=vaps(i); 
    }
	
  }  
  
// std::cout<<"J\n"<<J<<"\n U=\n"<<U<<"\n V= \n"<<V<<"\n S=\n"<<S<<"\n -------------------------SVD DONE-------------------\n";
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
    ros::Rate loop_rate(10); 
    while(ros::ok()){
      ros::spinOnce();
      loop_rate.sleep(); 
    }
}
