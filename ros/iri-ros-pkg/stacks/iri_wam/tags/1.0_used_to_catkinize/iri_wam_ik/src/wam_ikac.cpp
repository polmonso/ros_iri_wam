

// BETA VERSION, STILL BEING TESTED, BUT APPARENTLY ROBUST AND EFFICIENT.


#include "wam_ikac.h"
using namespace Eigen;
using namespace std;

WamIKAC::WamIKAC() {
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
    this->joint_states_subscriber = this->nh_.subscribe(port_name, 5, &WamIKAC::joint_states_callback, this);

    // [init services]
    port_name = ros::names::append(ros::this_node::getName(), "wamik"); 
    this->wamik_server = this->nh_.advertiseService(port_name, &WamIKAC::wamikCallback, this);

    port_name = ros::names::append(ros::this_node::getName(), "wamik_from_pose");
    this->wamik_server_fromPose = this->nh_.advertiseService(port_name, &WamIKAC::wamikCallbackFromPose, this);

    // [init clients]
    port_name = ros::names::append(ros::this_node::getName(), "joints_move"); 
    joint_move_client = this->nh_.serviceClient<iri_wam_common_msgs::joints_move>(port_name);

    // [init action servers]

    // [init action clients] 

}

void WamIKAC::ikPub(void)
{
    for (int i=0;i<7;i++){
        this->ik_joints_msg.position[i]=joints_(i);
    }
    this->ik_joints_publisher_.publish(this->ik_joints_msg);
}

/*  [subscriber callbacks] */
void WamIKAC::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg) { 

    for(int i=0;i<7;i++)
        currentjoints_[i] = msg->position[i]; 

}

/*  [service callbacks] */
bool WamIKAC::wamikCallbackFromPose(iri_wam_common_msgs::wamInverseKinematicsFromPose::Request &req, iri_wam_common_msgs::wamInverseKinematicsFromPose::Response &res){

    ROS_INFO("[WamIKAC] User Given Current Joints %s (j1, j2, j3, j4, j5, j6, j7): [ %f, %f, %f, %f, %f, %f, %f ]",
            req.current_joints.header.frame_id.c_str(), 
            req.current_joints.position[0], 
            req.current_joints.position[1],
            req.current_joints.position[2],
            req.current_joints.position[3],
            req.current_joints.position[4],
            req.current_joints.position[5],
            req.current_joints.position[6]);

    ROS_INFO("[WamIKAC] Received Pose from frame_id %s (x, y, z, qx, qy, qz, qw): [ %f, %f, %f, %f, %f, %f, %f ]",
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
    ROS_INFO("[WamIKAC] Received HRt:\n [ \t%f %f %f %f\n\t %f %f %f %f\n\t %f %f %f %f\n\t %f %f %f %f\n",
            desired_pose[0], desired_pose[1], desired_pose[2], desired_pose[3],
            desired_pose[4], desired_pose[5], desired_pose[6], desired_pose[7],
            desired_pose[8], desired_pose[9], desired_pose[10], desired_pose[11],
            desired_pose[12], desired_pose[13], desired_pose[14], desired_pose[15]);

    if(!WamIKAC::ik(desired_pose, currentjoints, desired_joints)){
        ROS_ERROR("[WamIKAC] IK solution not found. Is pose out of configuration space?");
        return false;
    }else{

        ROS_INFO("[WamIKAC] Service computed joints:\n %f %f %f %f %f %f %f\n", desired_joints.at(0), desired_joints.at(1), desired_joints.at(2), desired_joints.at(3), desired_joints.at(4), desired_joints.at(5), desired_joints.at(6));

        res.desired_joints.position.resize(7);
        joints_.resize(7);
        for(int i=0; i<7; i++){
            res.desired_joints.position[i] = desired_joints.at(i);
            joints_(i) = desired_joints.at(i);
        }
    }
    return true;
}

bool WamIKAC::wamikCallback(iri_wam_common_msgs::wamInverseKinematics::Request &req, iri_wam_common_msgs::wamInverseKinematics::Response &res){

    ROS_INFO("[WamIKAC] Received Pose from frame_id %s (x, y, z, qx, qy, qz, qw): [ %f, %f, %f, %f, %f, %f, %f ]",
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
    ROS_INFO("[WamIKAC] Received HRt:\n [ \t%f %f %f %f\n\t %f %f %f %f\n\t %f %f %f %f\n\t %f %f %f %f\n",
            pose[0],pose[1],pose[2],pose[3],
            pose[4],pose[5],pose[6],pose[7],
            pose[8],pose[9],pose[10],pose[11],
            pose[12],pose[13],pose[14],pose[15]);

    if(!WamIKAC::ik(pose, currentjoints_, joints)){
        ROS_ERROR("[WamIKAC] IK solution not found. Is pose out of configuration space?");
        return false;
    }else{

        ROS_INFO("[WamIKAC] Service computed joints:\n %f %f %f %f %f %f %f\n", joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5), joints.at(6));

        res.joints.position.resize(7);
        joints_.resize(7);
        for(int i=0;i<7;i++){
            res.joints.position[i] = joints.at(i);
            joints_(i)=joints.at(i);
        }
    }
    return true;
}

/*  [action callbacks] */

/*  [action requests] */


bool WamIKAC::ik(vector<double> pose, vector<double> currentjoints, vector<double>& joints){
    //ik solver

    MatrixXd posec(4,4);
    VectorXd cjoints(7),nextangles(7),qoptim(7);
    for(int i=0;i<16;i++)
        posec(i/4,i%4) = pose.at(i); 

    /* Convert current joints to a eigen vector structure*/
    for(int i=0;i<7;i++)
        cjoints(i) = currentjoints.at(i);

    clock_t start,end;
    start=clock();
    joints.resize(7);

    MatrixXd A(4,4);
    double i=0;
    bool initfound=false;
    bool breakloop=false;

    clock_t t1,t2,t3;
    t1=clock();

    /*T0 is the objective position*/
    MatrixXd T0(4,4);
    T0=posec;
    MatrixXd qsol(8,7);
    VectorXd sol(8);	
    qsol.fill(0.0);
    sol.fill(0.0);
    double q10=cjoints(0);
    double step(0.0);
    MatrixXd qaux;
    Vector3f Xw;

    double potq=10000.;
    //int basicelements=4;
    step=0.0005;
    ROS_DEBUG("WAM_IKAC: Beta Version, still being tested but apparently working correctly");
    ROS_DEBUG("Solution found is that which minimizes a weighted norm of the joints variation, to insert another criterion, modify the function WamIKAC::potentialfunction");
    ROS_DEBUG("Variation on q0 at each step for obtaining first solution : %e",step);
    ROS_DEBUG("Initial joints:\n %f %f %f %f %f %f %f\n", currentjoints.at(0), currentjoints.at(1), currentjoints.at(2), currentjoints.at(3), currentjoints.at(4), currentjoints.at(5), currentjoints.at(6));
    //std::cout<<"q1 iteration step:\n"<<step<<std::endl;	
    //std::cin>>step;
    /* Look for the initial solution */

    //q10=-2.44;
    //	WamIKAC::exactikine(T0,q10,qsol,sol);
    //	WamIKAC::filtersols(qsol,sol);
    //std::cout<<"q10="<<q10<<std::endl<<"qsol=\n"<<qsol<<std::endl<<"sol \n"<<sol.transpose()<<std::endl;	


    while (!initfound & !breakloop){
        /* q10 is the joint 1 value used to find a solution*/
        WamIKAC::getq1(cjoints, i, q10, step);

        if (abs(q10)<2.6){
            //std::cout<<"provant q10="<<q10<<std::endl;
            /* qsol is the solution matrix (up to 8 solutions) found with q10, sol is a vector indicating which solutions are valid
               posec is the objective homogeneous transformation*/
            WamIKAC::exactikine(T0,q10,qsol,sol);

            //std::cout<<"q10="<<q10<<std::endl;
            //std::cout<<"qsol="<<qsol<<std::endl;
            //		std::cout<<"sol exactikine"<<sol<<std::endl;
            /* we eliminate solutions which are not valid*/
            WamIKAC::filtersols(qsol,sol);
            if (sol.sum()!=0.){
                WamIKAC::basicsols(qsol,qaux,sol);
                //ROS_INFO("LOOP BREAK, SOLUTION FOUND");
                //std::cout<<"sol="<<std::endl<<sol.transpose()<<std::endl;
                initfound=true;
            }
        }else{
            if (i*step+cjoints(0)>2.6 && -i*step+cjoints(0)<-2.6){
                breakloop=true;
                ROS_INFO("LOOP BREAK,q1 steps OUT OF RANGE");
            }
        }

        /* we take the one with best (q) evaluation function over the rows of qsol */

        i=i+1.;
        /* add possibility of not finding a solution, add i limits*/
    }
    //std::cout<<"initfound="<<initfound<<std::endl;
    //std::cout<<"firstsol="<<std::endl<<qaux<<std::endl;


    if (initfound){
        t2=clock();
        /* Rotate solutions to optimize*/
        /* perform optimization */
        WamIKAC::optimizesol(qaux,T0,qoptim,potq,cjoints);	
        t3=clock();
        double dif1, dif2;
        dif1=((double)t2 - (double)t1)/((double)CLOCKS_PER_SEC);
        dif2=((double)t3 - (double)t2)/((double)CLOCKS_PER_SEC);

        //std::cout<<"initial solution time="<<dif1<<std::endl<<"Optimization time="<<dif2<<std::endl<<"Best potential value="<<potq<<std::endl;
        joints.clear();
        joints.resize(7);
        for(int i=0;i<7;i++)
            joints[i] = qoptim(i);
        end=clock();

        double dif;

        dif = ((double)end-(double)start)/((double)CLOCKS_PER_SEC);

        MatrixXd T01,T12,T23,T34,T45,T56,T67,T07;
        double PI=3.14159265;
        WamIKAC::DHmatrix(-PI/2,0.,0.,qoptim(0),T01),
            WamIKAC::DHmatrix(PI/2,0.,0.,qoptim(1),T12),
            WamIKAC::DHmatrix(-PI/2,0.045,0.55,qoptim(2),T23),
            WamIKAC::DHmatrix(PI/2,-0.045,0.,qoptim(3),T34);
        WamIKAC::DHmatrix(-PI/2,0.,0.3,qoptim(4),T45);
        WamIKAC::DHmatrix(PI/2,0.,0.,qoptim(5),T56);
        WamIKAC::DHmatrix(0.,0.,0.06,qoptim(6),T67);


        T07=T01*T12*T23*T34*T45*T56*T67;

        ROS_DEBUG("ELAPSED TIME IS  %.5lf seconds ", dif );
        return true;
    } else{
        for(int i=0;i<7;i++)
            joints[i] = cjoints(i);
        ROS_DEBUG("SOLUTION NOT FOUND, robot not moving ");
        return false;
    }    


}





void WamIKAC::filtersols(MatrixXd qsol,VectorXd& sol){
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

void WamIKAC::DHmatrix(double alpha, double a, double d,double& theta, MatrixXd& T){
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



void WamIKAC::exactikine(MatrixXd T0, double q10, MatrixXd& qsol,VectorXd&sol){
    // INPUT: T0=MatrixXd, the desired homogeneous transformation.
    //	  q10=double, a fixed value for the first joint.
    // OUTPUT: qsol=MatrixXd, a 8x7 matrix, each row corresponds to a solution of the inverse kinematics for the desired goal.
    //	   sol=vector, indicating wether solution has been found or not for the fixed q1. Note it does not consider joint limits!

    /*DEFINITION DHs*/
    double L=0.06;
    double a=0.3;
    double b=0.045;
    double c=0.55;
    double s3;
    double c3;
    double PI=3.14159265;

    qsol.resize(8,7);

    MatrixXd q2sol2(2,2);
    VectorXd Pw1(4);
    MatrixXd Pw(1,4);
    VectorXd q3(2);
    MatrixXd Pw3(4,2);
    MatrixXd q4sol4(2,2);
    MatrixXd T12,T23,T34,T01,T04;
    MatrixXd qout(8,7);
    qout.fill(0.0);

    int k;
    Vector2d q5;
    Vector2d q6;
    Vector2d q7;


    Pw=T0.block<4,1>(0,3)-L*T0.block<4,1>(0,2);


    double aux1=pow(Pw(0,0),2)+pow(Pw(1,0),2)+pow(Pw(2,0),2)-pow(a,2)-2*pow(b,2)-pow(c,2);
    double aux2=2*a*b+2*b*c;
    double aux3=2*a*c-2*pow(b,2);

    WamIKAC::soltrig(aux1, aux2, aux3,q4sol4);

    WamIKAC::DHmatrix(-PI/2,0.,0.,q10,T01);

    Pw1=T01.inverse()*Pw;

    Pw3.fill(0.0);
    for (int i4=0;i4<2;i4++){
        Pw3(0,i4)=a*sin(q4sol4(i4,0))-b*cos(q4sol4(i4,0));		
        Pw3(1,i4)=-a*cos(q4sol4(i4,0))-b*sin(q4sol4(i4,0));
        Pw3(2,i4)=0.;
        Pw3(3,i4)=1.;

        aux1=11.0/20.0-Pw3(1,i4);
        aux2=Pw1(0);
        aux3=-Pw1(1);
        WamIKAC::soltrig(aux1, aux2, aux3, q2sol2);
        //std::cout<<"q2sol2="<<q2sol2<<std::endl;
        for(int i2=0;i2<2;i2++){
            s3=Pw1(2)/(Pw3(0,i4)+9/200);
            c3=(cos(q2sol2(i2,0))*Pw1(0)+sin(q2sol2(i2,0))*Pw1(1))/(Pw3(0,i4)+9/200);
            q3(i2)=atan2(s3,c3);
            WamIKAC::DHmatrix(PI/2,0.,0.,q2sol2(i2,0),T12);
            WamIKAC::DHmatrix(-PI/2,0.045,0.55,q3(i2),T23);
            WamIKAC::DHmatrix(PI/2,-0.045,0.,q4sol4(i4,0),T34);
            T04=T01*T12*T23*T34;
            WamIKAC::sphericalikine(T0,T04,q5,q6,q7);
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



void WamIKAC::basicsols(MatrixXd qsol,MatrixXd& qaux,VectorXd sol){
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
        //	std::cout<<"i="<<i<<" nbasics="<<nbasics<<std::endl;
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





void WamIKAC::optimizesol(MatrixXd q, MatrixXd T0, VectorXd& qoutput,double& potact,VectorXd qref){
    // INPUT q=qaux in basicsol function.
    //	 T0=desired goal
    // OUPUT qoutput=the best solution, according to a desired optimization function, obtained by rotating the basic solutions around Origin-Wrist axis.
    // 	 potact=the value of the potential function for the solution chosen.

    /* q is the initial guess*/
    VectorXd PLa,PJa,qelbows;
    MatrixXd PL(3,q.rows()); //each column corresponds to one basic solution
    MatrixXd PJ(3,q.rows());

    int IMAX;

    IMAX=50;
    ROS_INFO("Rotations for each solution: %d",IMAX);

    for (int ia=0;ia<q.rows();ia++){
        qelbows=q.row(ia);
        WamIKAC::initialelbows(qelbows,T0,PLa,PJa);
        PL.col(ia)=PLa;
        PJ.col(ia)=PJa;
    }

    double L=0.06;
    MatrixXd T04;
    Matrix3d Mrot,Rn,I3;
    double phi,qn4,qn3,qn2,qn1,s3,c3,eps1,eps3,potq;
    Vector2d qn5,qn6,qn7;
    double PI=3.14159265;
    Vector3d v,CJ,CL,VLJ;
    VectorXd qn4b,sol,qbest;
    MatrixXd Pw,T01,T12,T23,T34,Qaux;
    MatrixXd Qglobal(4*q.rows(),7);	
    qoutput=q.row(0);
    Pw=T0.block<3,1>(0,3)-L*T0.block<3,1>(0,2);
    double aux0=Pw.norm();
    v=Pw/aux0;	

    /*Compute q4*/
    WamIKAC::skewop(Mrot,v);
    I3.setIdentity();

    for (int i=0;i<IMAX;i++){
        phi=2*PI*double(i)/double(IMAX)*pow(-1,double(i));
        Rn=I3+sin(phi)*Mrot+(1-cos(phi))*Mrot*Mrot;

        Qglobal.fill(-9.9);
        for (int nconfig=0;nconfig<q.rows();nconfig++){
            qn4=q(nconfig,3);
            CJ=Rn*PJ.block<3,1>(0,nconfig);
            CL=Rn*PL.block<3,1>(0,nconfig);
            Qaux.resize(4,7);
            Qaux.fill(-9.9);
            if (CL(2)/0.55>-0.4161){
                /*Calculacte q2*/
                qn2=acos(CL(2)/0.55);
                /*Calculate q1*/
                qn1=atan2(CL(1),CL(0));
                /*Calculate q3*/
                VLJ=(CJ-CL)/0.045;
                s3=VLJ(1)*cos(qn1)-VLJ(0)*sin(qn1);
                if (sin(qn2)<0.001){
                    c3=(VLJ(1)*sin(qn1)+VLJ(0)*cos(qn1))/cos(qn2);
                }else{
                    c3=-VLJ(2)/sin(qn2);
                }
                qn3=atan2(s3,c3);
                WamIKAC::DHmatrix(-PI/2,0,0,qn1,T01),
                    WamIKAC::DHmatrix(PI/2,0,0,qn2,T12),
                    WamIKAC::DHmatrix(-PI/2,0.045,0.55,qn3,T23),
                    WamIKAC::DHmatrix(PI/2,-0.045,0,qn4,T34);
                T04=T01*T12*T23*T34;
                WamIKAC::sphericalikine(T0,T04,qn5,qn6,qn7);
                eps1 = copysign(1.0, qn1);
                eps3 = copysign(1.0, qn3);			
                Qaux<<qn1,qn2,qn3,qn4,qn5(0),qn6(0),qn7(0),
                    qn1,qn2,qn3,qn4,qn5(1),qn6(1),qn7(1),
                    qn1-eps1*PI,-qn2,qn3-eps3*PI,qn4,qn5(0),qn6(0),qn7(0),
                    qn1-eps1*PI,-qn2,qn3-eps3*PI,qn4,qn5(1),qn6(1),qn7(1);

            }
            Qglobal.row(4*nconfig)=Qaux.row(0);
            Qglobal.row(4*nconfig+1)=Qaux.row(1);
            Qglobal.row(4*nconfig+2)=Qaux.row(2);
            Qglobal.row(4*nconfig+3)=Qaux.row(3);
        }

        WamIKAC::filtersols(Qglobal,sol);

        WamIKAC::bestsol(Qglobal,sol,qref,qbest,potq);

        // if the potential found is better than the best we had, we update the solution:
        if (potq<potact){
            qoutput=qbest;
            potact=potq;

        }
    }
}




void WamIKAC::skewop(Matrix3d& M,Vector3d v){
    M.fill(0.0);
    M(0,1)=-v(2);
    M(1,0)=v(2);
    M(0,2)=v(1);
    M(2,0)=-v(1);
    M(1,2)=-v(0);
    M(2,1)=v(0);
}

void WamIKAC::getq1(VectorXd cjoints,float i,double& q10,double step){
    q10=cjoints(0)+i*step*pow(-1,i); 	
}





void WamIKAC::bestsol(MatrixXd qsol,VectorXd sol,VectorXd qref,VectorXd& q,double& potbest){


    potbest=10000; /*high initial potential value*/
    double potq;
    VectorXd qact;

    for(int i=0;i<qsol.rows();i++){
        if (sol(i)!=0.0){
            qact=qsol.block<1,7>(i,0);
            WamIKAC::potentialfunction(qref,qact,potq);
            if (potq<potbest){
                q=qact;
                potbest=potq;
            }
        }
    }
}




void WamIKAC::potentialfunction(VectorXd qref,VectorXd& q, double& potq){
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





void WamIKAC::sphericalikine(MatrixXd T0,MatrixXd T4, Vector2d& q5,Vector2d& q6,Vector2d& q7){
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



void WamIKAC::initialelbows(VectorXd q,MatrixXd T0,VectorXd& PL,VectorXd& PJ){
    const double pi=3.14159265;
    MatrixXd T01,T12,T23,T02,T03;
    WamIKAC::DHmatrix(-pi/2,0,0,q(0),T01);
    WamIKAC::DHmatrix(pi/2,0,0,q(1),T12);
    WamIKAC::DHmatrix(-pi/2,0.045,0.55,q(2),T23);
    T02=T01*T12;
    T03=T02*T23;
    VectorXd aux(4),PLaux(4);
    aux(0)=0.0;
    aux(1)=0.0;
    aux(2)=0.55;
    aux(3)=1.0;
    PLaux=T02*aux;
    PL=PLaux.block<3,1>(0,0);
    PJ=T03.block<3,1>(0,3);

}



void WamIKAC::soltrig(double a, double b, double c, MatrixXd& q2sol2){
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
    WamIKAC wam_ikac;
    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep(); 
    }

}
