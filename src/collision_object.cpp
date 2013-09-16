#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

ros::Publisher planning_scene_publisher;
moveit_msgs::PlanningScene planning_scene;
std::string origin;
const double PI = 3.14159265359;

// returns the position in the vector of collision objects given the name of the object
int iterator (const std::string& id)
{
  int it = 0;
  while (it < (int) planning_scene.world.collision_objects.size() and id != planning_scene.world.collision_objects[it].id) ++it;
  return it;
}

// publishes the collision object in the planning scene
void add (const int& it)
{
  moveit_msgs::CollisionObject& obstacle = planning_scene.world.collision_objects[it];
  obstacle.operation = obstacle.ADD;
  planning_scene_publisher.publish(planning_scene);
}

void add (const std::string& id)
{
  int it = iterator(id);
  add(it);
}

// publishes that the collision objects doesn't exist and erases it from the vector of collision objects
void erase (const int& it)
{
  moveit_msgs::CollisionObject& obstacle = planning_scene.world.collision_objects[it];
  obstacle.operation = obstacle.REMOVE;
  planning_scene_publisher.publish(planning_scene);
  planning_scene.world.collision_objects.erase(planning_scene.world.collision_objects.begin()+it);
}

void erase (const std::string& id)
{
  int it = iterator(id);
  erase(it);
}

// given a collision object this action lets move it
void move (const int& it) 
{
// get the collision object chosen
  moveit_msgs::CollisionObject& obstacle = planning_scene.world.collision_objects[it];
// get the pose of collision object
  geometry_msgs::Pose pose;
  int size = obstacle.primitive_poses.size();
  if (size)
  {
    pose = obstacle.primitive_poses.back();
    obstacle.primitive_poses.pop_back();
  }
// prints the current position of the object
  std::cout << "I'm here: [" << pose.position.x << "," << pose.position.y << "," << pose.position.z << "]" << std::endl;
  std::cout << "Move to: ";
// reads the new position
  std:: cin >> pose.position.x >> pose.position.y >> pose.position.z;
// redefine the pose of the collision object and publish it
  obstacle.primitive_poses.push_back(pose);
  if (size) add(it);
}

void move (const std::string& id) 
{
  int it = iterator(id);
  move(it);
}

// given a collision object this action lets change its size
void resize (const int& it)
{
// get the collision object chosen
  shape_msgs::SolidPrimitive& primitive = planning_scene.world.collision_objects[it].primitives.back();
// prints the current size
  std::cout << "My dimensions are ";
  int size = primitive.dimensions.size();
  if (size)
  {
    if (size == 1) std::cout << "(radius): " << primitive.dimensions[0] << std::endl;
    else if (size == 2) std::cout << "(height and radius): " << primitive.dimensions[0] << " " << primitive.dimensions[1] << std::endl;
    else std::cout << "(x, y and z): " << primitive.dimensions[0] << " " << primitive.dimensions[1] << " " << primitive.dimensions[2] << std::endl;
  }
  else 
  {
    std::cout << "--" << std::endl;
    if (primitive.type == primitive.BOX) size = 3;
    else if (primitive.type == primitive.SPHERE) size = 1;
    else size = 2;
    primitive.dimensions.resize(size);
  }
// reads the new size  
  std::cout << "My dimensions will be ";
  if (size == 1) 
  {
    std::cout << "(radius): ";
    std::cin >> primitive.dimensions[0];
  }
  else if (size == 2) 
  {
    std::cout << "(height and radius): ";
    std::cin >> primitive.dimensions[0] >> primitive.dimensions[1];
  }
  else 
  {
    std::cout << "(x, y and z): ";
    std::cin >> primitive.dimensions[0] >> primitive.dimensions[1] >> primitive.dimensions[2];
  }
// publish the new size 
  add(it);
}

void resize (const std::string& id) 
{
  int it = iterator(id);
  resize(it);
}

// function that makes dot product
double dot_prod (const std::vector<double>& u, const std::vector<double>& v) 
{
  double product = 0;
  for (int i = 0; i < 3; ++i) product += u[i]*v[i];
  return product;
}

// function that maket scalar multiplication
std::vector<double> scalar_mult (const double& a, const std::vector<double>& u) 
{
  std::vector<double> v = u;
  for (int i = 0; i < 3; ++i) v[i] *= a;
  return v;
}

// function that adds vectors 
std::vector<double> add_vec (const std::vector<double>& u, const std::vector<double>& v) 
{
  std::vector<double> add_vec = u;
  for (int i = 0; i < 3; ++i) add_vec[i] += v[i];
  return add_vec;
} 

// function that returns the vectorial part of the multiplication of two quaternions given by his scalar and vector parts
std::vector<double> quat_prod_vec(const double& p, const double& q, const std::vector<double>& u, const std::vector<double>& v) 
{
  std::vector<double> pv(3, 0);
  pv[0] = u[1]*v[2] - u[2]*v[1];
  pv[1] = u[2]*v[0] - u[0]*v[2];
  pv[2] = u[0]*v[1] - u[1]*v[0];
  std::vector<double> w1 = scalar_mult(p, v);
  std::vector<double> w2 = scalar_mult(q, u);
  std::vector<double> parc = add_vec(pv, w1);
  std::vector<double> quat = add_vec(parc, w2);
  return quat;
}

// function that makes quaternion product
geometry_msgs::Quaternion quat_prod (const geometry_msgs::Quaternion& P, const geometry_msgs::Quaternion& Q)
{
  geometry_msgs::Quaternion product;
  double p = P.w, q = Q.w;
  std::vector<double> u(3), v(3);
  u[0] = P.x, u[1] = P.y, u[2] = P.z; 
  v[0] = Q.x, v[1] = Q.y, v[2] = Q.z;
  product.w = p*q - dot_prod(u, v);
  std::vector<double> w =quat_prod_vec(p, q, u, v);
  product.x = w[0], product.y = w[1], product.z = w[2];
  return product;
}

// given a collision object this action lets rotate it
void rotate (const int& it)
{
// gets current pose
  geometry_msgs::Pose& pose = planning_scene.world.collision_objects[it].primitive_poses.back();
// reads the direction and angle of the rotation
  std::cout << "Make this rotation (normal vector and angle): ";
  double x, y, z, w;
  std:: cin >> x >> y >> z >> w;
// normalize the vector to use it as an unitary quaternion
  double norm = sqrt(x*x + y*y + z*z);
  x /= norm, y /= norm, z /= norm;
  w = w/360*2*PI;
  x *= sin(w/2), y *= sin(w/2), z *= sin(w/2);
  w = cos(w/2);
  geometry_msgs::Quaternion rot;
  rot.x = x, rot.y = y, rot.z = z, rot.w = w;
  geometry_msgs::Quaternion q0 = pose.orientation;
  geometry_msgs::Quaternion quat = quat_prod(rot, q0);
  pose.orientation = quat;
// publish the object with his new rotation respect the origin
  add(it);
}

void rotate (const std::string& id) 
{
  int it = iterator(id);
  rotate(it);
}
// creates a new collision object
void create () 
{
  moveit_msgs::CollisionObject obstacle;

  std::cout << "Name of the object: ";
  std::cin >> obstacle.id;
  
  std::cout << "Name of the link used as origin: " << origin << std::endl;
  obstacle.header.frame_id = origin;

  planning_scene.world.collision_objects.push_back(obstacle);

  int last = planning_scene.world.collision_objects.size()-1;
  move(last);

  geometry_msgs::Pose& pose = planning_scene.world.collision_objects[last].primitive_poses.back();
  pose.orientation.x = 0, pose.orientation.y = 0, pose.orientation.z = 0, pose.orientation.w = 1;

  std::cout << "Shape (BOX=b/SPHERE=s/CYLINDER=cl/CONE=cn): ";
  shape_msgs::SolidPrimitive primitive;
  std::string type;
  std::cin >> type;
  while (type != "b" and type != "s" and type != "cl" and type != "cn")
  {
    std::cout << "Say (b/s/cl/cn): ";
    std::cin >> type;
  }
  if (type == "b") primitive.type = primitive.BOX;
  else if (type == "s") primitive.type = primitive.SPHERE;
  else if (type == "cl") primitive.type = primitive.CYLINDER;
  else primitive.type = primitive.CONE;

  planning_scene.world.collision_objects[last].primitives.push_back(primitive);

  resize(last);
}

int main(int argc, char **argv)
{
// start a new node
  ros::init (argc, argv, "collision_object");
  ros::NodeHandle nh;
// this node will publish to /planning_scene topic and will be client of the \get_planning_scene service
  planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  ros::AsyncSpinner spinner(1);
  spinner.start();
// define a service message that demmands all the information that the service can give
  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components = 1023;
  
  char comanda = 'c';
  std::cout << "Create a collision object!" << std::endl;
  while (comanda != 'f')
  {
// read the current planning scene
    client.call(srv);
    planning_scene = srv.response.scene;
    origin = planning_scene.world.octomap.header.frame_id;
    if (comanda == 'c') create();
    else 
    {
      std::cout << "Which object are you talking about?" << std::endl;
      for (int i = 0; i < (int) planning_scene.world.collision_objects.size(); ++i) std::cout << planning_scene.world.collision_objects[i].id << std::endl;
      std::string s;
      std::cin >> s;
      if (comanda == 'e') erase(s);
      else if (comanda == 'm') move(s);
      else if (comanda == 's') resize(s);
      else if (comanda == 'r') rotate(s);
    }
    std::cout << "What do you want to do? (CREATE=c" << (planning_scene.world.collision_objects.size() ? "/ERASE=e/MOVE=m/SIZE=s/ROTATE=r" : "" ) << "/FINISH=f) ";
    std::cin >> comanda;
    while ((!planning_scene.world.collision_objects.size() and comanda != 'c' and comanda != 'f') or (comanda != 'c' and comanda != 'e' and comanda != 'm' and comanda != 's' and comanda != 'r' and comanda != 'f'))
    {
      std::cout << "Say (c" << (planning_scene.world.collision_objects.size() ? "/e/m/s/r" : "" ) << "/f): ";
      std::cin >> comanda;
    }
  }

  while ((int) planning_scene.world.collision_objects.size()) erase(0);
  ros::shutdown();
  return 0;
}
