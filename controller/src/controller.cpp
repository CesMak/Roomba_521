//TODO: Bugs: This will give us a working turtlesim position controller with a few bugs. You will notice a rotational error when the turtle needs to go left sometimes and the turtle never quiet makes it into position. Feel free to fork this repo and look into solutions to these probems. 
// from website: https://github.com/utari/UTARI_ROSTutorials/wiki/TurtlesimPositionController-Tutorial

//mit der ich dreh mich um in einem rückwärtsliegenden Punkt zu fahren klappt bisher alles super!

#include "ros/ros.h"
#include <math.h>
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose2D.h" // to get desired position
#include "nav_msgs/Odometry.h"  // type von odom also um die aktuelle Position zu lesen.
#include "geometry_msgs/Twist.h"  // type von cmd_vel senden von velocity commands.
#include <tf/transform_datatypes.h> // include to transform quaterions to RYP http://wiki.ros.org/tf/Overview/Data%20Types

// Function declarations
void ComPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg); //desired Position
void Stop_subCallback(const std_msgs::Empty::ConstPtr& msg);
void curPosCallback(const nav_msgs::Odometry::ConstPtr& msg);
double Quaterion2Yaw(nav_msgs::Odometry curodom);
float GetErrorAng(nav_msgs::Odometry curpose, geometry_msgs::Pose2D despose);
float GetErrorLin(nav_msgs::Odometry curpose, geometry_msgs::Pose2D despose);
double Yaw2Degree(double yaw);

//Global Variables:
bool STOP = true;
geometry_msgs::Pose2D DesPose; // desired Position
nav_msgs::Odometry CurPose; // current Position.
geometry_msgs::Twist CmdVel;    //geschwindigkeit die es fahren soll.

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TurtlesimPositionController_pubsub"); // connect to roscore
    ros::NodeHandle n;                                     // node object


    // register sub to get desired position/pose commands
    // höre gewünschte Position
    ros::Subscriber ComPose_sub = n.subscribe("/turtle1/PositionCommand", 5, ComPoseCallback);
 
    // aboniert ob Button gedrückt wird wenn ja dann wird Stop_subCallback ausgeführt!	   
    ros::Subscriber Stop_sub = n.subscribe("clean_button", 5, Stop_subCallback);
    
    ros::Subscriber Curr_sub = n.subscribe("odom", 5, curPosCallback);
   
    // register pub to send twist velocity (cmd_vel)
    // sende Geschwindigkeitskomando!
    //gleicher Publisher gleich nennen wie subscriber dann wird
   //hiermit zu dem subscriber ein wert gesendet!
    ros::Publisher Twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);



    ros::Rate loop_rate(10); //set a frequency to run loops in ROS.
			      // ros::spin() = loop if only once: ros::spinOnce(); 
			     //Alternatively use ros::Rate to control the loop frequency.
	
	//Initialisierung wird nur einmal am start aufgerufen:	
	float ErrorLin=0;
	float ErrorAng =0;
	 CmdVel.linear.x=0;
	 CmdVel.angular.z=0;

  ROS_INFO("[Controller Node:] Controller Node sucessfully started");
  ROS_INFO("[Controller Node:] Ready to send position commands");
  ROS_INFO("[Controller Node:] Ready to hear Stop command (If you press the Clean button)");

  while (ros::ok() && n.ok() )// while ros and the node are ok
    {
        ros::spinOnce();
	if(STOP == false)
	{
	    ErrorLin = GetErrorLin(CurPose, DesPose);
            ErrorAng = GetErrorAng(CurPose, DesPose);
            ROS_INFO("Error linear: %f, Error angular: %f\n", ErrorLin, ErrorAng);
		
	if(ErrorLin>0)
	{
	CmdVel.linear.x = 0.2 * ErrorLin;// multiple by linear P for control signal
	}
	else{
      	CmdVel.linear.x=0;
	}
	
	  CmdVel.angular.z = 0.5 * ErrorAng; // multiple by linear P for control signal 
          Twist_pub.publish(CmdVel);
	}
	else{
	   ROS_INFO("Waiting...\n");
	}
 	 loop_rate.sleep();    
}
}


// call back to send new desired Pose msgs
//Eingabe vom Typ 2D: rostopic pub /turtle1/PositionCommand geometry_msgs/Pose2D "x: 0.0 y: 0.0 theta: 0.0" -1
// Aufruf mit: rostopic pub /turtle1/PositionCommand geometry_msgs/Pose2D "x: 0.0, y:0.0, theta: 0.0" -1

//Desired Position: speichern in DesPose
void ComPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)            
{
    ROS_INFO("Received Command msg");
    STOP = false;
    DesPose.x = msg->x;
    DesPose.y = msg->y;
   // CmdVel.linear.x=0.0;
   //   CmdVel.angular.z=0.1;
    return;
}

void Stop_subCallback(const std_msgs::Empty::ConstPtr& msg)            
{
    STOP = true;
    ROS_INFO("Stop received!");
    CmdVel.angular.z=0;
    CmdVel.linear.x=0;
    return;
}

// Information about the Twist message:
//  geometry_msgs/Twist twist
//    geometry_msgs/Vector3 linear
//      float64 x
//      float64 y
//      float64 z
//    geometry_msgs/Vector3 angular
//      float64 x
//      float64 y
//      float64 z
//  float64[36] covariance
void curPosCallback(const nav_msgs::Odometry::ConstPtr& msg)            
{
	// wird die ganze Zeit aufgerufen: ROS_INFO("Current Position received!");	
	//ROS_INFO(msg);    
	// xVel: CurPos.twist.twist.linear.x

	//Geschwindigkeit:
	//CurPose.twist = msg->twist; 
	CurPose.pose = msg->pose;
	

	//ROS_INFO("Orientation: z(deg): [%f]",yaw_degrees);
	ROS_INFO("Position and Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      //  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

    return;
}


// Methods for a Controller:
float GetErrorAng(nav_msgs::Odometry curpose, geometry_msgs::Pose2D despose){
	float Ex = despose.x - curpose.pose.pose.position.x;
	float Ey = despose.y - curpose.pose.pose.position.y;

	//desired Angle:
	float dest = atan2f(Ey,Ex); //radians Winkel von 0 bis +3.141 (180Grad) und 0 bis -3.141 (-180grad);

	float Et = dest - Quaterion2Yaw(curpose);
	
return Et;
}

float GetErrorLin(nav_msgs::Odometry curpose, geometry_msgs::Pose2D despose){

	float Ex = despose.x - curpose.pose.pose.position.x;
	float Ey = despose.y - curpose.pose.pose.position.y;
	float Et = GetErrorAng(curpose,despose);

    // project error onto turtle x axis
    // hypot liefert: r = sqrt(x²+y²)
    //*cos(Et) liefert error auf der x-Achse.
    //~ float Etx =  pow( pow(Ex,2.0) + pow(Ey,2.0), 0.5 )*cos(Et);
    float Etx = hypotf(Ex, Ey)*cos(Et); // improved c function

	return Etx;
}


double Yaw2Degree(double yaw){
	double yaw_degrees = yaw * 180.0 / M_PI; // conversion to degrees
	if( yaw_degrees < 0 ) yaw_degrees += 360.0; // funktioniert!
//output 0-360 degree
return yaw_degrees;
}

double Quaterion2Yaw(nav_msgs::Odometry curodom){
	//Quaterions to RPY (Euler roll, pitch, yaw)
	tf::Quaternion q(curodom.pose.pose.orientation.x, curodom.pose.pose.orientation.y, curodom.pose.pose.orientation.z, curodom.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

//output: yaw: [0,PI] (-PI,0) radians

return yaw;
}


