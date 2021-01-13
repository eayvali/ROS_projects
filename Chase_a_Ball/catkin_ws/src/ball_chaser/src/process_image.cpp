#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Commanding the robot to chase the ball");
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service and pass the requested velocities
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{


    int white_pixel = 255;
    int num_channels =3; //RGB
    bool ball_in_view = false;
    //img: 1D array [[R00,G00,B00],...,[R0r,G0r,0r], [R10,G10,B10],..] 

    int col=0; // x-pixel coordinate  
    int ball_col=0; //detected ball  position (x-pixel)
    int ball_dia=0;// ball diameter in pixels
    int col_cnt[img.width]={0}; // count white pixels along each row ( max column sum is proxy for ball center)
    int max_cnt=0;
    float lin_x=0;
    float ang_z=0;

    //ROS_INFO("height: %d, width: %d, step: %d", img.height, img.width, img.step); //800,800,2400

    // Loop through each pixel in the image and check if there's a bright white one

    for (int i = 0; i < img.height * img.step; i += 3) {
        if(img.data[i]==white_pixel && img.data[i+1]==white_pixel && img.data[i+2]==white_pixel) {
            col=(i % img.step)/num_channels;
	    col_cnt[col]+=1;
        }
    }

    //Find ball_col: column with the max white pixels
    for (int i=0;i<img.width;i++){
        if (col_cnt[i]>max_cnt){
        max_cnt=col_cnt[i];
        ball_col=i; //assign ball column in pixels
        ball_dia=max_cnt; //ball diameter in pixels
        }
    }
    //Need at least 10 white pixels to define a ball
    if ((ball_dia>10) && (ball_dia<0.75*img.height)){
        ball_in_view=true;
        ROS_INFO("Ball detected!");
        // Identify if this pixel falls in the left, mid, or right side of the image
        // Depending on the white ball position, call the drive_bot function and pass velocities to it
	lin_x=std::min(0.1*(img.height/ball_dia),0.5);
	ang_z=1.0;

        if (ball_col < img.width/3){
            drive_robot(lin_x, ang_z);
            ROS_INFO("Ball pos: left");
	}
        else if (ball_col < 2*img.width/3){
            drive_robot(lin_x, 0.0); 
            ROS_INFO("Ball pos: center");
	}
        else {
            drive_robot(lin_x, -ang_z);
            ROS_INFO("Ball pos: right");

	}
    }


    // Request a stop when there's no white ball seen by the camera
    if (ball_in_view==false){
	drive_robot(0.0,0.0); 
    }

    ROS_INFO("ball_in_view: %d ,ball_column: %d, ball_diameter: %d",ball_in_view, ball_col, ball_dia);
    ROS_INFO("Commands: lin_x: %f, ang_z: %f", lin_x, ang_z);


}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
