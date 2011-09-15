/*
 * Jarvis Schultz
 * July 7, 2011
 *
 * Servo animation ROS package code.
 */

#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <float.h>
#include <assert.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
using namespace std;

/*******************************************************************************
 * GLOBAL VARIABLES ************************************************************
 ******************************************************************************/ 

#define		BAUDRATE	B115200
#define		MODEMDEVICE	"/dev/ttyUSB0"
#define 	_POSIX_SOURCE	1 /* POSIX compliant source */
#define         PACKET_SIZE	4
#define		NUM_SERVOS	8
#define		MIN_PULSE	992
#define		MAX_PULSE	2000
#define		MAX_CHANNELS	18
#define		MAX_SPEED	255

int fd;
struct termios oldtio,newtio;
unsigned int NUM_FRAMES = 0;
int offset_array[NUM_SERVOS] = {-60,0,160,120,30,70,-20,-160};
std::string filename;


/*******************************************************************************
 * FUNCTION DECLARATIONS *******************************************************
 ******************************************************************************/
void init_comm(void);
void send_calibrate_frame(void);

/*******************************************************************************
 * CLASS DECLARATIONS ** *******************************************************
 ******************************************************************************/
typedef struct
{
    unsigned int chan;
    unsigned int ramp;
    unsigned int range;
} Servo;

typedef struct
{
    unsigned int pause;
    Servo servo_array[NUM_SERVOS];
} Frame;

typedef struct
{
    int dummy;
    Frame frame_array[];
} Show;

class ServoController {

private:
    unsigned int current_frame;
    int operating_condition;
    bool start_flag;
    Show *animation;
    char path[256];
    ros::NodeHandle n_;
    ros::Timer timer;

public:
    ServoController() {
	start_flag = true;
	current_frame = 0;

	// Reading controls:
	ROS_INFO("Reading Controls");
	animation = ReadControls(filename);
	ROS_INFO("Done Reading controls");

	// send_calibrate_frame();

	// Check for system operating_condition parameter:
	if(ros::param::has("operating_condition"))
	{
	    // set operating_condition to idle so the robot doesn't move
	    ros::param::set("operating_condition", 0);
	}
	else
	{
	    ROS_WARN("Cannot Find Parameter: operating_condition");
	    ros::param::set("/operating_condition", 0);
	}
	ROS_INFO("Waiting for operating_condition to be set to run");

	// create timer:
	timer = n_.createTimer(ros::Duration(0.01), &ServoController::timerCallback, this);
    }

    void timerCallback(const ros::TimerEvent& e)
	{
	    static double delay_time = 0.0;
	    static ros::Time base_time;
	    static bool timer_flag = true;
	    
	    if(ros::param::has("operating_condition"))
	    {
		ros::param::get("/operating_condition", operating_condition);
	    }
	    else 
		return;

	    if (operating_condition != 2)
	    {
		delay_time = 0.0;
		start_flag = true;
		return;
	    }
	    else
	    {
		if (timer_flag == true)
		{
		    // Get base time:
		    base_time = ros::Time::now();
		    timer_flag = false;
		}

		// calculate total delay time:
		delay_time = ((ros::Time::now()).toSec()-base_time.toSec())*1000.0;

		if (delay_time >= (&animation->frame_array[current_frame])->pause)
		{
		    ROS_DEBUG("Sending new servo commands");
		    // Then send data, and reset timers:
		    send_animation(animation);
		    timer_flag = true;
		    delay_time = 0.0;
		}
	    }
	}
		
// This function actually sends out the position and speed commands to
// a given servo
    void send_data(Servo *servo)
	{
	    char dest[PACKET_SIZE];
	    memset(dest,0,sizeof(dest));

	    // set the channel value:
	    if (((int) servo->chan >= 0) && ((int) servo->chan <= MAX_CHANNELS))
		dest[1] = servo->chan;
	    else
	    {
		puts("ERROR: Channel out of range");
		return;
	    }
	    
	    // Send the speed packet:
	    dest[0] = 0x87;

	    if ((int) servo->ramp >=0 && (int) servo->ramp <= MAX_SPEED)
	    {
		dest[2] = servo->ramp & 0x7F;
		dest[3] = ((servo->ramp) >> 7) & 0x7F;
	    }
	    else
	    {
		puts("ERROR: Speed out of range");
		return;
	    }
	    // Now we can send the data:
	    write(fd, dest, PACKET_SIZE);
	    fsync(fd);


	    // Set the position command byte:
	    dest[0] = 0x84;
	    // set position value:
	    if ((int) servo->range >= (MIN_PULSE+offset_array[servo->chan])*4 &&
		(int) servo->range <= (MAX_PULSE+offset_array[servo->chan])*4)
	    {
		dest[2] = servo->range & 0x7F;
		dest[3] = ((servo->range) >> 7) & 0x7F;
	    }
	    else
	    {
		puts("ERROR: Position out of range");
		return;
	    }
	    // Now we can send the data:
	    write(fd, dest, PACKET_SIZE);
	    fsync(fd);
	}


// This function sends out a speed and position command to all of the
// servos on the robot:
    void send_frame(Frame *frame)
	{
	    int i;
	    for(i=0; i<NUM_SERVOS; i++)
	    {
		send_data(&frame->servo_array[i]);
	    }
	}


// This function sends out a complete animation to the robot
    void send_animation(Show *animation)
	{
	    if (start_flag == true)
	    {
		ROS_INFO("Starting Animation");
		current_frame = 0;
		start_flag = false;
	    }
	    ROS_INFO("FRAME NUMBER %u",current_frame);
	    send_frame(&animation->frame_array[current_frame]);
	    current_frame++;
	    if(current_frame>(NUM_FRAMES-1)) current_frame = 0;
	}

    
// This function reads in a text file that contains a complete
// animation, and then returns a struct that contains all of the
// relevant information.
    Show *ReadControls(std::string filename)
	{
	    unsigned int i,j, temp_int;
	    std::string line, temp;
	    Show *animation;
	    Frame temp_frame;
	    Servo temp_servo;
	    ifstream file;
	    file.open(filename.c_str(), fstream::in);
	    // Read line that tells us the number of frames:
	    getline(file, line);
	    // Get number of frames:
	    std::stringstream ss(line);
	    
	    ss >> temp >> NUM_FRAMES;

	    // Read and ignore headers:
	    getline(file, line);

	    // Now we can read in the show:
	    unsigned int pause_array[NUM_FRAMES];
	    unsigned int big_array_length = NUM_FRAMES*NUM_SERVOS;
	    unsigned int range_array[big_array_length], ramp_array[big_array_length];

	    for(i=0; i<NUM_FRAMES; i++)
	    {
		// Get rid of frame number:
		getline(file, line, ',');
		// Get pause value:
		getline(file, line, ',');
		std::stringstream ss(line);
		ss >> temp_int;
		pause_array[i] = temp_int;

		// Get ramp values:
		for(j=0; j<NUM_SERVOS; j++)
		{
		    getline(file, line, ',');
		    std::stringstream ss(line);
		    ss >> temp_int;
		    ramp_array[i*NUM_SERVOS+j] = temp_int;
		}

		// Get range values:
		for(j=0; j<NUM_SERVOS; j++)
		{
		    getline(file, line, ',');
		    std::stringstream ss(line);
		    ss >> temp_int;
		    range_array[i*NUM_SERVOS+j] = temp_int;
		}
	    }
	    
	    // Now, we need to define the size of the show struct:
	    size_t alloc;
	    alloc = sizeof(*animation) + sizeof(animation->frame_array[0])*NUM_FRAMES;
	    animation = (Show*) malloc(alloc);

	    // Now, let's fill in the arrays:
	    animation->dummy = 0;
	    for(i=0; i<NUM_FRAMES; i++)
	    {
		for(j=0; j<NUM_SERVOS; j++)
		{
		    temp_servo.chan = j;
		    temp_servo.ramp = (1-ramp_array[i*NUM_SERVOS+j]/63)*MAX_SPEED;
		    temp_servo.range = (range_array[i*NUM_SERVOS+j]*2+offset_array[j])*4;
		    temp_frame.servo_array[j] = temp_servo;
		}
		temp_frame.pause = pause_array[i];
		animation->frame_array[i] = temp_frame;
	    }
	    file.close();
	    return(animation);  
	}

};


/*******************************************************************************
 * FUNCTIONS TO CALL************************************************************
 ******************************************************************************/

// The following function is used for opening a com port to
// communicate with the servo minimatronic.
void init_comm(void)
{
    // Open port
    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY ); 
    if (fd <0) {perror(MODEMDEVICE); exit(-1); }

    tcgetattr(fd,&oldtio); /* save current serial port settings */
    bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    // Set all desired settings:
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD | CSTOPB;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag &= ~OPOST;
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    // Flush buffer, and set attributes:
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
}


// Calling this function sends all servos to their respective neutral positions
void send_calibrate_frame(void)
{
    // Create a Show:
    Servo temp_servo;
    Frame temp_frame;
    Show *cal;
    unsigned int i,j;

    // Now, we need to define the size of the show struct:
    size_t alloc;
    alloc = sizeof(*cal) + sizeof(cal->frame_array[0]);
    cal = (Show*) malloc(alloc);

    // Now, let's fill in the arrays:
    cal->dummy = 0;
    for(i=0; i<NUM_FRAMES; i++)
    {
	for(j=0; j<NUM_SERVOS; j++)
	{
	    temp_servo.chan = j;
	    temp_servo.ramp = 220;
	    temp_servo.range = (1500+offset_array[j])*4;
	    temp_frame.servo_array[j] = temp_servo;
	}
	temp_frame.pause = 0;
	cal->frame_array[i] = temp_frame;
    }
}


// command_line parsing:
void command_line_parser(int argc, char** argv)
{
    std::string working_dir,  file;
   
    // First set the global working directory to the location of the
    // binary:
    working_dir = argv[0];

    int fflag = 0, pflag = 0;
    int index;
    int c;
     
    opterr = 0;
     
    while ((c = getopt (argc, argv, "f:p:")) != -1)
    {
	switch (c)
	{
	case 'f':
	    fflag = 1;
	    file = optarg;
	    break;
	case 'p':
	    pflag = 1;
	    working_dir = optarg;
	    break;
	case ':':
	    fprintf(stderr,
		    "No argument given for command line option %c \n\r", c);
	default:
	    fprintf(stderr, "Usage: %s [-f filename] [-p path-to-file]\n",
		    argv[0]);
	    exit(EXIT_FAILURE);
	}
    }
     
    for (index = optind; index < argc; index++)
    	printf ("Non-option argument %s\n", argv[index]);
    if (pflag != 1)
    {
	// Then we just use the default path:
	std::size_t found = working_dir.find("bin");
	std::string tmp_dir = working_dir.substr(0, found);
	working_dir = tmp_dir+"data/";
    }
 
    if (fflag == 0)
    {
	// No file was given:
	file = "default.txt";
    }
  
    // Get filenames:
    filename = working_dir + file;
    cout << filename << "\n";
    return;
}


int main(int argc, char** argv)
{
    // Create ROS Node:
    ros::init(argc, argv, "servo_controller");
    ros::NodeHandle n;

    command_line_parser(argc, argv);

    // instantiate a ServoController class object
    ServoController controller1;

    // Initialize communication:
    init_comm();

    // infinite loop
    ros::spin();
    
    close(fd);
    return (EXIT_SUCCESS);
}
