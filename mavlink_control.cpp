//---Include---//

#include "mavlink_control.h"

//---Top---//

int top(int argc, char **argv)
{
	
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";  
#else
	char *uart_name = (char*)"/dev/ttyS0"; //dia chi UART Port
#endif
	int baudrate = 921600;

	bool use_udp = false;
	char *udp_ip = (char*)"127.0.0.1";  
	int udp_port = 14540;
	bool autotakeoff = true;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate, use_udp, udp_ip, udp_port, autotakeoff);

    
//---PORT and THREAD STARTUP---//


	/*
	 * Instantiate a generic port object
	 *
	 * Quan ly su dong/mo cac port giao tiep voi autopilot cua offboard computer.
	 * Bao gom cac phuong thuc doc va viet cac a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock. It can be a serial or an UDP port.
	 *
	 */
	
	Generic_Port *port;
	if(use_udp)		//use_udp = false
	{
		port = new UDP_Port(udp_ip, udp_port);
	}
	else
	{
		port = new Serial_Port(uart_name, baudrate);		//cap phat bo nho dong cho con tro "port"
	}

	
	/*
	 * Instantiate an autopilot interface object
	 *
	 * Bat dau 2 luong doc va viet MAVLink. Luong doc lang nghe bat ky MAVlink message nao va pushes no vao current_messages
	 * attribute.  Luong viet tai thoi diem hien tai chi streams position target
	 * trong he toa do local NED frame (mavlink_set_position_target_local_ned_t), 
	 * duoc thay doi bang phuong phap update_setpoint().  Sending these messages
	 * are only half the requirement to get response from the autopilot, tin hieu vao 
	 * "offboard_control" mode duoc gui bang ham enable_offboard_control().
	 * Tin hieu thoat ra khoi mode nay se disable_offboard_control().  It's
	 * important that one way or another this program signals offboard mode exit,
	 * otherwise the vehicle will go into failsafe.
	 *
	 */
	
	Autopilot_Interface autopilot_interface(port);
	
	
	/*
	 * Thiet lap tin hieu ngat
	 *
	 * Ngat truong trinh voi Ctrl-C.  The handler se thuc hien
	 * thoat offboard mode if duoc yeu cau, va dong luong va cac port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
	 
	port_quit         = port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);
	
	/*
	 * Start the port and autopilot_interface
	 * Tai day cac port se bat dau mo, cac luong doc va viet bat dau/
	 */
	
	port->start();
	autopilot_interface.start();

//---RUN COMMANDS---//

	/*
	 * Lap trinh thuat toan ta muon ben tren autopilot interface
	 */
	 commands(autopilot_interface, autotakeoff);
	 
//---THREAD and PORT SHUTDOWN---///

    /*
     * Ket thuc, dong luong va port
	 */
	 
	autopilot_interface.stop();
	port->stop();
	
	delete port;

//---DONE---//

	// woot!
	return 0;

}

//---HAM COMMANDS---//

void
commands(Autopilot_Interface &api, bool autotakeoff)
{

	// --------------------------------------------------------------------------
	//   START OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.enable_offboard_control();
	usleep(100); // give some time to let it sink in

	// now the autopilot is accepting setpoint commands

	if(autotakeoff)
	{
		// arm autopilot
		api.arm_disarm(true);
		usleep(100); // give some time to let it sink in
	}

	// --------------------------------------------------------------------------
	//   SEND OFFBOARD COMMANDS
	// --------------------------------------------------------------------------
	printf("SEND OFFBOARD COMMANDS\n");

	// Khoi tao Object
	mavlink_set_position_target_local_ned_t sp; // Object sp du dung de thiet lap vi tri cho uav.
	mavlink_set_position_target_local_ned_t ip = api.initial_position; // Object ip luu vi tri hien tai cua uav.

	// autopilot_interface.h provides some helper functions to build the command




	// Fly up by to 1m
	set_position( ip.x ,       // [m]
			 	  ip.y ,       // [m]
				  ip.z - 0.5, // [m]
				  sp         ); // Thiet lap vi tri 1m theo he toa do xyz

	if(autotakeoff)
	{
		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;
	}

	// SEND THE COMMAND
	api.update_setpoint(sp); //??
	// NOW pixhawk will try to move

	// Wait for 8 seconds, check position
	for (int i=0; i < 6; i++)
	{
		mavlink_local_position_ned_t pos = api.current_messages.local_position_ned; // Object pos dung de lay thong tin vi tri uav va hien thi.
		printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
		sleep(1);
	}
	
 /*   
    //Land using position
    set_position( ip.x,
                  ip.y,
                  ip.z + 1.0 ,
                  sp );
     if(autotakeoff)
     {
		 sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND;
	 }
	 
	 //SEND THE COMMAND
	 api.update_setpoint(sp);
	 // NOW pixhawk will try to move
	 
	 // Wait for 8n seconds, check position
	 for(int i=0; i < 8; i++)
	 {
		mavlink_local_position_ned_t pos = api.current_messages.local_position_ned; // Doi tuong pos du dung de lay thong tin vi tri uav va hien thi.
		printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
		sleep(1);
	 }
*/
    if(autotakeoff)
    {
	//Land using fixed velocity
		set_velocity(  0.0       , // [m/s]
					   0.0       , // [m/s]
					   0.2       , // [m/s]
					   sp        ); // Thiet lap van toc ha canh

		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND;

		// SEND THE COMMAND
		api.update_setpoint(sp);
		// NOW pixhawk will try to move

		// Wait for 8 seconds, check position
		for (int i=0; i < 10; i++)
		{
			mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
			printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
			sleep(1);
		}

		printf("\n");

		// disarm autopilot
		api.arm_disarm(false);
		usleep(100); // give some time to let it sink in
		
	}

	// --------------------------------------------------------------------------
	//   STOP OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.disable_offboard_control();

	// now pixhawk isn't listening to setpoint commands


	// --------------------------------------------------------------------------
	//   GET A MESSAGE
	// --------------------------------------------------------------------------
	printf("READ SOME MESSAGES \n");

	// copy current messages
	Mavlink_Messages messages = api.current_messages;

	// local position in ned frame
	//Tu struct "MAVLINK_Messages", tao tin nhan "messages", truyen thong tin current_messages vao bien nay
	//current_messages bao gom cac truong(fields):
	//heartbeat, sys_status, battery, radio, highres_imu, attitude, local_position_ned, pos_target_local...
	//du lieu tu "messages", truyen vao trong struct "mavlink_local_position_ned_t" thong qua field "local_postion_ned"
	mavlink_local_position_ned_t pos = messages.local_position_ned; 
	printf("Got message LOCAL_POSITION_NED (spec: https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED)\n");
	printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

	// hires imu
	mavlink_highres_imu_t imu = messages.highres_imu;
	printf("Got message HIGHRES_IMU (spec: https://mavlink.io/en/messages/common.html#HIGHRES_IMU)\n");
	printf("    ap time:     %llu \n", imu.time_usec);
	printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
	printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
	printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
	printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
	printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
	printf("    temperature: %f C \n"       , imu.temperature );

	printf("\n");


	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------

	return;

}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate,
		bool &use_udp, char *&udp_ip, int &udp_port, bool &autotakeoff)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_control [-d <devicename> -b <baudrate>] [-u <udp_ip> -p <udp_port>] [-a ]";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				i++;
				uart_name = argv[i];
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				i++;
				baudrate = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// UDP ip
		if (strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--udp_ip") == 0) {
			if (argc > i + 1) {
				i++;
				udp_ip = argv[i];
				use_udp = true;
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// UDP port
		if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) {
			if (argc > i + 1) {
				i++;
				udp_port = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Autotakeoff
		if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--autotakeoff") == 0) {
			autotakeoff = true;
		}

	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// port
	try {
		port_quit->stop();
	}
	catch (int error){}

	// end program here
	exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}
