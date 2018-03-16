
/*******************************************************************************
* This program retrieves the angle estimates from the IMU and passes the values
* through a complementary filter to for a more accurate description of the angle
* and then uses a body angle reference set point to balance around. Two controllers
* are arranged in a successive loop closure to make the robot stay in the same
* position. The sampling rate of the IMU is 100 Hz and the inner loop runs at 100 Hz
* while the outer loop runs at 20 Hz. The program prints to the screen at
* 10 Hz.
*******************************************************************************/

// usefulincludes is a collection of common system includes
#include <rc_usefulincludes.h>
// main roboticscape API header
#include <roboticscape.h>

//Define inner and outer loop timesteps
#define INNER_TIMESTEP 0.01
#define OUTER_TIMESTEP 0.05

//Define D1 gain tuning factor, overload and reset factor
#define D1_GAIN 1.005 //Tuned for good performance
#define D1_OVERLOAD_TIMEOUT 50
#define D1_RESET_TIMEOUT 200

//Define D2 gain tuning factor
#define D2_GAIN 1.0 //No tuning needed

#define STEERING_GAIN 0.25
#define ANGLE_OFFSET 0.31
#define START_ANGLE_RANGE 0.2
#define MAX_LEAN                1.2

//Motor and encoder controls
#define L_MOTOR_POLARITY 1
#define R_MOTOR_POLARITY -1

#define R_MOTOR_CHANNEL 2
#define L_MOTOR_CHANNEL 3

#define R_ENCODER_CHANNEL 2
#define L_ENCODER_CHANNEL 3
#define L_ENCODER_POLARITY 1
#define R_ENCODER_POLARITY -1

#define GEARBOX 4*15*35.555


//Function declarations
void on_pause_pressed();
void on_pause_released();
void* print_thread_func();
void* outer_loop_thread_func();
void inner_loop();
void reset_controller();


//Define an armstate enum, used to turn various loops and checks on/off.
typedef enum arm_state_t{
        DISARMED,
        ARMED
}arm_state_t;

arm_state_t armstate=DISARMED;
//Initialize inner loop body angle variables.
 double theta_a_raw = 0;
 double theta_g_raw = 0;
 double theta_g_raw_last = 0;
 double theta_a = 0;
 double theta_f = 0;
 double theta_g = 0;

//Initialize body angle error variables
double theta_e = 0;
double theta_e_last = 0;
double theta_e_last_last = 0;

//Initialize body angle reference
double theta_ref = 0;
double theta_ref_last = 0;
double theta_ref_last_last = 0;

//Initialize inner loop overload and reset timer
int inner_overload_timer = 0;
int inner_reset_timer = 0;
//Initialize duty cycle variables
double u = 0;
double u_last = 0;
double u_last_last = 0;

//Initialize outer loop setpoint variable
double phi_ref = 0;
double phi = 0;
double phi_R = 0;
double phi_L = 0;
double phi_diff = 0;
double steerinput = 0;

//Initialize outer loop setpoint errors
double phi_e = 0;
double phi_e_last = 0;
double phi_e_last_last = 0;


//Struct to hold imu data
rc_imu_data_t imudata;

/*******************************************************************************
* int main()
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - initialize IMU and threads
* - main while loop that checks for EXITING condition
* - rc_cleanup() at the end
*******************************************************************************/
int main(){
        //Always initialize cape library first
        if(rc_initialize()){
                fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
                return -1;
        }

        //Connect buttons to functions
        rc_set_pause_pressed_func(&on_pause_pressed);
        rc_set_pause_released_func(&on_pause_released);

        //Configure the IMU
        rc_imu_config_t conf = rc_default_imu_config();

        //Set up the imu for interrupt operation
      if(rc_initialize_imu_dmp(&imudata, conf)){ //Points at the data struct where we store the imu-values and to the configurations of the imu stored in conf
                fprintf(stderr,"rc_initialize_imu_failed\n");
                return -1;
        }
        if(rc_read_accel_data(&imudata)<0){
                printf("read accel data failed\n");
        }
        printf("|Angle|Motor input|Position|\n");

        theta_a = atan2(-imudata.accel[2],imudata.accel[1]); //Set the output of low pass filter to the current accelerometer value to account for steady state

         //Create thread for printing angle estimates
        pthread_t print_thread;
        pthread_create(&print_thread, NULL, print_thread_func, (void*) NULL);

        //Set priority of thread
        struct sched_param params_print_thread;
        params_print_thread.sched_priority = 1;
        pthread_setschedparam(print_thread, SCHED_FIFO,&params_print_thread);

        //Make thread for outer loop (setpoint control)
        pthread_t outer_loop_thread;
        pthread_create(&outer_loop_thread, NULL, outer_loop_thread_func, (void*) NULL);

        //Set priority of thread
        struct sched_param params_outer_loop_thread;
        params_outer_loop_thread.sched_priority = 10;
        pthread_setschedparam(outer_loop_thread, SCHED_FIFO,&params_outer_loop_thread);

        //Give the IMU an interrupt function
        rc_set_imu_interrupt_func(&inner_loop);

        rc_set_state(RUNNING);


        // Keep looping until state changes to EXITING
        while(rc_get_state() != EXITING){
                if(armstate==ARMED) {
                        rc_set_led(GREEN,ON);
                        rc_set_led(RED,OFF);
                }

                else if(armstate==DISARMED) {
                        rc_set_led(GREEN,OFF);
                        rc_set_led(RED,ON);
                }
              rc_usleep(1000000);
        }

        //Exit cleanly
        printf("Waiting for print thread to join \n");
        pthread_join(print_thread, NULL);
        printf("Print thread joined \n");
        printf("Waiting for setpoint thread to join \n");
        pthread_join(outer_loop_thread, NULL);
        printf("Outer loop thread joined \n");

        rc_power_off_imu();
        rc_cleanup();
        return 0;
}

//The interrupt function is called everytime the IMU has new data available (100 Hz) and filters the data that is collected by the imu
void inner_loop() {
        //Complementary filter cross-over frequency as a static variable which means that it is only instantiated once.
        static double wcfilter = 0.5;

        theta_a_raw = atan2(-imudata.accel[2],imudata.accel[1]);
        theta_g_raw_last = theta_g_raw;
    theta_g_raw = theta_g_raw + imudata.gyro[0]*PI/180*INNER_TIMESTEP;

        theta_a = -(wcfilter*INNER_TIMESTEP-1)*theta_a + wcfilter*INNER_TIMESTEP*theta_a_raw;

        theta_g = -(wcfilter*INNER_TIMESTEP-1)*theta_g + theta_g_raw  - theta_g_raw_last;

        theta_f = theta_a + theta_g+ANGLE_OFFSET;

        // INNER LOOP CONTROLLER
        theta_e_last_last = theta_e_last;
        theta_e_last= theta_e;
        theta_e = theta_ref-theta_f;

        //Evaluate inner loop difference equation
        u_last_last = u_last;
        u_last = u;
        u = D1_GAIN*(1.569*u_last-0.5695*u_last_last-2.589*theta_e+4.602*theta_e_last-2.037*theta_e_last_last);

        //Prevent windup by setting max and min values for the motor input
        if(u>=1){
              u=1;
        }
        if(u<=-1){
              u=-1;
        }

        /**************************
        Perform checks on status of system, and react accordingly
        **************************/

        //Turn off motor if exiting
        if(rc_get_state()==EXITING){
        rc_disable_motors();
        return;
        }

        /*nitiate a reset timer that checks if the robot is lying down, ready to be picked up to balance.
        This is used to prevent the motors from running when the robot is picked up */
        if(theta_f > -1.7 && theta_f < -1.5){
                inner_reset_timer++;
       }

        if(fabs(theta_f) < START_ANGLE_RANGE && armstate == DISARMED) {
                 if(inner_reset_timer > D1_RESET_TIMEOUT){
                        reset_controller();
                        rc_enable_motors();
                    armstate = ARMED;
                        inner_reset_timer = 0;
                }
        }

        //If the robot is disarmed, do nothing
        if(armstate == DISARMED) {
        return;
        }


        //If robot has tipped past point of no return, stop trying
        if(fabs(theta_f)>MAX_LEAN){
                printf("\r I have fallen over. Need help! \n");
                reset_controller();
                rc_disable_motors();
                armstate = DISARMED;
        }

        //Initiate check if the robot has gone stuck, to prevent motors from breaking
        if(fabs(u)>0.95){
                 inner_overload_timer++;
        }
   else{
                 inner_overload_timer=0;
        }
        if(inner_overload_timer > D1_OVERLOAD_TIMEOUT){
                printf("\r I have gone stuck. Rescue me! \n");
                reset_controller();
                rc_disable_motors();
                armstate = DISARMED;
                inner_overload_timer = 0;
                return;
        }


        //Proportional steering controller to keep robot pointing in approximately the right direction
        steerinput=STEERING_GAIN*phi_diff;

        rc_set_motor(L_MOTOR_CHANNEL, L_MOTOR_POLARITY*(u-steerinput));
        rc_set_motor(R_MOTOR_CHANNEL, R_MOTOR_POLARITY*(u+steerinput));

        return;
        }

void* print_thread_func(){
   while(rc_get_state() != EXITING) {
        printf("\r");
        printf("|%6.3f |%6.3f |%6.3f |", theta_f, u, phi);
        fflush(stdout);
        rc_usleep(100000);
        }
        return NULL;
}

void* outer_loop_thread_func(){
        while(rc_get_state() != EXITING){
        phi_R = (R_ENCODER_POLARITY*rc_get_encoder_pos(R_MOTOR_CHANNEL)*2*PI)/(GEARBOX);
        phi_L = (L_ENCODER_POLARITY*rc_get_encoder_pos(L_MOTOR_CHANNEL)*2*PI)/(GEARBOX);

        //Take average of left and right encoder and add body angle
        phi = (phi_R+phi_L)/2 +theta_f;

        //Difference between wheel positions for use in steering controller
        phi_diff = phi_L-phi_R;

        phi_e_last_last = phi_e_last;
        phi_e_last = phi_e;
        phi_e = phi_ref - phi;

        theta_ref_last_last = theta_ref_last;
        theta_ref_last = theta_ref;

        //Outer loop difference equation
        theta_ref = D2_GAIN*(1.397*theta_ref_last - 0.3972*theta_ref_last_last + 0.1233*phi_e - 0.2428*phi_e_last + 0.1195*phi_e_last_last);
        rc_usleep(OUTER_TIMESTEP*1000000);
        }
        return NULL;
}

/*******************************************************************************
* void on_pause_pressed()
*
* If the user holds the pause button for 2 seconds, set state to exiting which
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void on_pause_pressed(){
        int i=0;
        const int samples = 100;        // check for release 100 times in this period
        const int us_wait = 2000000; // 2 seconds

        // now keep checking to see if the button is still held down
     for(i=0;i<samples;i++){
                rc_usleep(us_wait/samples);
                if(rc_get_pause_button() == RELEASED) return;
        }
        printf("long press detected, shutting down\n");
        rc_set_state(EXITING);
        return;
}

void on_pause_released(){
        // toggle betewen armed and disarmed modes
        if (armstate==ARMED){
                 reset_controller();
                 rc_disable_motors();
                armstate = DISARMED;
        }
        else if (armstate==DISARMED){
                if(fabs(theta_f) < START_ANGLE_RANGE) {
                 reset_controller();
                rc_enable_motors();
                armstate = ARMED;
                }
        }
      return;
}

void reset_controller(){
        //Reset encoders and all variables to reset controller
        rc_set_encoder_pos(R_MOTOR_CHANNEL,0);
        rc_set_encoder_pos(L_MOTOR_CHANNEL,0);

        phi_e = 0;
        phi_e_last = 0;
        phi_e_last_last = 0;

        theta_ref = 0;
        theta_ref_last = 0;
        theta_ref_last_last = 0;

        u = 0;
        u_last = 0;
        u_last_last = 0;

        theta_e = 0;
        theta_e_last = 0;
        theta_e_last_last = 0;
}
