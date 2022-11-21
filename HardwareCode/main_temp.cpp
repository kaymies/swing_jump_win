#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "BezierCurve.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"

#define BEZIER_ORDER_FOOT    7
#define NUM_INPUTS (12 + 2*(BEZIER_ORDER_FOOT+1))
#define NUM_OUTPUTS 19

#define PULSE_TO_RAD (2.0f*3.14159f / 1200.0f)
# define PI           3.14159265358979323846  /* pi */

// Initializations
Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment

QEI encoderA(PE_9,PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderB(PA_5, PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderC(PC_6, PC_7, NC, 1200, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING);// MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding)

MotorShield motorShield(24000); //initialize the motor shield with a period of 24000 ticks or ~10kHZ
Ticker currentLoop;

// Variables for q1
float current1;
float current_des1 = 0;
float prev_current_des1 = 0;
float current_int1 = 0;
float angle1;
float velocity1;
float duty_cycle1;
float angle1_init;

// Variables for q2
float current2;
float current_des2 = 0;
float prev_current_des2 = 0;
float current_int2 = 0;
float angle2;
float velocity2;
float duty_cycle2;
float angle2_init;

// Fixed kinematic parameters
//const float l_OA=.011; 
//const float l_OB=.042; 
//const float l_AC=.096; 
//const float l_DE=.091;
const float l_OA= 0.0; 
const float l_OB=0.0; // our l3    
const float l_AC=.096; // our l2, same as l1
const float l_DE=.091; // our l1
const float ls= 0.0548;
const float l3 = 0.05584;

// Timing parameters
float current_control_period_us = 200.0f;     // 5kHz current control loop
float impedance_control_period_us = 1000.0f;  // 1kHz impedance control loop
float start_period, traj_period, end_period;

// Control parameters
float current_Kp = 4.0f;         
float current_Ki = 0.4f;           
float current_int_max = 3.0f;       
float duty_max;      
float K_xx;
float K_yy;
float K_xy;
float D_xx;
float D_xy;
float D_yy;
float tih;
float tis;
float tipre;
float thpre;
float th2_limu;
float tihend;
bool jumped = false;

//float K_1;
//float D_1;

// Model parameters
float supply_voltage = 12;     // motor supply voltage
float R = 2.0f;                // motor resistance
float k_t = 0.18f;             // motor torque constant
float nu = 0.0005;             // motor viscous friction

float max_torque = 10.0;       // Nm

// Current control interrupt function
void CurrentLoop()
{
    // This loop sets the motor voltage commands using PI current controllers with feedforward terms.
    
    //use the motor shield as follows:
    //motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION =1 is backwards.
        
    current1 = -(((float(motorShield.readCurrentA())/65536.0f)*30.0f)-15.0f);           // measure current
    velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;                                  // measure velocity        
    float err_c1 = current_des1 - current1;                                             // current errror
    current_int1 += err_c1;                                                             // integrate error
    current_int1 = fmaxf( fminf(current_int1, current_int_max), -current_int_max);      // anti-windup
    float ff1 = R*current_des1 + k_t*velocity1;                                         // feedforward terms
    duty_cycle1 = (ff1 + current_Kp*err_c1 + current_Ki*current_int1)/supply_voltage;   // PI current controller
    
    float absDuty1 = abs(duty_cycle1);
    if (absDuty1 > duty_max) {
        duty_cycle1 *= duty_max / absDuty1;
        absDuty1 = duty_max;
    }    
    if (duty_cycle1 < 0) { // backwards
        motorShield.motorAWrite(absDuty1, 1);
    } else { // forwards
        motorShield.motorAWrite(absDuty1, 0);
    }             
    prev_current_des1 = current_des1; 
    
    current2     = -(((float(motorShield.readCurrentB())/65536.0f)*30.0f)-15.0f);       // measure current
    velocity2 = encoderB.getVelocity() * PULSE_TO_RAD;                                  // measure velocity  
    float err_c2 = current_des2 - current2;                                             // current error
    current_int2 += err_c2;                                                             // integrate error
    current_int2 = fmaxf( fminf(current_int2, current_int_max), -current_int_max);      // anti-windup   
    float ff2 = R*current_des2 + k_t*velocity2;                                         // feedforward terms
    duty_cycle2 = (ff2 + current_Kp*err_c2 + current_Ki*current_int2)/supply_voltage;   // PI current controller
    
    float absDuty2 = abs(duty_cycle2);
    if (absDuty2 > duty_max) {
        duty_cycle2 *= duty_max / absDuty2;
        absDuty2 = duty_max;
    }    
    if (duty_cycle2 < 0) { // backwards
        motorShield.motorBWrite(absDuty2, 1);
    } else { // forwards
        motorShield.motorBWrite(absDuty2, 0);
    }             
    prev_current_des2 = current_des2; 
    
}

int main (void)
{
    
    // Object for 7th order Cartesian foot trajectory
    BezierCurve rDesFoot_bez(2,BEZIER_ORDER_FOOT);
    
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    pc.printf("%f",input_params[0]);
    
    while(1) {
        
        // If there are new inputs, this code will run
        if (server.getParams(input_params,NUM_INPUTS)) {
            
                        
            // Get inputs from MATLAB          
            start_period                = input_params[0];    // First buffer time, before trajectory
            traj_period                 = input_params[1];    // Trajectory time/length
            end_period                  = input_params[2];    // Second buffer time, after trajectory
    
            angle1_init                 = input_params[3];    // Initial angle for q1 (rad)
            angle2_init                 = input_params[4];    // Initial angle for q2 (rad)

            K_xx                        = input_params[5];    // Foot stiffness N/m
            K_yy                        = input_params[6];    // Foot stiffness N/m
            K_xy                        = input_params[7];    // Foot stiffness N/m
            D_xx                        = input_params[8];    // Foot damping N/(m/s)
            D_yy                        = input_params[9];    // Foot damping N/(m/s)
            D_xy                        = input_params[10];   // Foot damping N/(m/s)
            duty_max                    = input_params[11];   // Maximum duty factor
            
            tis                         = input_params[12];   // Start of shoulder trajectory
            tih                         = input_params[13];   // Start of hip trajectory
            tipre                       = input_params[14];   // Start preload time
            thpre                       = input_params[15];   // Preload angle
            th2_limu                    = input_params[16];   // Hip max angle 
            tihend                      = input_params[17];   //hip end time
            
            
            // Get foot trajectory points
            float foot_pts[2*(BEZIER_ORDER_FOOT+1)];
            for(int i = 0; i<2*(BEZIER_ORDER_FOOT+1);i++) {
              foot_pts[i] = input_params[18+i];    
            }
            rDesFoot_bez.setPoints(foot_pts);
            
            // Attach current loop interrupt
            currentLoop.attach_us(CurrentLoop,current_control_period_us);
                        
            // Setup experiment
            t.reset();
            t.start();
            encoderA.reset();
            encoderB.reset();
            encoderC.reset();
            encoderD.reset();

            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
                         
            // Run experiment
            while( t.read() < start_period + traj_period + end_period) { 
                 
                // Read encoders to get motor states
                angle1 = encoderA.getPulses() *PULSE_TO_RAD + angle1_init;       
                velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;
                 
                angle2 = encoderB.getPulses() * PULSE_TO_RAD + angle2_init;       
                velocity2 = encoderB.getVelocity() * PULSE_TO_RAD;           
                
                const float th1 = angle1;
                const float th2 = angle2;
                const float dth1= velocity1;
                const float dth2= velocity2;
 
                // Calculate the Jacobian
//                float Jx_th1 = l_AC*cos(th1 + th2) + l_DE*cos(th1) + l_OB*cos(th1);
//                float Jx_th2 = l_AC*cos(th1 + th2);
//                float Jy_th1 = l_AC*sin(th1 + th2) + l_DE*sin(th1) + l_OB*sin(th1);
//                float Jy_th2 = l_AC*sin(th1 + th2);
                
                float Jx_th1 = ls*cos(th1);
                float Jy_th1 = ls*sin(th1);
                                
                // Calculate the forward kinematics (position and velocity)
                //float xFoot = l_AC*sin(th1 + th2) + l_DE*sin(th1) + l_OB*sin(th1);
//                float yFoot = -l_AC*cos(th1 + th2) - l_DE*cos(th1) - l_OB*cos(th1);
//                float dxFoot = Jx_th1*dth1 + Jx_th2*dth2;
//                float dyFoot = Jy_th1*dth1 + Jy_th2*dth2; 
                
                float xFoot = ls*sin(th1);
//                float yFinger = y + l3 - ls*cos(th1); // CHECK, how get y since not calculating
                float dxFoot = Jx_th1*dth1;
//                float dyFinger = Jy_th1*dth1;

                // Set gains based on buffer and traj times, then calculate desired x,y from Bezier trajectory at current time if necessary
                float teff  = 0;
                float vMult = 0;
                if( t < start_period) {
                    if (K_xx > 0 || K_yy > 0) {
                        K_xx = 1; // for joint space control, set this to 1; for Cartesian space control, set this to 50
                        K_yy = 1; // for joint space control, set this to 1; for Cartesian space control, set this to 50
                        D_xx = 0.1;  // for joint space control, set this to 0.1; for Cartesian space control, set this to 2
                        D_yy = 0.1;  // for joint space control, set this to 0.1; for Cartesian space control, set this to 2
                        K_xy = 0;
                        D_xy = 0;
                    }
                    teff = 0;
                }
                else if (t < start_period + traj_period)
                {
                    K_xx = input_params[5];  // Foot stiffness N/m
                    K_yy = input_params[6];  // Foot stiffness N/m
                    K_xy = input_params[7];  // Foot stiffness N/m
                    D_xx = input_params[8];  // Foot damping N/(m/s)
                    D_yy = input_params[9];  // Foot damping N/(m/s)
                    D_xy = input_params[10]; // Foot damping N/(m/s)
                    teff = (t-start_period);
                    vMult = 1;
                }
                else
                {
                    teff = traj_period;
                    vMult = 0;
                }
                
                // Get desired foot positions and velocities
                float rDesFoot[2] , vDesFoot[2];
                rDesFoot_bez.evaluate(teff/traj_period,rDesFoot);
                rDesFoot_bez.evaluateDerivative(teff/traj_period,vDesFoot);
                vDesFoot[0]/=traj_period;
                vDesFoot[1]/=traj_period;
                vDesFoot[0]*=vMult;
                vDesFoot[1]*=vMult;
                
                // Calculate the inverse kinematics (joint positions and velocities) for desired joint angles              
                float xFoot_inv = -rDesFoot[0];
                float yFoot_inv = rDesFoot[1];                
                float l_OE = sqrt( (pow(xFoot_inv,2) + pow(yFoot_inv,2)) );
                float alpha = abs(acos( (pow(l_OE,2) - pow(l_AC,2) - pow((l_OB+l_DE),2))/(-2.0f*l_AC*(l_OB+l_DE)) ));
//                float th1_des = -((3.14159f/2.0f) + atan2(yFoot_inv,xFoot_inv) - abs(asin( (l_AC/l_OE)*sin(alpha) )));
//                float th2_des = -(3.14159f - alpha); 

                float th1_des = acos(xFoot_inv/ls);

                
                //float dd = (Jx_th1*Jy_th2 - Jx_th2*Jy_th1);
//                float dth1_des = (1.0f/dd) * (  Jy_th2*vDesFoot[0] - Jx_th2*vDesFoot[1] );
//                float dth2_des = (1.0f/dd) * ( -Jy_th1*vDesFoot[0] + Jx_th1*vDesFoot[1] );

//                float dd = (Jx_th1*Jy_th2 - Jx_th2*Jy_th1);
                float dth1_des = vDesFoot[0]/Jx_th1;
//                float dth2_des = (1.0f/dd) * ( -Jy_th1*vDesFoot[0] + Jx_th1*vDesFoot[1] );
        
                // Calculate error variables
                float e_x = rDesFoot[0] - xFoot;
//                float e_y = rDesFoot[1] - yFoot;
                float de_x = vDesFoot[0] - dxFoot;
//                float de_y = vDesFoot[1] - dyFoot;
        
                // Calculate virtual force on foot
                //float fx = K_xx*e_x + K_xy*e_y + D_xx*de_x + D_xy*de_y;
//                float fy = K_xy*e_x + K_yy*e_y + D_xy*de_x + D_yy*de_y;
                                
                // Set desired currents           
//                current_des1 = (-K_xx*th1 - D_xx*dth1)/k_t;          
//                current_des2 = 0;  
//                current_des1 = 0;         
//                current_des2 = (-K_yy*th2 - D_yy*dth2)/k_t; 
//                current_des1 = 0;
//                current_des2 = 0;
                 
        
                // Joint impedance
                // sub Kxx for K1, Dxx for D1, Kyy for K2, Dyy for D2
                // Note: Be careful with signs now that you have non-zero desired angles!
                // Your equations should be of the form i_d = K1*(q1_d - q1) + D1*(dq1_d - dq1)
                
                
                
                th1_des = PI;
                dth1_des = 0;
                
                pc.printf("jumped %d \n\r", jumped);
                
                if (t.read() >= tis) {
                    current_des1 = (-K_xx*(th1 - th1_des) - D_xx*(dth1 - dth1_des))/k_t;    
                } else {
                    current_des1 = 0.0;
                }  
//                current_des2 = (-K_yy*(th2 - th2_des) - D_yy*(dth2 - dth2_des))/k_t; 
                  
                
                if (t.read() >= tipre && t.read() <= tih) {
                    
                    current_des2 = -1*(-K_yy*(th2 - thpre) - D_yy*(dth2))/k_t; // negative sign so goes clockwise
                    
                } else if (t.read() >= tih  && jumped == false) { //&& th2 > th2_limu
                    current_des2 = -max_torque/k_t;
//                    current_des2 = 0;
                    if (th2 <= th2_limu)  {
                        jumped = true;
                        pc.printf("got here");
                    }
                    pc.printf("th2: %f, th2_limu: %f \n\r", th2, th2_limu);
                } else  {
                    current_des2 = 0.0;
                }

                if (t.read() >= tihend)  {
                    current_des2 = 0.0;
                }            
                           
                // Cartesian impedance  
                // Note: As with the joint space laws, be careful with signs!              
//                current_des1 = (Jx_th1*fx + Jy_th1*fy)/k_t;          
//                current_des2 = (Jx_th2*fx + Jy_th2*fy)/k_t;   
                
                
                // Form output to send to MATLAB     
                float output_data[NUM_OUTPUTS];
                // current time
                output_data[0] = t.read();
                // motor 1 state
                output_data[1] = angle1;
                output_data[2] = velocity1;  
                output_data[3] = current1;
                output_data[4] = current_des1;
                output_data[5] = duty_cycle1;
                // motor 2 state
                output_data[6] = angle2;
                output_data[7] = velocity2;
                output_data[8] = current2;
                output_data[9] = current_des2;
                output_data[10]= duty_cycle2;
                // foot state
                output_data[11] = xFoot;
                output_data[12] = 0.0; // CHECK - FIX LATER used to be yFoot
                output_data[13] = dxFoot;
                output_data[14] = 0.0; // CHECK - FIX LATER used to be dyFoot
                output_data[15] = rDesFoot[0];
                output_data[16] = rDesFoot[1];
                output_data[17] = vDesFoot[0];
                output_data[18] = vDesFoot[1];
                
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);

                wait_us(impedance_control_period_us);   
            }
            
            // Cleanup after experiment
            server.setExperimentComplete();
            currentLoop.detach();
            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
        
        } // end if
        
    } // end while
    
} // end main



