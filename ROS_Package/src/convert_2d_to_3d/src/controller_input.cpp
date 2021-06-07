#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "convert_2d_to_3d/robot_pose.h"
#include "omni_bot_msg/omnibot.h"

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <cmath>

using namespace std;


#define NB -1
#define NM -0.666
#define NS -0.333
#define ZO 0
#define PS 0.333
#define PM 0.666
#define PB 1


#define distance 0.316

double U_delta();

omni_bot_msg::omnibot wheel_w_msg;


#define dt 0.05;    //50ms
double m_x_error=0;
double m_y_error=0;
double m_theta_error=0;


double m_previous_x_error=0;
double m_previous_y_error=0;
double m_previous_theta_error=0;

double m_y_error_dot=0;
double m_x_error_dot=0;
double m_theta_error_dot=0;


double m_output_vx=0;
double m_output_vy=0;
double m_output_w=0;


double vm_0=0;
double vm_1=0;
double vm_2=0;

/*
                                //NB,NM,NS,0,PS,PM,PB
const double Fuzzy_Rule[7][7]= {{NB,NB,NB,NM,NM,NS,ZO},  //error NB
                                {NB,NM,NM,NM,NS,ZO,PS},  //error NM
                                {NM,NS,NS,NS,PS,PS,PM}, //error NS
                                {PM,PS,ZO,ZO,ZO,NS,NM}, //error 0`
                                {NM,NS,NS,PS,PS,PS,PM}, //error PS
                                {NS,ZO,PS,PM,PM,PM,PB},  //error PM
                                {PB,PS,PM,PB,PB,PB,PB}};//error PB
                                */


// //                                //NB,NM,NS,0,PS,PM,PB
const double Fuzzy_Rule[7][7]= {{NS,NM,NB,NB,NB,NM,NS},  //error NB
                                {NS,NS,NM,NM,NM,NS,NS},  //error NM
                                {NS,NS,NM,NS,NM,NS,NS}, //error NS
                                {ZO,ZO,ZO,ZO,ZO,ZO,ZO}, //error 0`
                                {PS,PS,PM,PS,PM,PS,PS}, //error PS
                                {PS,PS,PM,PM,PM,PS,PS},  //error PM
                                {PS,PM,PB,PB,PB,PM,PS}};//error PB


//                             //ed NB,NM,NS,0,PS,PM,PB        PI
// const double Fuzzy_Rule[7][7]= {{ZO,ZO,NS,NB,PS,ZO,ZO},  //error NB
//                                 {ZO,ZO,NS,NM,PS,ZO,ZO},  //error NM
//                                 {ZO,ZO,NS,NS,PS,ZO,ZO}, //error NS
//                                 {NS,NS,NS,ZO,PS,PS,PS}, //error 0`
//                                 {ZO,ZO,NS,PS,PS,ZO,ZO}, //error PS
//                                 {ZO,ZO,NS,PM,PS,ZO,ZO},  //error PM
//                                 {ZO,ZO,NS,PB,PS,ZO,ZO}};//error PB
//error is possible , error dot is more possivle -> error is more up


static int mode = 0;    //1 == pi fuzzy control

// NB ~ NM
static double match_rule11[]={0,-1/(NM-NB),NM/(NM-NB)};
static double match_rule12[]={1,1/(NM-NB),-NB/(NM-NB)};

//NM ~ NS
static double match_rule21[]={1,-1/(NS-NM),NS/(NS-NM)};
static double match_rule22[]={2,1/(NS-NM),-NS/(NS-NM)};

//NS ~ ZO
static double match_rule31[]={2,1/NS,0};
static double match_rule32[]={3,-1/NS,1};

//ZO ~ PS
static double match_rule41[]={3,-1/PS,1};
static double match_rule42[]={4,1/PS,0};

//PS ~ PM
static double match_rule51[]={4,-1/(PM-PS),PM/(PM-PS)};
static double match_rule52[]={5,1/(PM-PS),-PS/(PM-PS)};

//PM~PB
static double match_rule61[]={5,-1/(PB-PM),PB/(PB-PM)};
static double match_rule62[]={6,1/(PB-PM),-PM/(PB-PM)};

//input max min-----------------------------------------------
double X_max=2;
double X_min =0;

double Y_max=2;
double Y_min =0;

//double theta_min = -90;
//-----------------------------------------------------------


//output max min---------------------------------------------
#define V_x_max 0.46//0.46        //  m/s
double V_x_min = -0.46;         //  m/s


double V_x_dot_max = 0.46/0.02;     // 23

#define V_y_max 0.4 //0.4        //  m/s
double V_y_min = -0.4;

double V_y_dot_max=0.4/0.02;        //20

double theta_max = 130;//362; change 362 -> 180 becuase of w is low

double W_max=1.264;            //  rad/s  1.264 is max    
double W_min=-1.264;


double W_dot_max=0.1264/0.02;


double out = 0;
//-----------------------------------------------------------


//input error, input dot error------------------------------
/*
double X_error;
double X_delta_error;
double Y_error;
double Y_delta_error;
double theta_error;
double theta_delta_error;
*/
//-----------------------------------------------------------
int m_mode=0;

double x_now=0;
double y_now =0;
double theta_now=0;


//fuzzy, fuzzy match
double first_Fuzzy_High[2]={0,0};
double first_Fuzzy_Low[2]={0,0};
double second_Fuzzy_High[2]={0,0};
double second_Fuzzy_Low[2]={0,0};


void fuzzy_first_check(double error);
void fuzzy_second_check(double error_dot);


//callback -> robot_pose
//fuzzy controller
class ControlInputNode
{
    public:
        ControlInputNode()
        {
            sub_robot_pose = nh_.subscribe<convert_2d_to_3d::robot_pose>("robot_pose",1 ,&ControlInputNode::rosbot_pose_callback, this);
            pub_wheel_speed = nh_.advertise<omni_bot_msg::omnibot>("CCR",1000);

			
            // pub_result_ = nh_.advertise<convert_2d_to_3d::Result>("detected_object", 1);
            ROS_INFO("[%s] initialized...", ros::this_node::getName().c_str());
        }

    // #define MODE_FLAG_LP	    	0
    // #define MODE_FLAG_STRAIGHT   1
    // #define MODE_FLAG_STOP 		2
    // #define MODE_FLAG_ELEVATION  3
    // #define MODE_FLAG_BACK 		4

	void rosbot_pose_callback(const convert_2d_to_3d::robot_pose::ConstPtr& pose) {
            //20ms 

        
        m_mode = pose->mode;

        switch(m_mode)
        {
            case 0: // Mode : MODE_FLAG_LP
                m_x_error = pose->dis_x;
                m_y_error = pose->dis_y;
                m_theta_error = pose->theta;

                cout << "\n\n-------------------------------------" << endl;
                //cout << "dis_y is this : " << m_y_error << endl;

                m_x_error_dot = (m_previous_x_error-m_x_error)/0.02;
                m_y_error_dot = (m_previous_y_error-m_y_error)/0.02;        //0.02 is dt
                m_theta_error_dot = (m_previous_theta_error-m_theta_error)/0.02;

                cout << "m_x_error_dot : " << m_x_error_dot << endl;
                cout << "m_y_error_dot : " << m_y_error_dot << endl;
                cout << "m_theta_error_dot : " << m_theta_error_dot << endl;

                //std::cout<< "x : " << m_x_error <<std::endl;
                //std::cout<< "y : " << m_y_error <<std::endl;
                //std::cout<< "theta : " << m_theta_error <<std::endl;


                cout << ">>>>>>>>>>>>>>>>>>>>fuzzy check<<<<<<<<<<<<<<<<<<<" << endl;
                if(m_x_error == 0)
                {
                    m_output_vx = 0;

                }else
                {
                    cout << "X fuzzy " << endl;
                    fuzzy_first_check(m_x_error/X_max); //error

                    cout << "\n X dot fuzzy" << endl;
                    fuzzy_second_check(m_x_error_dot/V_x_dot_max);  //error dot

                
                
                    if(mode == 1)
                        m_output_vx =+ U_delta();
                    else
                        m_output_vx = U_delta();
                }

                //cout << "x fuzzy first check" << m_x_error/V_x_max<< endl;
                //cout << "x fuzzy second check" << m_x_error_dot/V_x_dot_max<<endl;
                

                if(m_y_error == 0)
                {
                    m_output_vy = 0;

                }else
                {
                    cout << "\n\n Y fuzzy" << endl;
                    fuzzy_first_check(m_y_error/Y_max); //error
                    cout << "\n Y dot fuzzy" << endl;
                    fuzzy_second_check(m_y_error_dot/V_y_dot_max);  //error dot
                    
                    if(mode == 1)
                        m_output_vy =+ U_delta();
                    else
                        m_output_vy = U_delta();
                }


                /*
                printf("\nthis is y fuzzy first is [%f]\n",m_y_error/V_y_max);
                printf("this is y fuzzy second is [%f]\n\n",m_y_error_dot/V_y_dot_max);
                cout << "m_y_error" << m_y_error<< endl;
                cout << "m_y_error_dot" << m_y_error_dot << endl;
                cout << "V_y_dot_max" << V_y_dot_max << endl;
                */

                if(m_theta_error == 0)
                {
                    m_output_w = 0;
                }else
                {
                    cout << "\n\n Theta fuzzy" << endl;
                    fuzzy_first_check(m_theta_error/theta_max); //error

                    cout << "\n Theta dot fuzzy" << endl;
                    fuzzy_second_check(m_theta_error_dot/theta_max);  //error dot

                    if(mode == 1)
                        m_output_w =+ U_delta();
                    else
                        m_output_w = U_delta();

                }

                // cout << "<<<<<<<<<<<<<<<<<<fuzzy end >>>>>>>>>>>>>>>>>>\n\n" << endl;

                m_previous_y_error=m_y_error;
                m_previous_x_error=m_x_error;
                m_previous_theta_error=m_theta_error;

                /*
                cout << "m_previous x error is " << m_previous_x_error << endl;
                cout << "m_previous y error is " << m_previous_y_error << endl;
                cout << "m_previous theta error is " << m_previous_theta_error << endl;


                printf("theta = ---- [%f] ----",m_theta_error);
                */

                // printf("\n\noutput v_x = [%f]\n",m_output_vx);
                // printf("output v_y = [%f]\n",m_output_vy);
                // printf("output v_w = [%f]\n\n",m_output_w);
                

                vm_0 = -0.866*m_output_vx*V_x_max+0.5*m_output_vy*V_y_max+distance*m_output_w*W_max;
                vm_1 = -m_output_vy*V_y_max+distance*m_output_w*W_max;
                vm_2 = 0.866*m_output_vx*V_x_max+0.5*m_output_vy*V_y_max+distance*m_output_w*W_max;

                cout << "x_velocity is " << m_output_vx*V_x_max << endl;
                cout << "y_velocity is " << m_output_vy*V_y_max << endl;
                cout << "w verlocity is" << m_output_w*W_max << endl;

                // cout << "double w0 x 100 : " << vm_0*20<< endl;
                // cout << "double w1 x 100 : " << vm_1*20<< endl;
                // cout << "double w2 x 100 : " << vm_2*20<< endl;

                // if(vm_0 < 0.9 && vm_0>-0.9)
                // {
                //     vm_0 = 0;
                // }
                // if(vm_1 < 0.9 && vm_1>-0.9)
                // {
                //     vm_1 = 0;
                // }
                // if(vm_2 < 0.9 && vm_2>-0.9)
                // {
                //     vm_2 = 0;
                // }            
                


                wheel_w_msg.Radian_0=vm_0*20.0*10;    //wheel linear V -> w(*20)   *10 to serial
                wheel_w_msg.Radian_1=vm_1*20.0*10;
                wheel_w_msg.Radian_2=vm_2*20.0*10;

                cout << "Mode :  " << pose->mode << endl;
                cout << "W0_velocity is " << wheel_w_msg.Radian_0 << endl;
                cout << "W1_velocity is " << wheel_w_msg.Radian_1 << endl;
                cout << "W2 verlocity is" << wheel_w_msg.Radian_2 << endl;
                
                pub_wheel_speed.publish(wheel_w_msg);
                break;
                ////////
            case 1: // Mode : MODE_FLAG_STRAIGHT
                wheel_w_msg.Radian_0=0;  
                wheel_w_msg.Radian_1=0;
                wheel_w_msg.Radian_2=0;
                wheel_w_msg.Elevation = 1;
                
                cout << "Mode :  " << pose->mode << endl;
                cout << "W0_velocity is " << wheel_w_msg.Radian_0 << endl;
                cout << "W1_velocity is " << wheel_w_msg.Radian_1 << endl;
                cout << "W2 verlocity is" << wheel_w_msg.Radian_2 << endl;
                
                pub_wheel_speed.publish(wheel_w_msg);
                
                break;
            
            // case 2: // Mode : MODE_FLAG_STOP
            //     wheel_w_msg.Radian_0=0;   
            //     wheel_w_msg.Radian_1=0;
            //     wheel_w_msg.Radian_2=0;

                
            //     cout << "W0_velocity is " << wheel_w_msg.Radian_0 << endl;
            //     cout << "W1_velocity is " << wheel_w_msg.Radian_1 << endl;
            //     cout << "W2 verlocity is" << wheel_w_msg.Radian_2 << endl;
                
            //     pub_wheel_speed.publish(wheel_w_msg);
                
            //     break;

            // case 3: // Mode : MODE_FLAG_ELEVATION
            //     wheel_w_msg.Radian_0=0;   
            //     wheel_w_msg.Radian_1=0;
            //     wheel_w_msg.Radian_2=0;
                

                
            //     cout << "W0_velocity is " << wheel_w_msg.Radian_0 << endl;
            //     cout << "W1_velocity is " << wheel_w_msg.Radian_1 << endl;
            //     cout << "W2 verlocity is" << wheel_w_msg.Radian_2 << endl;
                
            //     pub_wheel_speed.publish(wheel_w_msg);

            //     break;

            // case 4: // Mode : MODE_FLAG_BACK
            //     wheel_w_msg.Radian_0=20;   
            //     wheel_w_msg.Radian_1=0;
            //     wheel_w_msg.Radian_2=-20;

                
            //     cout << "W0_velocity is " << wheel_w_msg.Radian_0 << endl;
            //     cout << "W1_velocity is " << wheel_w_msg.Radian_1 << endl;
            //     cout << "W2 verlocity is" << wheel_w_msg.Radian_2 << endl;
                
            //     pub_wheel_speed.publish(wheel_w_msg);

            //     break;

            case 10:    // Mode : ROT_LEFT
                wheel_w_msg.Radian_0=1;   
                wheel_w_msg.Radian_1=1;
                wheel_w_msg.Radian_2=1;

                
                cout << "W0_velocity is " << wheel_w_msg.Radian_0 << endl;
                cout << "W1_velocity is " << wheel_w_msg.Radian_1 << endl;
                cout << "W2 verlocity is" << wheel_w_msg.Radian_2 << endl;
                
                pub_wheel_speed.publish(wheel_w_msg);

                break;
            
            case 11:    // Mode : ROT_RIGHT
                wheel_w_msg.Radian_0=-1;   
                wheel_w_msg.Radian_1=-1;
                wheel_w_msg.Radian_2=-1;

    
                cout << "W0_velocity is " << wheel_w_msg.Radian_0 << endl;
                cout << "W1_velocity is " << wheel_w_msg.Radian_1 << endl;
                cout << "W2 verlocity is" << wheel_w_msg.Radian_2 << endl;
                
                pub_wheel_speed.publish(wheel_w_msg);

                break;
            
            case 12:    // Mode : GO_BACK
                wheel_w_msg.Radian_0=2;   
                wheel_w_msg.Radian_1=0;
                wheel_w_msg.Radian_2=-2;
                
                cout << "W0_velocity is " << wheel_w_msg.Radian_0 << endl;
                cout << "W1_velocity is " << wheel_w_msg.Radian_1 << endl;
                cout << "W2 verlocity is" << wheel_w_msg.Radian_2 << endl;
                
                pub_wheel_speed.publish(wheel_w_msg);

                break;

        }       

        
        
    }

    private:
        ros::NodeHandle nh_;
		ros::Publisher pub_robot_pose;
        ros::Subscriber sub_robot_pose;
        ros::Publisher pub_wheel_speed;

};

double U_delta()
{
    double f=0;
    double w=0;

    double Fuzzy[4]={0,};
    double match[4] ={0,};

    Fuzzy[0] = Fuzzy_Rule[(int)first_Fuzzy_High[0]][(int)second_Fuzzy_High[0]];
    Fuzzy[1] = Fuzzy_Rule[(int)first_Fuzzy_High[0]][(int)second_Fuzzy_Low[0]];
    Fuzzy[2] = Fuzzy_Rule[(int)first_Fuzzy_Low[0]][(int)second_Fuzzy_High[0]];
    Fuzzy[3] = Fuzzy_Rule[(int)first_Fuzzy_Low[0]][(int)second_Fuzzy_Low[0]];

    match[0] = first_Fuzzy_High[1]*second_Fuzzy_High[1];
    match[1] = first_Fuzzy_High[1]*second_Fuzzy_Low[1];
    match[2] = first_Fuzzy_Low[1]*second_Fuzzy_High[1];
    match[3] = first_Fuzzy_Low[1]*second_Fuzzy_Low[1];

/*
    cout <<"Fuzzt[0]" << Fuzzy[0] << endl;
    cout <<"Fuzzt[1]" << Fuzzy[1] << endl;
    cout <<"Fuzzt[2]" << Fuzzy[2] << endl;
    cout <<"Fuzzt[3]" << Fuzzy[3] << endl;
    
    cout << "match [0]" << match[0] << endl;
    cout << "match [1]" << match[1] << endl;
    cout << "match [2]" << match[2] << endl;
    cout << "match [3]" << match[3] << endl;
    
*/


/*
    printf("first_Fuzzy_High[0] : %f\n", first_Fuzzy_High[0]);
    printf("first_Fuzzy_High[1] : %f\n", first_Fuzzy_High[1]);
    printf("first_Fuzzy_Low[0] : %f\n", first_Fuzzy_Low[0]);
    printf("first_Fuzzy_Low[1] : %f\n", first_Fuzzy_Low[1]);

    printf("second_Fuzzy_High[0] : %f\n", second_Fuzzy_High[0]);
    printf("second_Fuzzy_High[1] : %f\n", second_Fuzzy_High[1]);

    printf("second_Fuzzy_Low[0] : %f\n", second_Fuzzy_Low[0]);
    printf("second_Fuzzy_Low[1] : %f\n", second_Fuzzy_Low[1]);


    for(int i=0; i<4; i++)
    {
        printf("Fuzzy[%d] = %f, match = %f\n", i, Fuzzy[i],match[i]);

    }
*/

    for(int i=0; i<4; i++)
    {
        w += match[i];
        f += Fuzzy[i]*match[i];
    }

    //printf("f : %f, w : %f\n",f,w);

    if(w == 0)
    {        
        out = 0;                                                   //because of inf error
    }else
    {
        out = (double)(f/w);
    }
    return out;

}


void fuzzy_first_check(double error)
{

    //cout << "fuzzy_first_check start" << endl;
    

    if(error < NB)
    {
        printf("error<NB\n");
        first_Fuzzy_High[0] = match_rule11[0];
        first_Fuzzy_High[1] = 1.0;
        first_Fuzzy_Low[0] = match_rule12[0];
        first_Fuzzy_Low[1] = 0;
    }
    else if(error >= NB && error < NM)
    {
        printf("NB<=error<NM\n");
        first_Fuzzy_High[0] = match_rule11[0];
        first_Fuzzy_High[1] = match_rule11[1]*error+match_rule11[2];
        first_Fuzzy_Low[0] = match_rule12[0];
        first_Fuzzy_Low[1] = match_rule12[1]*error+match_rule12[2];
    }
    else if(error >= NM && error < NS)
    {
        printf("NM<=error<NS\n");

        first_Fuzzy_High[0] = match_rule21[0];
        first_Fuzzy_High[1] = match_rule21[1]*error+match_rule21[2];
        first_Fuzzy_Low[0] = match_rule22[0];
        first_Fuzzy_Low[1] = match_rule22[1]*error+match_rule22[2];
    }
    else if(error >= NS && error < ZO)
    {
        printf("NS<=error<ZO\n");

        first_Fuzzy_High[0] = match_rule31[0];
        first_Fuzzy_High[1] = match_rule31[1]*error+match_rule31[2];
        first_Fuzzy_Low[0] = match_rule32[0];
        first_Fuzzy_Low[1] = match_rule32[1]*error+match_rule32[2];
    }
    else if(error >= ZO && error < PS)
    {
        printf("ZO<=error<PS\n");

        first_Fuzzy_High[0] = match_rule41[0];
        first_Fuzzy_High[1] = match_rule41[1]*error+match_rule41[2];
        first_Fuzzy_Low[0] = match_rule42[0];
        first_Fuzzy_Low[1] = match_rule42[1]*error+match_rule42[2];
    }
    else if(error >= PS && error < PM)
    {
        printf("PS<=error<PM\n");

        first_Fuzzy_High[0] = match_rule51[0];
        first_Fuzzy_High[1] = match_rule51[1]*error+match_rule51[2];
        first_Fuzzy_Low[0] = match_rule52[0];
        first_Fuzzy_Low[1] = match_rule52[1]*error+match_rule52[2];
    }
    else if(error >= PM && error < PB)
    {
        printf("PM<=error<PB\n");

        first_Fuzzy_High[0] = match_rule61[0];
        first_Fuzzy_High[1] = match_rule61[1]*error+match_rule61[2];
        first_Fuzzy_Low[0] = match_rule62[0];
        first_Fuzzy_Low[1] = match_rule62[1]*error+match_rule62[2];
    }
    else if(error >= PB)
    {
        printf("PB<=error\n");

        first_Fuzzy_High[0] = match_rule61[0];
        first_Fuzzy_High[1] = 0;
        first_Fuzzy_Low[0] = match_rule62[0];
        first_Fuzzy_Low[1] = 1;
    }

    //cout << "fuzzy first check end" << endl;

}

void fuzzy_second_check(double error_dot)
{

    //cout << "fuzzy second check start" << endl;

    if(error_dot < NB)
    {
        printf("error<NB\n");

        second_Fuzzy_High[0] = match_rule11[0];
        second_Fuzzy_High[1] = 1.0;
        second_Fuzzy_Low[0] = match_rule12[0];
        second_Fuzzy_Low[1] = 0;
    }
    else if(error_dot >= NB && error_dot < NM)
    {
        printf("NB<=error<NM\n");

        second_Fuzzy_High[0] = match_rule11[0];
        second_Fuzzy_High[1] = match_rule11[1]*error_dot+match_rule11[2];
        second_Fuzzy_Low[0] = match_rule12[0];
        second_Fuzzy_Low[1] = match_rule12[1]*error_dot+match_rule12[2];
    }
    else if(error_dot >= NM && error_dot < NS)
    {
        printf("NM<=error<NS\n");

        second_Fuzzy_High[0] = match_rule21[0];
        second_Fuzzy_High[1] = match_rule21[1]*error_dot+match_rule21[2];
        second_Fuzzy_Low[0] = match_rule22[0];
        second_Fuzzy_Low[1] = match_rule22[1]*error_dot+match_rule22[2];
    }
    else if(error_dot >= NS && error_dot < ZO)
    {
        printf("NS<=error<ZO\n");

        second_Fuzzy_High[0] = match_rule31[0];
        second_Fuzzy_High[1] = match_rule31[1]*error_dot+match_rule31[2];
        second_Fuzzy_Low[0] = match_rule32[0];
        second_Fuzzy_Low[1] = match_rule32[1]*error_dot+match_rule32[2];
    }
    else if(error_dot >= ZO && error_dot < PS)
    {
        printf("ZO<=error<PS\n");

        second_Fuzzy_High[0] = match_rule41[0];
        second_Fuzzy_High[1] = match_rule41[1]*error_dot+match_rule41[2];
        second_Fuzzy_Low[0] = match_rule42[0];
        second_Fuzzy_Low[1] = match_rule42[1]*error_dot+match_rule42[2];
    }
    else if(error_dot >= PS && error_dot < PM)
    {
        printf("PS<=error<PM\n");

        second_Fuzzy_High[0] = match_rule51[0];
        second_Fuzzy_High[1] = match_rule51[1]*error_dot+match_rule51[2];
        second_Fuzzy_Low[0] = match_rule52[0];
        second_Fuzzy_Low[1] = match_rule52[1]*error_dot+match_rule52[2];
    }
    else if(error_dot >= PM && error_dot < PB)
    {
        printf("PM<=error<PB\n");

        second_Fuzzy_High[0] = match_rule61[0];
        second_Fuzzy_High[1] = match_rule61[1]*error_dot+match_rule61[2];
        second_Fuzzy_Low[0] = match_rule62[0];
        second_Fuzzy_Low[1] = match_rule62[1]*error_dot+match_rule62[2];
    }
    else if(error_dot >= PB)
    {
        printf("PB<=error\n");

        second_Fuzzy_High[0] = match_rule61[0];
        second_Fuzzy_High[1] = 0;
        second_Fuzzy_Low[0] = match_rule62[0];
        second_Fuzzy_Low[1] = 1;
    }

    //cout << "second fuzzy check end" << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_input_node");
    ControlInputNode control = ControlInputNode();

	// pt_data->clear();
	// pt_data->header.frame_id = "camera_color_optical_frame";


    //clock_t start, end;
    //double result;
    //start = clock(); // 수행 시간 측정 시작 /* 수행시간 측정하고자 하는 코드 */







    int x_rule=0;
    int y_rule =0;
    int theta_rule=0;




    //end = clock(); //시간 측정 끝
    //result = (double)(end - start); // 결과 출력
    //printf("%f", result / CLOCKS_PER_SEC);



    ros::spin();
    return 0;
}
