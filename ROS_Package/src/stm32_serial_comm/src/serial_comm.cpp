// ROS Headers
#include "ros/ros.h"
#include "omni_bot_msg/omnibot.h"
#include "sensor_msgs/Joy.h"
#include "serial/serial.h"

#define RX_BUF_SIZE 11
#define TX_BUF_SIZE 10

#define START_BIT 0xFF
#define PLUS_BIT 0x01
#define MINUS_BIT 0x00

/* ############################
 * Communication Protocol!
 * Start Bit(1bit) : 0xFF 
 * Three Motor CCR Data With Sign bit(+ : 0x01, - : 0x00), CCR bit (  Not Used   CCR_HIGH bit, CCR_LOW bit  )
 * Elevation Flag( 0 : Stop, 1: Up, 2: Down )
 * 11th bit is checksum for detecting communication error
 * ############################ */

serial::Serial ser;
uint8_t tx_buf[TX_BUF_SIZE] = {START_BIT, START_BIT, PLUS_BIT, 0x00, PLUS_BIT, 0x00, PLUS_BIT, 0x00, 0x00, 0x0a};
uint8_t tx_buf_[TX_BUF_SIZE] = {MINUS_BIT, MINUS_BIT, MINUS_BIT, MINUS_BIT, MINUS_BIT, MINUS_BIT, MINUS_BIT, MINUS_BIT, MINUS_BIT, MINUS_BIT};
uint8_t rx_buf[RX_BUF_SIZE];
uint8_t enter_buf[2]={13,10};


bool checksum_flag = true;
uint8_t checksum(uint8_t *buf){
    uint8_t sum = 0;
    for(int i = 2 ; i < TX_BUF_SIZE - 1; i++){
        sum += buf[i];
    }
    return sum;
}

void CCRCallback(const omni_bot_msg::omnibot::ConstPtr& msg){
    if(msg->Radian_0 < 0){
        tx_buf[2] = MINUS_BIT;
        tx_buf[3] = - msg->Radian_0;
    }
    else if(msg->Radian_0 >= 0){
        tx_buf[2] = PLUS_BIT;
        tx_buf[3] = msg->Radian_0;
    }

    if(msg->Radian_1 < 0){
        tx_buf[4] = MINUS_BIT;
        tx_buf[5] = - msg->Radian_1;
    }
    else if(msg->Radian_1 >= 0){
        tx_buf[4] = PLUS_BIT;
        tx_buf[5] = msg->Radian_1;
    }

    if(msg->Radian_2 < 0){
        tx_buf[6] = MINUS_BIT;
        tx_buf[7] = - msg->Radian_2;
    }
    else if(msg->Radian_2 >= 0){
        tx_buf[6] = PLUS_BIT;
        tx_buf[7] = msg->Radian_2;
    }
    tx_buf[8] = msg->Elevation;
    // tx_buf[8] = 0;
    tx_buf[9] = checksum(tx_buf);

    ROS_INFO_STREAM("CCR sub");
    ROS_INFO("Transmit : %x %x %x %x %x %x %x %x %x %x", tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7], tx_buf[8], tx_buf[9]);

    ser.write(tx_buf, TX_BUF_SIZE);
    ser.write(tx_buf, TX_BUF_SIZE);
    //ser.write(tx_buf, TX_BUF_SIZE);
    //ser.write(tx_buf, TX_BUF_SIZE);
    //ser.write(tx_buf, TX_BUF_SIZE);
    //ser.write(tx_buf, TX_BUF_SIZE);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "serial_test");
    ros::NodeHandle nh;

    std::string port;
    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    try{
        ser.setPort(port);
        ser.setBaudrate(38400);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else{
        return -1;
    }

    ros::Subscriber sub  = nh.subscribe("CCR", 1000, CCRCallback);

    ros::Rate rate(50);
    while (ros::ok()){

        ros::spinOnce();
        rate.sleep();
    }
}
