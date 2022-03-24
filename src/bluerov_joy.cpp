#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>


union JOYSTICK{
    struct{
        float joy_derecho[2];
        float joy_izquierdo[2];
        float gat_derecho;
        float gat_izquiero;
        float cruceta[2];
    };
    struct{
        float num[8];
    };
};

union BOTON{
    struct{
        int a;
        int b;
        int x;
        int y;
        int back;
        int start;
        int xbox;
        int lb;
        int rb;
        int but_jD;
        int but_jI;
    };
    struct{
        int num[11];
    };
};

struct A_LINEAL{
    float x;
    float y;
    float z;
};

struct ORIENTACION{
    float x;
    float y;
    float z;
    float w;
};

union IMU{
    struct{
        float aceleracion_lineal[3];
        float velocidad_angular[3];
        float orientacion[4];
    };
    struct{
        A_LINEAL a_lineal;
        A_LINEAL v_angular;
        ORIENTACION orienta;
    };
    struct{
        float val[10];
    };
};

/**
 * Inicio de la clase
 */
class BlueRov2{
    private:
        JOYSTICK joysticks;
        BOTON botones;
        IMU Imu;
        cv::Mat Frame;
    public:
        BlueRov2();
        ~BlueRov2();
        int leerBoton(unsigned char);
        float leerJoystic(unsigned char);
        void joyCallback(const sensor_msgs::Joy::ConstPtr&);
        void imuCallback(const sensor_msgs::Imu::ConstPtr&);
        void imageCallback(const sensor_msgs::ImageConstPtr&);
        void procesamiento(void);
};

BlueRov2::BlueRov2(){

};

BlueRov2::~BlueRov2(){

};

int BlueRov2::leerBoton(unsigned char _b){

    return botones.num[_b];
};

float BlueRov2::leerJoystic(unsigned char _b){

    return joysticks.num[_b];
};

void BlueRov2::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

    for(unsigned char i = 0 ; i < 8; i++){
        joysticks.num[i] = joy->axes[i];
    }
    for(unsigned char i = 0; i < 11 ; i++){
        botones.num[i] = joy->buttons[i];
    }
};

void BlueRov2::imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {

    Imu.a_lineal.x = imu->linear_acceleration.x;
    Imu.a_lineal.y = imu->linear_acceleration.y;
    Imu.a_lineal.z = imu->linear_acceleration.z;

    Imu.v_angular.x = imu->angular_velocity.x;
    Imu.v_angular.y = imu->angular_velocity.y;
    Imu.v_angular.z = imu->angular_velocity.z;

    Imu.orienta.x = imu->orientation.x;
    Imu.orienta.y = imu->orientation.y;
    Imu.orienta.z = imu->orientation.z;
    Imu.orienta.w = imu->orientation.w;
/*******************/
    ROS_INFO( "Acceleracion: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z,
              imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z,
              imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
    ROS_INFO( "Axis: %.3f, %.3f, boton A: %d", joysticks.joy_derecho[0], joysticks.joy_derecho[1], botones.a);
/*******************/
}

void BlueRov2::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat imagen_p;

    try{
        cv::resize(cv_bridge::toCvShare(msg, "bgr8")->image, imagen_p, cv::Size(640,480));
        Frame = imagen_p;
        cv::imshow("BlueRov2_view", Frame);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("No es posible convertir '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void BlueRov2::procesamiento(void){

}

/**
 * Node main function
 */
int main(int argc, char** argv) {

    ros::init(argc, argv, "bluerov_joy");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(10);

    BlueRov2 BlueRov2_control;

    cv::namedWindow("BlueRov2_view");

    image_transport::ImageTransport it(nodeHandle);
    image_transport::Subscriber subImage = it.subscribe("/BlueRov2/camera/image_raw", 1, &BlueRov2::imageCallback, &BlueRov2_control);

    ros::Subscriber subImu = nodeHandle.subscribe("/BlueRov2/imu/data", 10, &BlueRov2::imuCallback, &BlueRov2_control);
    ros::Subscriber subJoy = nodeHandle.subscribe("joy", 1000, &BlueRov2::joyCallback, &BlueRov2_control);

    ros::spin();
    cv::destroyWindow("BlueRov2_view");
    return 0;
};
