#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "drone_custom/DadosVisao.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/CameraInfo.h"
#include "ardrone_controller/pid.hpp"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
#include <std_srvs/Empty.h>





using namespace std;
double yawSetpoint,yawSetpointAligned, yawSetpoint2, magMeasured, height, vxMeasured, vyMeasured, dt, yawMeasured, imageX, imageY, imageH, imageW, contourCounter;
double imageHMin = 360;
double imageXF, imageYF, imageHF, imageWF, contourCounterF;
double startX,startY,contourStart,endX,endY,contourEnd;
double searchRightBuffer, searchLeftBuffer;
double movementY;
int alignCounter = 0;
int plannerCounter = 0;
bool flag = true;
bool toogleCamera = false;
double searchYawSetpoint;
geometry_msgs::Twist mv;
string cameraInfo;

PID xControl(1,-1,0.002,0,0);
PID yControl(1,-1,0.002,0,0);
PID zControl(5,-5,0.8,0,0);
PID yawControl(1,-1,0.5,0,0);



void visionCallback(const drone_custom::DadosVisao::ConstPtr& msg){
    imageX = msg->coordenada_X_centro_contorno_pista;
    imageY = msg->coordenada_Y_centro_contorno_pista;
    imageH = msg->larg_retangulo_pista;
    imageW = msg->alt_retangulo_pista;
    contourCounter = msg->qtde_contornos_pista;
    contourStart = msg->qtde_contornos_pista_inicio;
    contourEnd = msg->qtde_contornos_pista_final;
    startX = msg->coordenada_X_centro_contorno_pista_inicio;
    startY = msg->coordenada_Y_centro_contorno_pista_inicio;
    endX = msg->coordenada_X_centro_contorno_pista_final;
    endY = msg->coordenada_Y_centro_contorno_pista_final;
}

void visionCallbackF(const drone_custom::DadosVisao::ConstPtr& msg){
    imageXF = msg->coordenada_X_centro_contorno_pista;
    imageYF = msg->coordenada_Y_centro_contorno_pista;
    imageHF = msg->larg_retangulo_pista;
    imageWF = msg->alt_retangulo_pista;
    contourCounterF = msg->qtde_contornos_pista;
}

void sonarCallback(const sensor_msgs::Range::ConstPtr& msg){
    height = msg->range;
}

void navDataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg){
    vxMeasured = msg->vx;
    vyMeasured = msg->vy;
    yawMeasured = msg->rotZ;
}

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    cameraInfo = msg->header.frame_id;
}

void alignLine(){

    switch (alignCounter){
        case 0:
            searchYawSetpoint = yawMeasured;
            alignCounter++;
            cout << "Pista encontrada! Iniciando busca!" << endl;
            break;
        case 1:
            yawSetpoint = searchYawSetpoint - 45;
            if((yawMeasured < yawSetpoint +2) && (yawMeasured > yawSetpoint -2)){
                alignCounter++;
                cout << "Busca à direita concluída!" << endl;
            }
            break;
        case 2:
            yawSetpoint = searchYawSetpoint + 45;
            if ((yawMeasured < yawSetpoint +2) && (yawMeasured > yawSetpoint -2)){
                alignCounter++;
                yawSetpoint = yawSetpointAligned;
                cout << "Busca à esquerda concluída!" << endl;
            }
            break;     
        
    }
    if(imageH < imageHMin){
        imageHMin = imageH;
        yawSetpointAligned = yawMeasured;
    }
}

void followLine() {
    // liga o controlador(X) caso saia da faixa ao redor do centroide (X)
        mv.linear.x = xControl.calculate(dt,240,imageY); 

    
    // liga o controlador caso saia da faixa ao redor do centroide (Y)
    if((imageX > 340 || imageX< 300)){
        mv.linear.y = yControl.calculate(dt,320,imageX); 
    }
}

// void planner(){

//     switch(plannerCounter){
//         case 0:
//             if(contourStart > 0){
//                 if (startX > 320){
//                     movementY = -0.005;
//                 } else {
//                     movementY = 0.005;
//                 }
//             } else if (contourEnd > 0) {
//                 if (endX < 320){
//                     movementY = 0.005;
//                     plannerCounter++;
//                 } else {
//                     movementY = -0.005;
//                 }
//             } else {
//                     movementY = 0.005;
//             }
            
//             break;
//         case 1:
//             mv.linear.y = 0;
//             mv.linear.x = 0;
//             plannerCounter++;

//             // if (endCounter > 0) {
//             //     if (endX < 300 || endX > 340){
//             //         mv.linear.y = -0.5;
//             //     } else {
//             //         mv.linear.y = 0.5;
//             //     }
//             // } else {
//             //     mv.linear.y = 0.5;
//             // }
//             break;
//     }
// }

void planner(){

    switch(plannerCounter){
        case 0:
            if(contourStart > 0){
                if (startX > 320){
                    movementY = -0.005;
                } else {
                    movementY = 0.005;
                }
            } else if (contourEnd > 0) {
                if (endX < 320){
                    movementY = 0.005;
                    plannerCounter++;
                } else {
                    movementY = -0.005;
                }
            } else {
                    movementY = 0.005;
            }
            
            break;
        case 1:
            mv.linear.y = 0;
            mv.linear.x = 0;
            plannerCounter++;
            break;
    }
}

void searchLine() {
    if (contourCounterF < 1 ){//|| (imageXF == 320)){
    //if ((imageXF < 200) || (imageXF > 440)){
        mv.angular.z = 1;  
        mv.linear.y = 0;              
    } else {
        mv.angular.z = 0.2;
        //toogleCamera = true;
        //searchCounter++;
    }            

}


int main(int argc, char **argv)
{
    ros::init(argc, argv,"ardrone_controller");
    
    ros::NodeHandle node;
    ros::Rate(100);
    
    ros::ServiceClient camera = node.serviceClient<std_srvs::Empty>("ardrone/togglecam");
    std_srvs::Empty srv;
    std_msgs::Empty empty;

    ros::Subscriber visual_data_sub = node.subscribe("controle_visao_cam_baixo", 1, visionCallback);
    ros::Subscriber visual2_data_sub = node.subscribe("controle_visao_cam_frente", 1, visionCallbackF);
    ros::Subscriber nav_data_sub = node.subscribe("ardrone/navdata", 1, navDataCallback);
    // ros::Subscriber sonar_height_sub = node.subscribe("ardrone/navdata_altitude", 1000, sonarCallback);
    ros::Subscriber sonar_height_sub = node.subscribe("sonar_height", 1, sonarCallback);
    //ros::Subscriber camera_info = node.subscribe("ardrone/camera_info",1000,cameraInfoCallback);
    ros::Publisher land = node.advertise<std_msgs::Empty>("ardrone/land",1000);

    
    ros::Publisher cmd_vel = node.advertise<geometry_msgs::Twist>("cmd_vel",1);

    ros::Time timeNow(0), timePrev(0), timeLine(0);
    timePrev = ros::Time::now();

    mv.linear.x = 0;
    mv.linear.y = 0;
    mv.linear.z = 0;
    mv.angular.z = 0;
    cmd_vel.publish(mv);
    

    while(ros::ok()){
        // cout << cameraInfo.compare("ardrone_base_frontcam")<< " "<< cameraInfo.compare("ardrone_base_bottomcam") <<endl;
        if ((ros::Time::now() - timePrev).toSec() >= 0.05){
            dt = (ros::Time::now() - timePrev).toSec();
            timePrev = ros::Time::now();
            
            //CAMERA INFERIOR
            if(contourCounter > 0){    
                if (flag) {
                timeLine = ros::Time::now();
                flag = !flag;
                yawSetpoint = yawMeasured;
                toogleCamera = true;
                }
                
                followLine();
                
                if ((ros::Time::now() - timeLine).toSec() >= 3){
                   alignLine();
                }


                if (alignCounter > 2){
                    mv.angular.z = yawControl.calculate(dt, yawSetpointAligned,yawMeasured);
                    planner();                             
                } else {
                   mv.angular.z = yawControl.calculate(dt, yawSetpoint,yawMeasured);                
                }

               mv.linear.y += movementY;
                    
            } else { //CAMERA FRONTAL
                // if (toogleCamera){
                //     camera.call(srv);
                //     toogleCamera = false;
                // }
                //searchLine();
                mv.linear.x = 1;
                // mv.linear.y = 0;
                // mv.angular.z = 0;
                alignCounter = 0;
                plannerCounter = 0;
                flag = true;
                ros::Time timeLine(0);
                imageHMin = 360;
            }

            mv.linear.z = zControl.calculate(dt, 0.8, height);

            if(plannerCounter == 2){
                land.publish(empty);
                mv.linear.x = 0;
                mv.linear.y = 0;
                mv.linear.z = 0;
                mv.angular.z = 0;
            }

            cmd_vel.publish(mv);


        }


        
        ros::spinOnce();
        
    }
    return 0;
}