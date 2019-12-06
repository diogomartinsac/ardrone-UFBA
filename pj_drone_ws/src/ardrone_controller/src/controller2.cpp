#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "drone_custom/DadosVisao.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "ardrone_controller/pid.hpp"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"


using namespace std;
double yawSetpoint,yawSetpointAligned, yawSetpoint2, magMeasured, height, vxMeasured, vyMeasured, dt, yawMeasured, imageX, imageY, imageH, imageW, contourCounter;
double imageHMin = 360;
double imageXF, imageYF, imageHF, imageWF, contourCounterF;
double searchRightBuffer, searchLeftBuffer;
int alignCounter = 0;
bool searchFlagImageH = true;
bool searchRightFlag = false;
bool searchLeftFlag = false;
bool flag = true;
double searchYawSetpoint;
geometry_msgs::Twist mv;

PID xControl(1,-1,0.002,0,0);
PID yControl(1,-1,0.002,0,0);
PID zControl(5,-5,0.8,0,0);
PID yawControl(1,-1,0.5,0,0);


void visionCallback(const drone_custom::DadosVisao::ConstPtr& msg){
    imageX = msg->coordenada_X_centroContorno;
    imageY = msg->coordenada_Y_centroContorno;
    imageH = msg->larg_retangulo;
    imageW = msg->alt_retangulo;
    contourCounter = msg->qtde_contornos;
}

void visionCallbackF(const drone_custom::DadosVisao::ConstPtr& msg){
    imageXF = msg->coordenada_X_centroContorno;
    imageYF = msg->coordenada_Y_centroContorno;
    imageHF = msg->larg_retangulo;
    imageWF = msg->alt_retangulo;
    contourCounterF = msg->qtde_contornos;
}

void sonarCallback(const sensor_msgs::Range::ConstPtr& msg){
    height = msg->range;
}

void navDataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg){
    vxMeasured = msg->vx;
    vyMeasured = msg->vy;
    yawMeasured = msg->rotZ;
}

void alignBand(){

    switch (alignCounter){
        case 0:
            searchYawSetpoint = yawMeasured;
            alignCounter++;
            cout << "Pista encontrada! Iniciando busca!" << endl;
            break;
        case 1:
            yawSetpoint = searchYawSetpoint - 30;
            if((yawMeasured < yawSetpoint +2) && (yawMeasured > yawSetpoint -2)){
                alignCounter++;
                cout << "Busca à direita concluída!" << endl;
            }
            break;
        case 2:
            yawSetpoint = searchYawSetpoint + 30;
            if ((yawMeasured < yawSetpoint +2) && (yawMeasured > yawSetpoint -2)){
                alignCounter++;
                cout << "Busca à esquerda concluída!" << endl;
            }
            break;        
    }

    if(imageH < imageHMin){
        imageHMin = imageH;
        yawSetpointAligned = yawMeasured;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"ardrone_controller");
    
    ros::NodeHandle node;
    ros::Rate(60);
    
    ros::Subscriber visual_data_sub = node.subscribe("controle_visao_cam_baixo", 1000, visionCallback);
    ros::Subscriber visual2_data_sub = node.subscribe("controle_visao_cam_frente", 1000, visionCallbackF);
    ros::Subscriber nav_data_sub = node.subscribe("ardrone/navdata", 1000, navDataCallback);
    ros::Subscriber sonar_height_sub = node.subscribe("sonar_height", 1000, sonarCallback);

    
    ros::Publisher cmd_vel = node.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    ros::Time timeNow(0), timePrev(0), timeBand(0);
    timePrev = ros::Time::now();

    
    

    while(ros::ok()){

        if ((ros::Time::now() - timePrev).toSec() >= 0.06){
            dt = (ros::Time::now() - timePrev).toSec();
            timePrev = ros::Time::now();
            
            //se detecta alguma coisa
            if(contourCounter > 0){
                if (flag) {
                timeBand = ros::Time::now();
                flag = !flag;
                }
                
                // liga o controlador(X) caso saia da faixa ao redor do centroide (X)
                if((imageY > 230 || imageY< 190)){
                    mv.linear.x = xControl.calculate(dt,210,imageY); 
                }
               
                // liga o controlador caso saia da faixa ao redor do centroide (Y)
                if((imageX > 340 || imageX< 300)){
                    mv.linear.y = yControl.calculate(dt,320,imageX); 
                }
                
                //if ((imageY > 220 || imageY< 200) && (imageX > 330 || imageX< 310) && ((ros::Time::now() - timeBand).toSec() >= 2)){
                if ((ros::Time::now() - timeBand).toSec() >= 5){
                    alignBand();
                }
                cout << imageH << " " << yawSetpoint << " " << imageHMin << " "<< yawSetpointAligned << " " << yawMeasured <<endl;

                if (alignCounter > 2){
                    mv.angular.z = yawControl.calculate(dt, yawSetpointAligned,yawMeasured);                
                } else {
                    mv.angular.z = yawControl.calculate(dt, yawSetpoint,yawMeasured);                
                }
                    
            } else{
                //para o drone            
                // mv.linear.x = 0;
                // mv.linear.y = 0;
                mv.angular.z = 0;
                
                mv.linear.x = 0.5;
                alignCounter = 0;
                flag = true;
                ros::Time timeBand(0);
                searchFlagImageH = true;
            }

            mv.linear.z = zControl.calculate(dt, 0.8, height);
            cmd_vel.publish(mv);
        }
        
        ros::spinOnce();
        
    }
    return 0;
}