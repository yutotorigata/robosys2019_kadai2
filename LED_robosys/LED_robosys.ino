
//=====include file=================================
#include <Arduino.h>
#include <ros.h>                    //ros通信
#include <geometry_msgs/Twist.h>    //rosのtwist型

//=====nodeを立ち上げる===================================
ros::NodeHandle node;

//=====グローバル関数=====================================
int flag   = 0;

//=================//LED=================================
void LED_switch(double color){   
  if (color == 1.0){
    digitalWrite(2, HIGH);
    delay(4000);
    node.spinOnce();
    digitalWrite(2, LOW);
    delay(2000);
    node.spinOnce();
  }else if (color == 2.0){
    digitalWrite(3, HIGH);
    delay(4000);
    node.spinOnce();
    digitalWrite(3, LOW); 
    delay(2000);
    node.spinOnce(); 
  }else if (color == 3.0){
    digitalWrite(4, HIGH);
    delay(4000);
    node.spinOnce();
    digitalWrite(4, LOW);  
    delay(2000);
    node.spinOnce();
  }
}

//=============================================================
//                        callback
//=============================================================
void callback(const geometry_msgs::Twist& msg){
  double color = msg.linear.z;

    if(flag != 1){
      flag = 1;
      LED_switch(msg.linear.z);  
      flag = 0;     
    }
}

//============================================
//setup start

ros::Subscriber<geometry_msgs::Twist> sub("/number", &callback);
void setup(){
  Serial.begin(9600);
  
  //ros動作
  node.initNode();
  node.subscribe(sub);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
}

//============================================
//loop start

void loop (){   
  delay(500);
  node.spinOnce();
} 
