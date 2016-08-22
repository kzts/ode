/**　ISI lab x Nedo project　**************************************************************************

	Program name: Nedo_arm_20160331
	Files: main.cpp, Makefile, texturepath.h
	Version: 04.00
	Platform: ODE 0.12
	Author: Chang Shih-Yin, 2016/3/31
	Design: 
		- One arm
			Link0: body
			Link1: upper arm		0.20m
			Link2: lower arm		0.20m
			Link3: hand				0.05m
			Joint0: body to ground, fixed
			Joint1: yaw of shoulder, control by torque, add movable range
			Joint2: yaw of elbow, control by torque, add movable range
			Joint3: yaw of wrist, control by torque, add movable range
		- One stick
			LinkStick00: stick link		0.10m
			LinkStick01: stick link		0.10m
			LinkStick02: stick link		0.10m
			LinkStick03: stick link		0.10m
			LinkStick04: stick link		0.10m
			LinkJoint00: stick to arm, fixed
			LinkJoint01: yaw of stick joint, with spring and damping
			LinkJoint02: yaw of stick joint, with spring and damping
			LinkJoint03: yaw of stick joint, with spring and damping
			LinkJoint04: yaw of stick joint, with spring and damping
	Command:
		ctrl+p:				pause
		ctrl+x:				end program
		a:					add shoulder torque by 1
		d:					decrease shoulder torque by 1
		i:					input torque
		p:					input target angle (disable)
		c:					check joint
		r:					reset
	Update note:
		Update nearCallback	- 2016/3/31		(Not finished)
		Disable P control	- 2016/3/31		(Cannot use two control methods at the same time)
		Rotate stick		- 2016/3/31
		Add Pcontrol		- 2016/3/28
		Add arm				- 2016/3/28
		Add ball			- 2016/3/24
	        Add stick			- 2016/3/24
		Update command		- 2016/3/24
		Add erp and cmf		- 2016/3/23 18:00
		Add joint check		- 2016/3/23 14:40
		Add torque control	- 2016/3/17 20:26
	Next work:
		revise joint
		check nearCallback
****************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include "texturepath.h"

#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawLine dsDrawLineD
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#endif

/* --------------------------------------------------
					Set Parameters
   -------------------------------------------------- */
   
#define TEXTURES_PATH	"../../drawstuff/textures"	// Point the path to textures
#define TIMESIZE                10
#define STEP_SIZE		0.001/TIMESIZE						// Step size for simulation speed [sec]
#define EARTH_GRAVITY	-9.8						// Set gravity

#define L_NUM			4							// Number of link of arm
#define J_NUM			4							// Number of joint of arm
#define L_Stick_NUM		5							// Number of link of stick
#define J_Stick_NUM		5							// Number of joint of stick
#define time_sep                3
#define torque_max              200
#define timelimit               150
#define OUTPUT_PATH 	"./result/"
//#define DRAW 　//先頭のコメントを外すと描画します。
 

dWorldID		world;								// World for dynamic computation
dSpaceID		space;								// Space for collision-detection
dGeomID			ground;								// Geometry ID of ground
dJointGroupID	contactgroup;						// Contact group
dJointID		joint[J_NUM];						// Joint ID
dJointID		joint_Stick[J_Stick_NUM];			// Joint ID
dsFunctions		fn;									// For the DrawStuff function

typedef struct{
	dBodyID body;									// Body ID
	dGeomID geom;									// Geometry ID
}MyObject;
MyObject arm[L_NUM];								// Link
MyObject stick[L_Stick_NUM];						// Stick
MyObject ball;										// Ball

dMass mass;																// Mass paramater
// Arm
dReal x[L_NUM]      = {0.00, 0.11, 0.31, 0.435};						// Link COM x
dReal y[L_NUM]      = {0.00, 0.00, 0.00, 0.00};							// Link COM y
dReal z[L_NUM]      = {0.02, 0.02, 0.02, 0.02};							// Link COM z
dReal length[L_NUM] = {0.04, 0.20, 0.20, 0.05};							// Link length
dReal weight[L_NUM] = {1.00, 1.00, 1.00, 0.25};							// Link weight
dReal r[L_NUM]      = {0.01, 0.01, 0.01, 0.01};							// Link radius
dReal c_x[J_NUM]    = {0.00, 0.00, 0.21, 0.41};							// Joint center x
dReal c_y[J_NUM]    = {0.00, 0.00, 0.00, 0.00};							// Joint center y
dReal c_z[J_NUM]    = {0.00, 0.02, 0.02, 0.02};							// Joint center z
dReal axis_x[J_NUM] = {0, 0, 0, 0};										// Joint axis x
dReal axis_y[J_NUM] = {0, 0, 0, 0};										// Joint axis y
dReal axis_z[J_NUM] = {0, 1, 1, 1};										// Joint axis z
dReal angleMax[J_NUM]	= {1*M_PI, 0.5*M_PI, 0.99*M_PI, 0.5*M_PI};		// Joint angle Max (0~1 will cause error, so change to 0~0.99)
dReal angleMin[J_NUM]	= {-1*M_PI, -0.5*M_PI, 0*M_PI, -0.5*M_PI};		// Joint angle Min
dReal torque[J_NUM]	= {0.00, 0.00, 0.00, 0.00};							// Joint torque
dReal THETA[J_NUM] = {0.0, 0.0, 0.0, 0.0};								// Joint target angle (rad)
// Stick
dReal x_Stick[L_Stick_NUM]      	= {0.45, 0.45, 0.45, 0.45, 0.45};		// Link COM x
dReal y_Stick[L_Stick_NUM]      	= {0.06, 0.16, 0.26, 0.36, 0.46};		// Link COM y
dReal z_Stick[L_Stick_NUM]      	= {0.02, 0.02, 0.02, 0.02, 0.02};		// Link COM z
dReal length_Stick[L_Stick_NUM] 	= {0.10, 0.10, 0.10, 0.10, 0.10};		// Link length
dReal weight_Stick[L_Stick_NUM] 	= {0.07, 0.07, 0.07, 0.07, 0.07};		// Link weight
dReal r_Stick[L_Stick_NUM]      	= {0.01, 0.01, 0.01, 0.01, 0.01};		// Link radius
dReal c_x_Stick[J_Stick_NUM]    	= {0.45, 0.45, 0.45, 0.45, 0.45};		// Joint center x
dReal c_y_Stick[J_Stick_NUM]    	= {0.01, 0.11, 0.21, 0.31, 0.41};		// Joint center y
dReal c_z_Stick[J_Stick_NUM]    	= {0.02, 0.02, 0.02, 0.02, 0.02};		// Joint center z
dReal axis_x_Stick[J_Stick_NUM] 	= {0, 0, 0, 0, 0};						// Joint axis x
dReal axis_y_Stick[J_Stick_NUM] 	= {0, 0, 0, 0, 0};						// Joint axis y
dReal axis_z_Stick[J_Stick_NUM] 	= {1, 1, 1, 1, 1};						// Joint axis z
dReal angleMax_Stick[J_Stick_NUM]	= {0, 0, 0, 0, 0};						// Joint angle Max
dReal angleMin_Stick[J_Stick_NUM]	= {0, 0, 0, 0, 0};						// Joint angle Min
// Set spring and damping
dReal kp = 15;														// Spring constant [N/m]
dReal kd = 1.0;														// Damping constant [Ns/m]
dReal erp = STEP_SIZE * kp / (STEP_SIZE * kp + kd);
dReal cfm = 1.0 / (STEP_SIZE * kp + kd);
dReal stick_link_mass=0.10;
dReal tmp_data[time_sep][4]={};
// Variables to take result values of joint check
int jointCheck[J_NUM] = {0, 0, 0, 0};								// Value of joint connecting check
int jointCheck_Stick[J_Stick_NUM] = {0, 0, 0, 0, 0};				// Value of joint connecting check
// Ball
dReal x_Ball = 0.00;			// Position x
dReal y_Ball = 0.80;			// Position y
dReal z_Ball = 0.02;			// Position z
dReal r_Ball = 0.02;			// Radius
dReal weight_Ball = 0.20;		// Weight

float timetime  = 0;
float timetimetime = 0;
int goal_torque[time_sep][J_NUM];
int step[time_sep-1];
//int max_goal_torque[time_sep][J_NUM];
//int max_step[time_sep-1];
int max_goal_torque_box[time_sep][J_NUM][21952];
int max_step_box[time_sep-1][21952];
int nowstep = 0;
int counter = 0;
int temp = 0;
int mode1mode;
int mode = 3;
int m1iterate = 0;
int m21iterate = 0;
int m22iterate = 0;
int makelinkon = 0;
int para0=0;
int para1=0;
int para2=0;
int para3=0;
int para4=0;
int para5=0;
int para_mass=0;
int para_kp=0;
int para_kd=0;
char output_file[128];
float mindistance = 10000;
float value = 0;
//float max_value = -10000;
float valuelog[300];
float timesize = STEP_SIZE*1000;
int collideflag = 0;
const int patterns = timelimit-1;
int stock[patterns];
int box[28][28][28];

int max_goal_torque[time_sep][J_NUM]=
{{-105,43,-96,-3},
{153,-74,-102,-43},
{-55,-52,-200,-158}};
int max_step[time_sep-1]={104,134};
//float max_value = 0.905764;
float max_value = -100000;

/*for(int i=0;i<time_sep;i++){
    for(int j=0;j<J_NUM;j++){
	goal_torque[i][j] = 0;
    }
 }
*/

int boxnum(int i,int j, int k){
  return box[i][j][k];
}

int boxnum_shift(int i,int j, int k){
  if (i==0&&j!=0){
    return box[i][j-1][k];
      }
  else if(i==0&&j==0){
    return box[i][j][k-1];
  }
  else{
    return box[i-1][j][k];
  }
}

/* --------------------------------------------------
					My Functions
   -------------------------------------------------- */

// Generate links
void makeLink(){
	// Arm links
	for(int i=0; i<L_NUM; i++){
		arm[i].body = dBodyCreate(world);
		dMassSetZero(&mass);
		// Make cylinder link
		dMassSetCylinderTotal(&mass,weight[i],3,r[i],length[i]);
		// dMassAdjust(&mass, weight[i]);
		dBodySetMass(arm[i].body, &mass);
		// Set default position and poster
		dBodySetPosition(arm[i].body, x[i], y[i], z[i]);
		if(i!=0){
			dMatrix3 L_Rotation;									// Link rotation matrix
			dRFromAxisAndAngle(L_Rotation, 0, 1, 0, M_PI/2.0);		// Rotate π/2[rad] by Y-axis
			dBodySetRotation(arm[i].body, L_Rotation);
		}
		arm[i].geom = dCreateCylinder(space, r[i], length[i]);
		dGeomSetBody(arm[i].geom, arm[i].body);
	}
	// Stick links
	for(int i=0; i<L_Stick_NUM; i++){
		stick[i].body = dBodyCreate(world);
 		dMassSetZero(&mass);
		// Make cylinder link
		dMassSetCylinderTotal(&mass, weight_Stick[i], 3, r_Stick[i], length_Stick[i]);
		dBodySetMass(stick[i].body, &mass);
		// Set default position and poster
		dBodySetPosition(stick[i].body, x_Stick[i], y_Stick[i], z_Stick[i]);
		dMatrix3 L_Rotation;									// Link rotation matrix
		dRFromAxisAndAngle(L_Rotation, 1, 0, 0, M_PI/2.0);		// Rotate π/2[rad] by X-axis
		dBodySetRotation(stick[i].body, L_Rotation);
		stick[i].geom = dCreateCylinder(space, r_Stick[i], length_Stick[i]);
		dGeomSetBody(stick[i].geom, stick[i].body);
	}
	// Arm joints
	for(int j=0; j<J_NUM; j++){
		if(j==0){
			joint[j] = dJointCreateFixed(world, 0);
			dJointAttach(joint[j], arm[j].body, 0);
			dJointSetFixed(joint[j]);
		}else{
			joint[j] = dJointCreateHinge(world, 0);
			dJointAttach(joint[j], arm[j].body, arm[j-1].body);
			dJointSetHingeAnchor(joint[j], c_x[j], c_y[j], c_z[j]);
			dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j], axis_z[j]);
			// Set movable range of joint
			dJointSetHingeParam(joint[j], dParamHiStop, angleMax[j]);
			dJointSetHingeParam(joint[j], dParamLoStop, angleMin[j]);
		}
	}
	// Stick joints
	for(int j=0; j<J_Stick_NUM; j++){
		if(j==0){
			joint_Stick[j] = dJointCreateHinge(world, 0);
			dJointAttach(joint_Stick[j], stick[j].body, arm[L_NUM-1].body);
			dJointSetHingeAnchor(joint_Stick[j], c_x_Stick[j], c_y_Stick[j], c_z_Stick[j]);
			dJointSetHingeAxis(joint_Stick[j], axis_x_Stick[j], axis_y_Stick[j], axis_z_Stick[j]);
			// Set movable range of joint
			dJointSetHingeParam(joint_Stick[j], dParamHiStop, angleMax_Stick[j]);
			dJointSetHingeParam(joint_Stick[j], dParamLoStop, angleMin_Stick[j]);
		}else{
			joint_Stick[j] = dJointCreateHinge(world, 0);
			dJointAttach(joint_Stick[j], stick[j].body, stick[j-1].body);
			dJointSetHingeAnchor(joint_Stick[j], c_x_Stick[j], c_y_Stick[j], c_z_Stick[j]);
			dJointSetHingeAxis(joint_Stick[j], axis_x_Stick[j], axis_y_Stick[j], axis_z_Stick[j]);
			// Set movable range of joint
			dJointSetHingeParam(joint_Stick[j], dParamHiStop, angleMax_Stick[j]);
			dJointSetHingeParam(joint_Stick[j], dParamLoStop, angleMin_Stick[j]);
			// Set spring in joint
			dJointSetHingeParam(joint_Stick[j], dParamStopERP, erp);
			dJointSetHingeParam(joint_Stick[j], dParamStopCFM, cfm);
		}
	}
	// Make ball
	ball.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetSphereTotal(&mass, weight_Ball, r_Ball);
	dBodySetMass(ball.body, &mass);
	dBodySetPosition(ball.body, x_Ball, y_Ball, z_Ball);
	ball.geom = dCreateSphere(space, r_Ball);
	dGeomSetBody(ball.geom, ball.body);
}

// Draw links
void drawLink()
{
	dReal tmp_r, tmp_length;
	// Arm
	int color_arm = 0;
	for(int i=0; i<L_NUM; i++){
		// Set color
		if(color_arm==0){
			dsSetColor(0, 0, 0);
			color_arm++;
		}else{
			dsSetColor(1.0, 1.0, 1.0);
			color_arm--;
		}
		// Make cylinder link
		dGeomCylinderGetParams(arm[i].geom, &tmp_r, &tmp_length);
		dsDrawCylinder(dBodyGetPosition(arm[i].body), dBodyGetRotation(arm[i].body),tmp_length,tmp_r);
	}
	// Stick
	int color_stick = 0;
	for(int i=0; i<L_Stick_NUM; i++){
		// Set color
		if(color_stick==0){
			dsSetColor(0, 0.6, 1.0);
			color_stick++;
		}else{
			dsSetColor(0, 0.8, 0.4);
			color_stick--;
		}
		// Make cylinder link
		dGeomCylinderGetParams(stick[i].geom, &tmp_r, &tmp_length);
		dsDrawCylinder(dBodyGetPosition(stick[i].body), dBodyGetRotation(stick[i].body),tmp_length,tmp_r);
	}
	// Ball
	dsSetColor(1.0,0.0,0.0);
	dsDrawSphere(dBodyGetPosition(ball.body), dBodyGetRotation(ball.body), r_Ball);
}

// Detect collision ????
static void nearCallback(void *data, dGeomID o1, dGeomID o2){
	dBodyID b1 = dGeomGetBody(o1);		// Object 1
	dBodyID b2 = dGeomGetBody(o2);		// Object 2
	// Exclude following situations
	if(b1 && b2 && dAreConnected(b1, b2)) return;
	if((b1 == arm[0].body) || (b2 == arm[0].body)) return;


	/*if((b1 == stick[4].body && b2 == ball.body) || (b1 == ball.body && stick[4].body)){
	  // printf("HIT!!!\n");
	  collideflag = 1;
	  }*/


	// Set contact point numbers
	const int MAX_CONTACTS = 10;
	dContact contact[MAX_CONTACTS];
	for(int i=0; i<MAX_CONTACTS; i++){
		// 物体同士の接触時のパラメータ設定
		contact[i].surface.mode = dContactBounce | dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
		contact[i].surface.mu = 1;				// 摩擦係数
		contact[i].surface.bounce = 0.1;		// 反発係数(0.0~1.0)
		contact[i].surface.bounce_vel = 0.0;	// 反発に必要な最低速度(0.0以上)
		contact[i].surface.slip1 = 0.0;
		contact[i].surface.slip2 = 0.0;
		contact[i].surface.soft_erp = 0.8;
		contact[i].surface.soft_cfm = 0.01;
	}
	int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
	if(numc > 0){
		for(int i=0; i<numc; i++){
			dJointID c = dJointCreateContact(world, contactgroup, contact+i);
			dJointAttach(c, b1, b2);
		}
	}
}

// Pcontrol for target angle
/*
void Pcontrol()
{
	dReal k =  5.0, fMax = 100.0;                          // k = proportional gain, fMax = max torque
	for (int j=0; j<J_NUM; j++) {
		if((j!=0)){
			dReal tmp = dJointGetHingeAngle(joint[j]);     // get joint angle
			dReal z = THETA[j] - tmp;                      // error value (difference between measured angle and desired angle)
			dJointSetHingeParam(joint[j],dParamVel, k*z);  // set angular velocity
			dJointSetHingeParam(joint[j],dParamFMax,fMax); // set torque
		}
	}
}
*/

// Control torque
void TorqueControl(){
  //torque[1]=200;
  
  if(timetime==0&&timetimetime==TIMESIZE-1){
    
     if(mode==0){
	    printf("mode1\n");
	    m1iterate += 1;
	  int stock[patterns];
	  for(int i = 0; i < time_sep; i++){
	    for(int j = 0; j < J_NUM; j++){
	      goal_torque[i][j] = (((rand() % 41)-20) *5);
	    }
	  }

	  for(int k=0;k<patterns;k++){
	    stock[k]=k+1;
	  }
	  
	  for(int i=0; i<time_sep-1; i++){
	      /* stockの何番目か決める */
     int index=rand()%(patterns-i); 
	      /* index番目を出力 */
	      //printf("%d\n",stock[index]); 
    	      step[i] = stock[index];
	      /* 出力した値はstockから削除 */
    	       for(int j=index; j<patterns-1; j++) 
		stock[j]=stock[j+1];
	  }
	  for(int i=0; i<time_sep-1; i++){
	    for(int j=time_sep-2; j>i; j--){
	      if(step[j] < step[j-1]){
		temp = step[j];
		step[j] = step[j-1];
		step[j-1] = temp;
	      }
	    }
	  }
	  for(int i=0;i<time_sep-1;i++){
	    //printf("%d\n",step[i]);
	    }

	  /*for(int i = 0; i < time_sep; i++){
	    for(int j = 0; j < J_NUM; j++){
	      printf("%d,",goal_torque[i][j]);
	      if(j==J_NUM-1){
		printf("\n");
	      }
	    }
	    }*/
    }
	  else if(mode==1){
	    for(int i = 0; i < time_sep; i++){
	      for(int j = 0; j < J_NUM; j++){
		goal_torque[i][j] = max_goal_torque[i][j];
	      }
	    }
	    for(int i = 0; i < time_sep-1; i++){
	     step[i] = max_step[i];
	    }
	    mode1mode = rand()%2;
	    if(mode1mode==0){
	      printf("mode2-1\n");
	      m21iterate += 1;
	    for(int i = 0; i < time_sep; i++){
	      for(int j = 0; j < J_NUM; j++){
		goal_torque[i][j] = goal_torque[i][j] + (rand() % 21)-10;
	      }
	    }
	    for(int i = 0; i < time_sep-1; i++){
	      step[i] = step[i] + (rand() %11) - 5;
	    }
	    for(int i=0; i<time_sep-1; i++){
	      for(int j=time_sep-2; j>i; j--){
		if(step[j] < step[j-1]){
		  temp = step[j];
		  step[j] = step[j-1];
		  step[j-1] = temp;
		}
	      }
	    }
	    // for(int i = 0;i<time_sep-1;i++){
	    // printf("%d\n",max_step[i]);
	    //}
	   
	    }else if(mode1mode==1){
	      printf("mode2-2\n");
	      m22iterate += 1;
	      int i = rand()%time_sep;
	      int j = rand()%J_NUM;
	      goal_torque[i][j] = goal_torque[i][j]  + (rand() % 21)-10;
	    }
	    for(int i = 0; i < time_sep; i++){
	      for(int j = 0; j < J_NUM; j++){
		if(goal_torque[i][j]>torque_max){
		  goal_torque[i][j] = torque_max;
		}else if(goal_torque[i][j]<-torque_max){
		  goal_torque[i][j]=-torque_max;
		}
	      }
	    }
	    for(int i = 0; i < time_sep-1; i++){
	      if(step[i]>timelimit-1){
		step[i] = timelimit-1;
	      }else if(step[i]<1){
		step[i]=1;
	      }
	    }
	  }
	  else if(mode == 2){
	    printf("mode3\n");
	    for(int i = 0; i < time_sep; i++){
	      for(int j = 0; j < J_NUM; j++){
		goal_torque[i][j] = max_goal_torque_box[i][j][boxnum_shift(para_kd,para_kp,para_mass)];
	      }
	    }
	    for(int i = 0; i < time_sep-1; i++){
	      step[i] = max_step_box[i][boxnum_shift(para_kd,para_kp,para_mass)];
	    }
	    if(para_kd==0&&para_kp==0){
	      goal_torque[0][1] = goal_torque[0][1] + 2;
	    }
	    mode = 1;
	  }

	  else if(mode == 3){
	    printf("mode4\n");
	    for(int i = 0; i < time_sep; i++){
	      for(int j = 0; j < J_NUM; j++){
		goal_torque[i][j] = max_goal_torque[i][j];
	      }
	    }
	    for(int i = 0; i < time_sep-1; i++){
	      step[i] = max_step[i];
	    }
	    mode = 1;
	  }
	}
	

	//printf("%f\n", timetime);
	//printf("%f\n", timetimetime);

	if(timetime==0&&timetimetime==TIMESIZE-1){
	  for(int i=0;i<J_NUM;i++){
	    torque[i]=goal_torque[0][i];
	  }
	}
	if(timetime==step[nowstep]&&timetimetime==TIMESIZE-1){
	  //  printf("%d\n",nowstep);
	  for(int i=0;i<J_NUM;i++){
	    torque[i]=goal_torque[nowstep+1][i];
	  }
	  /* for(int i=0;i<J_NUM;i++){
	    printf("%d,",goal_torque[nowstep][i]);
	    }*/
	  // printf("\n");
	  nowstep += 1;
	}

	for(int j=0; j<J_NUM; j++){
		if(j!=0){
			dJointAddHingeTorque(joint[j], torque[j]);	// set torque
		}
	}


	const  dReal *stickend_pos = dBodyGetPosition(stick[4].body);
	const  dReal *ball_pos = dBodyGetPosition(ball.body);
	float distance = (stickend_pos[0]-ball_pos[0])*(stickend_pos[0]-ball_pos[0])+(stickend_pos[1]-ball_pos[1])*(stickend_pos[1]-ball_pos[1]);
	//printf("Distance: %f\n", distance);
	if(distance<mindistance){
	  mindistance = distance;
	}
	//printf("MinDistance: %f\n", mindistance);

	timetimetime += 1;
	if(timetimetime == TIMESIZE){
	timetime += 1; 
	timetimetime = 0;
	}
}


void output_data1(){
  sprintf(output_file, "%spara0_0/para1_0/result_%d_%d_%d_%d_%d.txt" , OUTPUT_PATH,para0,para1,para_mass,para_kp,para_kd);	
  std::ofstream fout(output_file, std::ios::out);	
  for(int j=0; j<time_sep; j++){
    //	    for(int jj=0; jj<15; jj++){
    for(int jj=0; jj<4; jj++){
      fout << tmp_data[j][jj] << "\t";
    }
    fout << std::endl;
  }
  fout.close();
}

void output_data2(){
  sprintf(output_file, "%spara0_0/para1_0/value_%d_%d_%d_%d_%d.txt" , OUTPUT_PATH,para0,para1,para_mass,para_kp,para_kd);	
  std::ofstream fout(output_file, std::ios::out);	
      fout << max_value<< "\t";
  fout.close();
}



// Input command from keyboard
void command(int cmd){
  if(cmd == 'q'){
    torque[1] += 2.0;
    printf("Torque1: %f\n", torque[1]);
  }else if(cmd == 'a'){
    torque[1] -= 2.0;
    printf("Torque1: %f\n", torque[1]);
  }else if(cmd == 'w'){
    torque[2] += 2.0;
    printf("Torque2: %f\n", torque[2]);
  }else if(cmd == 's'){
    torque[2] -= 2.0;
    printf("Torque2: %f\n", torque[2]);
  }else if(cmd == 'e'){
    torque[3] += 2.0;
    printf("Torque3: %f\n", torque[3]);
  }else if(cmd == 'd'){
    torque[3] -= 2.0;
    printf("Torque3: %f\n", torque[3]);
  }else if(cmd == 'r'){
    torque[4] += 2.0;
    printf("Torque4: %f\n", torque[4]);
  }else if(cmd == 'f'){
    torque[4] -= 2.0;
    printf("Torque4: %f\n", torque[4]);
  }else if(cmd == 'l'){
    const  dReal *stickend_pos = dBodyGetPosition(stick[4].body);
    const  dReal *ball_pos = dBodyGetPosition(ball.body);
    float distance = (stickend_pos[0]-ball_pos[0])*(stickend_pos[0]-ball_pos[0])+(stickend_pos[1]-ball_pos[1])*(stickend_pos[1]-ball_pos[1]);
    printf("Distance: %f\n", distance);
  }else if(cmd == 'm'){
    FILE *outputfile;
    outputfile = fopen("output/max_value.txt", "w");
    if (outputfile == NULL) {          
      printf("cannot open\n");         
      exit(1);                         
    }
    printf("random_iterate:%d\n",m1iterate);
    fprintf(outputfile,"random_iterate:%d\n",m1iterate);
    printf("climbing_iterate1:%d\n",m21iterate);
    fprintf(outputfile,"climbing_iterate1:%d\n",m21iterate);
    printf("climbing_iterate2:%d\n",m22iterate);
    fprintf(outputfile,"climbing_iterate2:%d\n",m22iterate);
    printf("max_value:%f\n",max_value);
    fprintf(outputfile,"max_value:%f\n",max_value);
    printf("max_goal_torque:\n");
    fprintf(outputfile,"max_goal_torque:\n");
    for(int i = 0; i < time_sep; i++){
      for(int j = 0; j < J_NUM; j++){
	printf("%d,",max_goal_torque[i][j]);
	fprintf(outputfile,"%d,",max_goal_torque[i][j]);
	if(j==J_NUM-1){
	  printf("\n");
	  fprintf(outputfile,"\n");
	}
      }
    }
    printf("max_time_separate:");
    fprintf(outputfile,"max_time_separate:");
    for(int i=0;i<time_sep-1;i++){
      printf("%d,",max_step[i]);
      fprintf(outputfile,"%d,",max_step[i]);
    }
    printf("\n");
    fprintf(outputfile,"\n");
    fclose(outputfile);
  }else if(cmd == 'n'){
    mode = 1;
  }else if(cmd == 'b'){
    mode = 0;
  }else if(cmd =='v'){
    mode = 2;
  }else if(cmd == 'i'){
    char input_mode;
    float input_torque_1 = 0.0;
    float input_torque_2 = 0.0;
    float input_torque_3 = 0.0;
    printf("Select mode (a-all, s-shoulder, e-elbow, w-wrist): ");
    scanf("%c", &input_mode);
    printf("\n");
    if(input_mode == 'a'){
      printf("Please input target torque of shoulder: ");
      scanf("%f", &input_torque_1);
      printf("elbow: ");
      scanf("%f", &input_torque_2);
      printf("wrist: ");
      scanf("%f", &input_torque_3);
      torque[1] = input_torque_1;
      torque[2] = input_torque_2;
      torque[3] = input_torque_3;
    }else if(input_mode == 's'){
      printf("Please input target torque of shoulder: ");
      scanf("%f", &input_torque_1);
      torque[1] = input_torque_1;
    }else if(input_mode == 'e'){
      printf("Please input target torque of elbow: ");
      scanf("%f", &input_torque_2);
      torque[2] = input_torque_2;
    }else if(input_mode == 'w'){
      printf("Please input target torque of wrist: ");
      printf("wrist: ");
      scanf("%f", &input_torque_3);
      torque[3] = input_torque_3;
    }
    printf("\n");
  }else if(cmd == 'c'){
    printf("Check joints are connected. ");
    for(int j=0; j<J_NUM; j++){
      if(j==0){
	jointCheck[j] = dAreConnected(arm[j].body, 0);
      }else{
	jointCheck[j] = dAreConnected(arm[j].body, arm[j-1].body);
      }
      // Should return 1 if are connected
      printf("Joint%d: %d", j, jointCheck[j]);	
      if(j==(J_NUM-1)){
	printf(". ");
      }else{
	printf(", ");
      }
    }
    printf("\n");
    for(int j=0; j<J_Stick_NUM; j++){
      if(j==0){
	jointCheck_Stick[j] = dAreConnected(stick[j].body, arm[L_NUM-1].body);
      }else{
	jointCheck_Stick[j] = dAreConnected(stick[j].body, stick[j-1].body);
      }
      // Should return 1 if are connected
      printf("StickJoint%d: %d", j, jointCheck_Stick[j]);		
      if(j==(J_Stick_NUM-1)){
	printf(". ");
      }else{
	printf(", ");
      }
    }
    printf("\n");
  }else if(cmd == 't'){
    // Reset
    dJointGroupDestroy(contactgroup);
    for(int i=0; i<L_NUM; i++){
      dBodyDestroy(arm[i].body);
      dGeomDestroy(arm[i].geom);
    }
    for(int i=0; i<L_Stick_NUM; i++){
      dBodyDestroy(stick[i].body);
      dGeomDestroy(stick[i].geom);
    }
    for(int j=0; j<J_NUM; j++){
      torque[j] = 0.00;
      THETA[j] = 0.00;
    }
    dBodyDestroy(ball.body);
    dGeomDestroy(ball.geom);
    contactgroup = dJointGroupCreate(0); 
    makeLink();
    printf("Reset\n");
  }else if(cmd == 'p'){
    // P control
    float angle_s_r, angle_s_p, angle_e_r;
    printf("Please input the 3 angles of shoulder, elbow, wrist.\n");
    printf("Shoulder):\n");
    scanf("%f", &angle_s_r);
    while(angle_s_r>=180 || angle_s_r<0){
      printf("Error! Shoulder(roll) should be set between 0~180 degree. Please input again:\n");
      scanf("%f", &angle_s_r);
    }
    printf("Elbow:\n");
    scanf("%f", &angle_s_p);
    while(angle_s_p>=180 || angle_s_p<0){
      printf("Error! Elbow should be set between 0~180 degree. Please input again:\n");
      scanf("%f", &angle_s_p);
    }
    printf("Wrist:\n");
    scanf("%f", &angle_e_r);
    while(angle_e_r>180 || angle_e_r<0){
      printf("Error! Wrist should be set between 0~180 degree. Please input again:\n");
      scanf("%f", &angle_e_r);
    }
    THETA[1] = angle_s_r * M_PI / 180;
    THETA[2] = angle_s_p * M_PI / 180;
    THETA[3] = angle_e_r * M_PI / 180;
  }
}

/* --------------------------------------------------
   System Functions
   -------------------------------------------------- */

// Set view point and the initial words on CUI
void start(){
  float xyz[3] = {    0.0f, 1.5f, 0.5f}; // position of observer (m)
  float hpr[3] = { -90.0f, -20.0f, 0.0f}; // angle of view(°)
  dsSetViewpoint(xyz, hpr);
  // Print on CUI
  printf("Use ctrl+x to end process.\n");
}

// Simulation loop
void simLoop(int pause){
  if(!pause){ // use ctrl+p to pause
    //Pcontrol();
    TorqueControl(); // torque control function
    dSpaceCollide(space, 0, &nearCallback); // collision detection
    dWorldStep(world, STEP_SIZE); // dynamic computation, change step time to tune the proper simulation speed
    dJointGroupEmpty(contactgroup); // empty joint
  }
#ifdef DRAW
  drawLink(); // draw link
#else
#endif	
}

// DrawStuff
void setDrawStuff(){
  fn.version = DS_VERSION; // version number
  fn.start   = &start; // start function
  fn.step    = &simLoop; // simLoop function
  fn.command = &command; // command function
  fn.stop    = NULL;
  fn.path_to_textures = TEXTURES_PATH;
}

/* --------------------------------------------------
   Main Process
   -------------------------------------------------- */

// Main
int main(int argc, char **argv){
  srand((unsigned) time(NULL));
  int box_num =0;
  for(int i=0;i<28;i++){
    for(int j=0;j<28;j++){
      for(int k=0;k<28;k++){
	box[k][j][i]  = box_num;
	box_num += 1;
      }
    }
  }
  /*
  for(int i=0;i<10;i++){
    for(int j=0;j<10;j++){
      for(int k=0;k<10;k++){
	printf("%d\n",boxnum_shift(k,j,i));
	
      }
    }
    }*/

  
  // for(para_mass=0; para_mass<10; para_mass++){
  para_mass = 0; 
  stick_link_mass=0.06+0.00333*para_mass;
    for(int j=0;j<5;j++){
      weight_Stick[j] = stick_link_mass;
    }
    for(para_kp=0; para_kp<28; para_kp++){
      kp=2.0+0.333*para_kp;
      for(para_kd=0; para_kd<28; para_kd++){
	kd=0.5+0.0666*para_kd;
	erp = STEP_SIZE * kp / (STEP_SIZE * kp + kd);
	cfm = 1.0 / (STEP_SIZE * kp + kd);
	while(1){
	
	dInitODE();// initialize ODE
	setDrawStuff();// Prepare DrawStuff
	world = dWorldCreate();// build world
	space = dHashSpaceCreate(0);// build space
	ground = dCreatePlane(space, 0, 0, 1, 0);// make ground
	dWorldSetGravity(world, 0, 0, EARTH_GRAVITY);// set gravity
	printf("%d_%d_%d\n",para_mass,para_kp,para_kd);
	contactgroup = dJointGroupCreate(0);// make contact group
	makeLink();	
#ifdef DRAW // build link
	dsSimulationLoop(argc, argv, 640, 480, &fn); // main simulation loop
#else
	while (1) {
	  simLoop(0);
	  //printf("%lf\n",timetime);
	  //printf("%lf\n",kd);
	  if(timetime>=timelimit+150){
	    break;
	  }
	}
#endif
	if (timetime==timelimit+150){
	  const  dReal *ball_pos = dBodyGetPosition(ball.body);
	  if(ball_pos[0]==x_Ball && ball_pos[1]==y_Ball){
	    value = -1-mindistance;
	  }
	  else{
	    value = (-x_Ball*(ball_pos[0]-x_Ball)+(-y_Ball*(ball_pos[1]-y_Ball)))/sqrt(((x_Ball*x_Ball)+(y_Ball*y_Ball))*(((ball_pos[0]-x_Ball)*(ball_pos[0]-x_Ball))+((ball_pos[1]-y_Ball)*(ball_pos[1]-y_Ball))));
	  }
	  if(value > max_value){
	    max_value = value;
	    for(int i = 0;i<time_sep;i++){
	      for(int j = 0;j<J_NUM;j++){
		max_goal_torque[i][j] = goal_torque[i][j];
	      }
	    }
	    for(int i = 0;i<time_sep-1;i++){
	      max_step[i] = step[i];
	    }
	  }
	  printf("spring,counter:%f,%d\n",kp,counter);
	  printf("now_value:%f\n",value);
	  printf("max_value:%f\n",max_value);
	  //reset
	  for(int j=0; j<J_NUM; j++){
	    torque[j] = 0.00;
	    THETA[j] = 0.00;
	  }
	  timetime  = 0;
	  nowstep = 0;
	  collideflag = 0;
	  value = 0;
	  mindistance = 10000;
	  
	  for(int i=0;i<time_sep-1;i++){
	    step[i]=0;
	  }
	  //valuelog[counter] = max_value;
	  counter += 1;
	}
	
	dSpaceDestroy(space); // destroy the space
	dWorldDestroy(world); // destroy the world
	dCloseODE();
	
	if (counter>500){
	  max_value = -10000;
	  counter = 0;
	  mode = 2;
	  m1iterate = 0;
	  m21iterate = 0;
	  m22iterate = 0;
	  makelinkon = 1;
	}
	
	
	if(counter >= 300&&max_value>=0.9999){
	  for(int i = 0;i<time_sep;i++){
	    for(int j = 0;j<J_NUM-1;j++){
	      tmp_data[i][j] = max_goal_torque[i][j+1];
	    }
	  }
	  tmp_data[0][3] =0;
	  for(int i = 0;i<time_sep-1;i++){
	    tmp_data[i+1][3] =  max_step[i];
	  }
	     
	  output_data1();
	  output_data2();
	  for(int i = 0; i < time_sep; i++){
	    for(int j = 0; j < J_NUM; j++){
	      max_goal_torque_box[i][j][boxnum(para_kd,para_kp,para_mass)] = max_goal_torque[i][j];
	    }
	  }
	  for(int i = 0; i < time_sep-1; i++){
	    max_step_box[i][boxnum(para_kd,para_kp,para_mass)] = max_step[i];
	  }
	  max_value = -10000;
	  counter = 0;
	  mode = 2;
	  m1iterate = 0;
	  m21iterate = 0;
	  m22iterate = 0;
	  makelinkon = 1;
	  for(int j=0; j<time_sep; j++){
	    for(int jj=0; jj<4; jj++){
	     tmp_data[j][jj] =0;
	    }
	  }  
	  break;
	}
	
	}
      }
    }
    // }									// end of ODE
  return 0;
}



