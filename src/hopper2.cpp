#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

//using namespace std;

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#endif

dWorldID world;  // dynamic simulation world
dSpaceID space;  // contact dection space
dGeomID  ground; // ground
dJointGroupID contactgroup; // contact group
dReal r = 0.2, m  = 1.0;
dsFunctions fn; // draw function of drawn stuff

typedef struct {       // MyObject構造体
  dBodyID body;        // ボディ(剛体)のID番号（動力学計算用）
  dGeomID geom;        // ジオメトリのID番号(衝突検出計算用）
  double  l,r,m;       // 長さ[m], 半径[m]，質量[kg]
} MyObject;

static int STEPS = 0; // simulation step number

//MyObject foot;
//MyObject l_leg;

//#define  NUM 4        // link number
//MyObject rlink[NUM];  // number
//dReal    THETA[NUM] = {0.0}; // joint target angle [rad]
//dJointID joint[NUM]; // joint ID number

#define  NUM 4        // link number
MyObject rlink[NUM];  // number
dJointID joint[NUM]; // joint ID number

/*
#define  NUM_l 4        // link number
#define  NUM_j 3        // link number
MyObject rlink[NUM_l];  // number
dReal    THETA[NUM_j]; // joint target angle [rad]
dJointID joint[NUM_j]; // joint ID number

dReal THETA_i0 = - 2.36;
dReal THETA_i1 = + 1.57;
dReal THETA_i2 = - 1.57;
*/
//MyObject foot, l_leg, u_leg, trunk;
//dJointID ankle, knee, hip;
//dReal THETA_ankle, THETA_knee, THETA_hip;

// make the arm
void  makeArm()
{
  dMass mass;                                    // 質量パラメータ

  dMatrix3 R;
  //dReal x[NUM]      = {0.00, 0.00, 0.00, 0.00};  // 重心 x
  //dReal y[NUM]      = {0.00, 0.00, 0.00, 0.00};  // 重心 y
  //dReal y[NUM]      = {0.00, 0.00, 0.05, 0.00};  // 重心 y
  //dReal z[NUM]      = {0.05, 0.50, 1.50, 2.50};  // 重心 z
  //dReal length[NUM] = {0.20, 0.30, 0.30, 0.50};  // 長さ
  //dReal weight[NUM] = {0.50, 1.00, 1.00, 3.00};  // 質量
  //dReal r[NUM]      = {0.02, 0.02, 0.02, 0.10};  // 半径
  //dReal c_x[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 x
  //dReal c_y[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 y
  //dReal c_z[NUM]    = {0.00, 0.00, 0.10, 2.00};  // 関節中心点 z
  //dReal axis_x[NUM] = {0, 0, 0, 0};              // joint axis x 
  //dReal axis_y[NUM] = {0, 0, 1, 1};              // joint axis y 
  //dReal axis_z[NUM] = {1, 1, 0, 0};              // joint axis z 
  //dReal axis_x[NUM] = {0, 0, 0, 0};              // joint axis x 
  //dReal axis_y[NUM] = {1, 1, 1, 1};              // joint axis y 
  //dReal axis_z[NUM] = {0, 0, 0, 0};              // joint axis z 
  //dReal NUM_l = 4;
  dReal length[NUM] = {0.20, 0.30, 0.30, 0.50}; 
  dReal weight[NUM] = {0.50, 1.00, 1.00, 3.00};
  dReal r[NUM]      = {0.02, 0.02, 0.02, 0.02};  // radius
  dReal axis_x = 0; // joint axis x 
  dReal axis_y = 1; // joint axis y 
  dReal axis_z = 0; // joint axis z 

  dReal x[NUM], y[NUM], z[NUM];  
  dReal c_x[NUM], c_y[NUM], c_z[NUM];   

  dReal Pi = 3.14159;
  dReal theta[NUM] = { Pi, Pi/6.0, 5.0*Pi/6.0, Pi/3.0}; 
  //dReal theta[NUM] = { 1.0*Pi/12.0, 2.0*Pi/12.0, 3.0*Pi/12.0, 4.0*Pi/12.0};
  //dReal theta[NUM] = { 0.0, 2.0*Pi/12.0, 0.0, 0.0};

  //x[0] = 0; y[0] = 0; z[0] = r[0];
  //x[0] = 0; y[0] = 0; z[0] = 1.1* r[0];

  //c_x[0] = x[0] + 0.5* length[0]* cos(theta[0]); 
  //c_y[0] = y[0];
  //c_z[0] = z[0] + 0.5* length[0]* sin(theta[0]); 

  c_x[0] = 0; c_y[0] = 0; c_z[0] = 1.2* r[0] + 0.5* length[0]* sin(theta[0]);

  for (int i = 1; i < NUM; i++) {
    c_x[i] = c_x[i-1] + 1.0* length[i-1]* cos(theta[i-1]); 
    c_y[i] = c_y[i-1];
    c_z[i] = c_z[i-1] + 1.0* length[i-1]* sin(theta[i-1]); 
  }

  //x[0] = c_x[0] - 0.5* length[0]* cos(theta[0]); 
  //y[0] = c_y[0];
  //z[0] = c_z[0] - 0.5* length[0]* sin(theta[0]); 

  //x[1] = c_x[0] + 0.5* length[1]* cos(theta[1]); 
  //y[1] = c_y[0];
  //z[1] = c_z[0] + 0.5* length[1]* sin(theta[1]); 

  //c_x[1] = c_x[0] + 1.0* length[1]* cos(theta[1]); 
  //c_y[1] = c_y[0];
  //c_z[1] = c_z[0] + 1.0* length[1]* sin(theta[1]); 

  for (int i = 0; i < NUM; i++) {
    x[i]   = c_x[i] + 0.5* length[i]* cos(theta[i]); 
    y[i]   = c_y[i];
    z[i]   = c_z[i] + 0.5* length[i]* sin(theta[i]); 
  }

  for (int i = 0; i < NUM; i++) {
    rlink[i].body  = dBodyCreate( world);
    dBodySetPosition( rlink[i].body,  x[i], y[i], z[i]);
    //dRFromAxisAndAngle( R, axis_x, axis_y, axis_z, theta[i] - Pi/2.0);
    dRFromAxisAndAngle( R, axis_x, axis_y, axis_z, - theta[i] - Pi/2.0);
    //dRFromAxisAndAngle( R, axis_x, axis_y, axis_z, theta[i]);
    dBodySetRotation( rlink[i].body, R);
    dMassSetZero( &mass);
    dMassSetCapsuleTotal( &mass, weight[i], 3, r[i], length[i]);
    dBodySetMass( rlink[i].body, &mass);
    rlink[i].geom  = dCreateCapsule( space, r[i], length[i]);
    dGeomSetBody( rlink[i].geom, rlink[i].body);
  }

  joint[0] = dJointCreateHinge( world, 0); // hinge

  for (int j = 1; j < NUM; j++) {
    joint[j] = dJointCreateHinge( world, 0); // hinge
    dJointAttach( joint[j], rlink[j].body, rlink[j-1].body);
    dJointSetHingeAnchor( joint[j], c_x[j], c_y[j], c_z[j]);
    dJointSetHingeAxis( joint[j], axis_x, axis_y, axis_z);
  }

  //for (int i = 0; i < NUM_l; i++) {
  //printf("%lf\t%lf\t%lf\n", c_x[i], c_y[i], c_z[i]);
  //}

  //dMassSetZero( &mass);
  //dMassSetCapsuleTotal( &mass, weight[1], 3, r[1], length[1]);
  //dBodySetMass( l_leg.body, &mass);

  //l_leg.geom = dCreateCapsule( space, r[0], length[0]);

  //dGeomSetBody( l_leg.geom, l_leg.body);


  /*
  dReal x[NUM_l], y[NUM_l], z[NUM_l];  // link CoM position
  dReal c_x[NUM_j], c_y[NUM_j], c_z[NUM_j]; // joint axis

  dReal length[NUM_l] = {0.20, 0.30, 0.30, 0.50};  // link length
  dReal weight[NUM_l] = {0.50, 1.00, 1.00, 3.00};  // link weight
  dReal r[NUM_l]      = {0.02, 0.02, 0.02, 0.02};  // 半径

  // set initial angle
  THETA[0] = THETA_i0;
  THETA[1] = THETA_i1;
  THETA[2] = THETA_i2;

  // make links
  c_x[0] = 0; c_y[0] = 0; c_z[0] = 1.0;
  x[0] = c_x[0] + 0.5* length[0]; y[0] = c_y[0]; z[0] = c_z[0]; 

  for (int i = 1; i < NUM_l; i++) {
    x[i] = x[i-1] + 0.5* length[i]* cos( THETA[i-1]);
    y[i] = 0.0;
    z[i] = z[i-1] + 0.5* length[i]* sin( THETA[i-1]);
  }
  for (int i = 1; i < NUM_j; i++) {
    c_x[i] = c_x[i-1] + 1.0* length[i]* cos( THETA[i-1]);
    c_y[i] = 0.0;
    c_z[i] = c_z[i-1] + 1.0* length[i]* sin( THETA[i-1]);
  }

  // make links
  for (int i = 0; i < NUM_l; i++) {
    rlink[i].body = dBodyCreate( world);
    dBodySetPosition( rlink[i].body, x[i], y[i], z[i]);

    if (i==0){    
      dRFromAxisAndAngle( R, axis_x, axis_y, axis_z, 1.57);
    }else{
      dRFromAxisAndAngle( R, axis_x, axis_y, axis_z, THETA[i - 1]);
    }
    dBodySetRotation( rlink[i].body, R);

    dMassSetZero( &mass);
    dMassSetCapsuleTotal( &mass, weight[i], 3, r[i], length[i]);
    dBodySetMass( rlink[i].body, &mass);
    rlink[i].geom = dCreateCapsule( space, r[i], length[i]);
    dGeomSetBody( rlink[i].geom, rlink[i].body);
  }
  */
  // make joints, attach them to links
  //joint[0] = dJointCreateFixed(world, 0);  // fixed joint
  //dJointAttach(joint[0], rlink[0].body, 0);
  //dJointSetFixed(joint[0]);
  /*
  for (int j = 0; j < NUM_j; j++) {
    joint[j] = dJointCreateHinge(world, 0); // hinge
    dJointAttach(joint[j], rlink[j + 1].body, rlink[j].body);
    dJointSetHingeAnchor(joint[j], c_x[j], c_y[j], c_z[j]);
    //dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j],axis_z[j]);
    dJointSetHingeAxis( joint[j], axis_x, axis_y, axis_z);
  }
  */
  //for (int j = 1; j < NUM; j++) {
  //for (int j = 0; j < NUM; j++) {
  //joint[j] = dJointCreateHinge(world, 0); // hinge
  //dJointAttach(joint[j], rlink[j].body, rlink[j-1].body);
  //dJointSetHingeAnchor(joint[j], c_x[j], c_y[j], c_z[j]);
  //dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j],axis_z[j]);
  //}
}

// draw arm
void drawArm()
{
   dReal r,length;
   for (int i = 0; i < NUM; i++ ) { // draw capsule
   //for (int i = 0; i < NUM_l; i++ ) { // draw capsule
   //for (int i = 0; i < 2; i++ ) { // draw capsule
     dGeomCapsuleGetParams( rlink[i].geom, &r, &length);
     dsDrawCapsule(dBodyGetPosition( rlink[i].body), dBodyGetRotation(rlink[i].body), length, r);
   }
   //dGeomCapsuleGetParams( foot.geom, &r, &length);
   //dsDrawCapsule( dBodyGetPosition(foot.body), dBodyGetRotation(foot.body), length, r);
}

// collison detection calculation
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int N = 7; // collision point number
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  // 2つのボディがジョイントで結合されていたら衝突検出しない
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (isGround)  {
    for (int i = 0; i < n; i++) {
      contact[i].surface.mode   = dContactBounce | dContactSoftERP |
                                  dContactSoftCFM;
      contact[i].surface.soft_erp   = 0.2;   // 接触点のERP
      contact[i].surface.soft_cfm   = 0.001; // 接触点のCFM
      contact[i].surface.mu     = dInfinity; // 摩擦係数:無限大
      dJointID c = dJointCreateContact(world,
                                       contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
    }
  }
}

void destroyArm()
// destroy the arm
{
  //dBodyDestroy(foot.body); // destroy body
  //dGeomDestroy(foot.geom); // destroy geometory
  for (int i = 0; i < NUM; i++) {
  //for (int i = 0; i < NUM_l; i++) {
  //for (int i = 0; i < 2; i++) {
    dJointDestroy(joint[i]);     // destroy joint 
    dBodyDestroy(rlink[i].body); // destroy body
    dGeomDestroy(rlink[i].geom); // destroy geometory
  }
  //for (int i = 0; i < (NUM_l - 1); i++)
  //dJointDestroy(joint[i]);     // destroy joint 

  //for (int i = 0; i < NUM_j; i++)
  //dJointDestroy(joint[i]);     // destroy joint 
}

/*
void Pcontrol()
// P control
{
  dReal k =  10.0, fMax = 100.0;                   // 比例ゲイン，最大トルク

  //for (int j = 1; j < NUM; j++) {
  for (int j = 1; j < NUM_j; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    dReal z = THETA[j] - tmp;                      // 残差
    dJointSetHingeParam(joint[j],dParamVel, k*z);  // 角速度の設定
    dJointSetHingeParam(joint[j],dParamFMax,fMax); // トルクの設定
  }
}
*/
static void restart()
// simulation restart
{
  STEPS    = 0;                        // initialize step number

  destroyArm();                        // destroy arm
  dJointGroupDestroy(contactgroup);    // destroy joint group
  contactgroup = dJointGroupCreate(0); // create joint group
  makeArm();                           // make robot
}

// simulation loop
static void simLoop(int pause)
{
  if (!pause) {
    STEPS++;
    //Pcontrol();
    dSpaceCollide(space,0,&nearCallback);
    dWorldStep(world,0.01);
    dJointGroupEmpty(contactgroup);

    //printf("%d\n",STEPS);

    if (STEPS > 50){
    //if (STEPS > 10000){
      STEPS = 0;
      restart();
    }
    
  }
  drawArm(); // draw robot
}

static void start()
{
  //static float xyz[3] = {   3.5, 0.0, 1.0};
  //static float hpr[3] = {-180.0, 0.0, 0.0};
  //static float xyz[3] = {   2.0, 0.0, 0.5};
  //static float hpr[3] = {-180.0, 0.0, 0.0};
  static float xyz[3] = {  0.0, 1.5, 0.5};
  static float hpr[3] = {-90.0, 0.0, 0.0};
  
  dsSetViewpoint(xyz,hpr);               // 視点，視線の設定
  dsSetSphereQuality(3);                 // 球の品質設定
}

void setDrawStuff()
// setup of draw functions
{
  fn.version = DS_VERSION;    // ドロースタッフのバージョン
  fn.start   = &start;        // 前処理 start関数のポインタ
  fn.step    = &simLoop;      // simLoop関数のポインタ
  fn.path_to_textures = "../../drawstuff/textures"; // テクスチャ
}


int main (int argc, char *argv[])
{
  dInitODE();
  setDrawStuff();

  world        = dWorldCreate();
  space        = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  dWorldSetGravity(world, 0,0, -9.8);
  dWorldSetERP(world, 0.9);          // ERPの設定
  dWorldSetCFM(world, 1e-4);         // CFMの設定
  ground = dCreatePlane(space, 0, 0, 1, 0);
  makeArm();

  dsSimulationLoop (argc, argv, 640, 480, &fn);

  dWorldDestroy (world);
  dCloseODE();

  return 0;
}
