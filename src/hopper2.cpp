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

#define  NUM 4        // link number
MyObject rlink[NUM];  // number
dJointID joint[NUM]; // joint ID number

// make the arm
void  makeArm()
{
  dMass mass; // mass parameter
  dMatrix3 R;
  dReal length[NUM] = {0.20, 0.30, 0.30, 0.50}; 
  dReal weight[NUM] = {0.50, 1.00, 1.00, 3.00};
  dReal r[NUM]      = {0.02, 0.02, 0.02, 0.02}; // radius
  dReal axis_x = 0; // joint axis x 
  dReal axis_y = 1; // joint axis y 
  dReal axis_z = 0; // joint axis z 

  dReal x[NUM], y[NUM], z[NUM];  
  dReal c_x[NUM], c_y[NUM], c_z[NUM];   

  dReal Pi = 3.14159;
  dReal theta[NUM] = { Pi, Pi/6.0, 5.0*Pi/6.0, Pi/3.0}; 

  c_x[0] = 0; c_y[0] = 0; c_z[0] = 1.2* r[0] + 0.5* length[0]* sin(theta[0]);

  for (int i = 1; i < NUM; i++) {
    c_x[i] = c_x[i-1] + 1.0* length[i-1]* cos(theta[i-1]); 
    c_y[i] = c_y[i-1];
    c_z[i] = c_z[i-1] + 1.0* length[i-1]* sin(theta[i-1]); 
  }
  for (int i = 0; i < NUM; i++) {
    x[i]   = c_x[i] + 0.5* length[i]* cos(theta[i]); 
    y[i]   = c_y[i];
    z[i]   = c_z[i] + 0.5* length[i]* sin(theta[i]); 
  }

  for (int i = 0; i < NUM; i++) {
    rlink[i].body  = dBodyCreate( world);
    dBodySetPosition( rlink[i].body,  x[i], y[i], z[i]);
    dRFromAxisAndAngle( R, axis_x, axis_y, axis_z, - theta[i] - Pi/2.0);
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

}

// draw arm
void drawArm()
{
   dReal r,length;
   for (int i = 0; i < NUM; i++ ) { // draw capsule
     dGeomCapsuleGetParams( rlink[i].geom, &r, &length);
     dsDrawCapsule(dBodyGetPosition( rlink[i].body), dBodyGetRotation(rlink[i].body), length, r);
   }
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
  for (int i = 0; i < NUM; i++) {
    dJointDestroy(joint[i]);     // destroy joint 
    dBodyDestroy(rlink[i].body); // destroy body
    dGeomDestroy(rlink[i].geom); // destroy geometory
  }
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
  static float xyz[3] = {  0.0, 1.5, 0.5};
  static float hpr[3] = {-90.0, 0.0, 0.0};
  
  dsSetViewpoint( xyz, hpr); // viewpoint, direction setting
  dsSetSphereQuality(3);     // sphere quality setting
}

void setDrawStuff()
// setup of draw functions
{
  fn.version = DS_VERSION;    // ドロースタッフのバージョン
  fn.start   = &start;        // 前処理 start関数のポインタ
  fn.step    = &simLoop;      // simLoop関数のポインタ
  fn.path_to_textures = "../../drawstuff/textures"; // texture path
}


int main (int argc, char *argv[])
{
  dInitODE();
  setDrawStuff();

  world        = dWorldCreate();
  space        = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  dWorldSetGravity(world, 0,0, -9.8);
  dWorldSetERP(world, 0.9);          // ERP setting
  dWorldSetCFM(world, 1e-4);         // CFM setting
  ground = dCreatePlane(space, 0, 0, 1, 0);
  makeArm();

  dsSimulationLoop (argc, argv, 640, 480, &fn);

  dWorldDestroy (world);
  dCloseODE();

  return 0;
}
