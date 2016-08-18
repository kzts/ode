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

static int STEPS = 0;            // シミュレーションのステップ数

#define  NUM 4                          // link number
MyObject rlink[NUM];                   // number
dReal    THETA[NUM] = {0.0};             // joint target angle [rad]
dJointID joint[NUM]; // joint ID number

// make the arm
void  makeArm()
{
  dMass mass;                                    // 質量パラメータ
  dReal x[NUM]      = {0.00, 0.00, 0.00, 0.00};  // 重心 x
  dReal y[NUM]      = {0.00, 0.00, 0.00, 0.00};  // 重心 y
  dReal z[NUM]      = {0.05, 0.50, 1.50, 2.50};  // 重心 z
  dReal length[NUM] = {0.10, 0.90, 1.00, 1.00};  // 長さ
  dReal weight[NUM] = {9.00, 2.00, 2.00, 2.00};  // 質量
  dReal r[NUM]      = {0.20, 0.04, 0.04, 0.04};  // 半径
  dReal c_x[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 x
  dReal c_y[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 y
  dReal c_z[NUM]    = {0.00, 0.10, 1.00, 2.00};  // 関節中心点 z
  dReal axis_x[NUM] = {0, 0, 0, 0};              // 関節回転軸 x
  dReal axis_y[NUM] = {0, 0, 1, 1};              // 関節回転軸 y
  dReal axis_z[NUM] = {1, 1, 0, 0};              // 関節回転軸 z
  
  // make links
  for (int i = 0; i < NUM; i++) {
    rlink[i].body = dBodyCreate(world);
    dBodySetPosition(rlink[i].body, x[i], y[i], z[i]);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,weight[i],3,r[i],length[i]);
    dBodySetMass(rlink[i].body, &mass);
    rlink[i].geom = dCreateCapsule(space,r[i],length[i]);
    dGeomSetBody(rlink[i].geom,rlink[i].body);
  }

  // make joints, attach them to links
  joint[0] = dJointCreateFixed(world, 0);  // fixed joint
  dJointAttach(joint[0], rlink[0].body, 0);
  dJointSetFixed(joint[0]);
  for (int j = 1; j < NUM; j++) {
    joint[j] = dJointCreateHinge(world, 0); // hinge
    dJointAttach(joint[j], rlink[j].body, rlink[j-1].body);
    dJointSetHingeAnchor(joint[j], c_x[j], c_y[j], c_z[j]);
    dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j],axis_z[j]);
  }
}

// draw arm
void drawArm()
{
   dReal r,length;

   for (int i = 0; i < NUM; i++ ) { // draw capsule
     dGeomCapsuleGetParams(rlink[i].geom, &r,&length);
     dsDrawCapsule(dBodyGetPosition(rlink[i].body),
     dBodyGetRotation(rlink[i].body),length,r);
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

// destroy the arm
void destroyArm()
{
  for (int i = 0; i < NUM; i++) {
    dJointDestroy(joint[i]);     // destroy joint 
    dBodyDestroy(rlink[i].body); // destroy body
    dGeomDestroy(rlink[i].geom); // destroy geometory
  }
}

// P control
void Pcontrol()
{
  dReal k =  10.0, fMax = 100.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < NUM; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    dReal z = THETA[j] - tmp;                      // 残差
    dJointSetHingeParam(joint[j],dParamVel, k*z);  // 角速度の設定
    dJointSetHingeParam(joint[j],dParamFMax,fMax); // トルクの設定
  }
}

// simulation restart
static void restart()
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
    Pcontrol();
    dSpaceCollide(space,0,&nearCallback);
    dWorldStep(world,0.01);
    dJointGroupEmpty(contactgroup);

    printf("%d\n",STEPS);

    if (STEPS > 100){
      STEPS = 0;
      restart();
    }
    
  }
  drawArm(); // draw robot
}

static void start()
{
  static float xyz[3] = {   3.5, 0.0, 1.0};
  static float hpr[3] = {-180.0, 0.0, 0.0};
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
