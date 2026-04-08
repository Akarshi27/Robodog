#include <Wire.h>
#include <Servo.h>
#include <math.h>

/* ============================================================
   SERVO MAP
   ============================================================ */
Servo s[8];
const int SERVO_PIN[8] = {7,8,5,4,6,9,2,3};
// 0 FL hip |1 FL knee |2 FR hip |3 FR knee 
// 4 BL hip |5 BL knee |6 BR hip |7 BR knee

bool invertHip[8]  = {0,0,1,0,0,0,1,0};
bool invertKnee[8] = {0,0,0,1,0,0,0,1};

/* ============================================================
   ROBOT GEOMETRY / GAIT
   ============================================================ */
const float L1 = 15.0f;
const float L2 = 15.0f;

float xNeutralF = 6.5f;
float xNeutralB = 6.0f;

float zStand = 12.5f;
float zSit   = 5.0f;

float strideLen = 7.0f;
float clrFL = 1.2f;
float clrFR = 1.6f;
float clrBL = 1.6f;
float clrBR = 1.4f;

float beta = 0.58f;
int   N    = 70;
int   dt   = 16;

const float HIP_OFFSET     = 90.0f;
const float KNEE_STAND_DEG = 126.87f;
const float KNEE_OFFSET    = 90.0f;

/* ============================================================
   HEIGHT BIAS
   ============================================================ */
float zBias_FL = +1.2f;
float zBias_FR =  0.0f;
float zBias_BL = +0.3f;
float zBias_BR = -0.3f;

/* ============================================================
   TRIMS
   ============================================================ */
float trimAngles[8] = {
  0,0,
  0,+1,
  +0.5,+0.5,
  +2,+3
};

/* ============================================================
   MPU9250 CONFIG
   ============================================================ */
#define MPU_ADDR 0x68

int16_t ax,ay,az,gx,gy,gz;
float gxo=0, gyo=0, gzo=0;

float anglePitch=0, angleRoll=0;
float pitchZero=0, rollZero=0;
float pitchFilt=0, rollFilt=0;

float imuLPF = 0.90f;

unsigned long prevMicros;

int ORIENT_SIGN_ROLL  = +1;
int ORIENT_SIGN_PITCH = +1;

float KpRollHip   = 0.22f;
float KpRollKnee  = 0.12f;
float KpPitchHip  = 0.16f;
float KpPitchKnee = 0.08f;

float KzRoll  = 0.010f;
float KzPitch = 0.008f;

/* ============================================================
   RC INPUT SETTINGS
   ============================================================ */
#define CH2 10

int ch2Val;
int mid = 1417;
int dz  = 12; // Dead-zone = ±12

/* ============================================================
   STATES
   ============================================================ */
bool isStanding = true;

/* ============================================================
   IMU FUNCTIONS
   ============================================================ */
void mpuReadRaw(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14,true);

  ax = (Wire.read()<<8)|Wire.read();
  ay = (Wire.read()<<8)|Wire.read();
  az = (Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read();
  gx = (Wire.read()<<8)|Wire.read();
  gy = (Wire.read()<<8)|Wire.read();
  gz = (Wire.read()<<8)|Wire.read();
}

void mpuInit(){
  Wire.begin();
  delay(50);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);

  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission();

  for(int i=0;i<800;i++){
    mpuReadRaw();
    gxo+=gx; gyo+=gy; gzo+=gz;
    delay(2);
  }
  gxo/=800; gyo/=800; gzo/=800;

  float ps=0, rs=0;
  for(int i=0;i<400;i++){
    mpuReadRaw();
    float axf=ax, ayf=ay, azf=az;

    float rawPitch = atan2f(ayf, sqrtf(axf*axf+azf*azf))*180/M_PI;
    float rawRoll  = atan2f(-axf, azf)*180/M_PI;

    ps+=rawPitch;
    rs+=rawRoll;
    delay(2);
  }
  pitchZero=ps/400.0;
  rollZero =rs/400.0;

  prevMicros = micros();
}

void mpuUpdate(){
  mpuReadRaw();

  float dt = (micros()-prevMicros)/1000000.0f;
  prevMicros = micros();

  float gxf=(gx-gxo)/131.0f;
  float gyf=(gy-gyo)/131.0f;

  anglePitch += gxf*dt;
  angleRoll  += gyf*dt;

  float axf=ax, ayf=ay, azf=az;
  float rawPitch = atan2f(ayf, sqrtf(axf*axf+azf*azf))*180/M_PI;
  float rawRoll  = atan2f(-axf, azf)*180/M_PI;

  anglePitch = anglePitch*0.98 + rawPitch*0.02;
  angleRoll  = angleRoll *0.98 + rawRoll *0.02;

  float Pitch=(anglePitch-pitchZero)*ORIENT_SIGN_PITCH;
  float Roll =(angleRoll -rollZero )*ORIENT_SIGN_ROLL;

  pitchFilt=imuLPF*pitchFilt + (1-imuLPF)*Pitch;
  rollFilt =imuLPF*rollFilt  + (1-imuLPF)*Roll;
}

/* ============================================================
   IK + AUTO LEVEL
   ============================================================ */
float clip1(float x){ if(x>1)x=1; if(x<-1)x=-1; return x; }
float degF(float r){ return r*180.0/M_PI; }

bool legIK(float x,float z,float &hip,float &knee,int sH,int sK){
  float r2=x*x+z*z;
  float r=sqrtf(r2);
  float rMin=fabs(L1-L2)+0.1f;
  float rMax=(L1+L2)-0.1f;
  if(r<rMin)r=rMin;
  if(r>rMax)r=rMax;

  float cK=clip1((r2-L1*L1-L2*L2)/(2*L1*L2));
  float tK=acosf(cK);

  float phi=atan2f(z,x);
  float psi=atan2f(L2*sinf(tK), L1+L2*cosf(tK));

  hip = HIP_OFFSET + sH * degF(phi-psi);
  float kneeGeom=degF(tK);
  knee = KNEE_OFFSET + sK*(kneeGeom-KNEE_STAND_DEG);

  hip=constrain(hip,0,180);
  knee=constrain(knee,0,180);
  return true;
}

void writeLeg(int hipIdx,int kneeIdx,float x,float z,bool isFront,bool isLeft){

  float zBias =
    isFront ? (isLeft?zBias_FL:zBias_FR)
            : (isLeft?zBias_BL:zBias_BR);

  float zCorrRoll  = (isLeft? -KzRoll*rollFilt : +KzRoll*rollFilt);
  float zCorrPitch = (isFront? -KzPitch*pitchFilt : +KzPitch*pitchFilt);

  float zCmd=z+zBias+zCorrRoll+zCorrPitch;

  float h,k;
  int sH=invertHip[hipIdx]?-1:+1;
  int sK=invertKnee[kneeIdx]?-1:+1;

  legIK(x,zCmd,h,k,sH,sK);

  if(isLeft){ h+=KpRollHip*rollFilt;  k+=KpRollKnee*rollFilt; }
  else      { h-=KpRollHip*rollFilt;  k-=KpRollKnee*rollFilt; }

  if(isFront){ h-=KpPitchHip*pitchFilt; k-=KpPitchKnee*pitchFilt; }
  else       { h+=KpPitchHip*pitchFilt; k+=KpPitchKnee*pitchFilt; }

  h+=trimAngles[hipIdx];
  k+=trimAngles[kneeIdx];

  s[hipIdx].write(constrain(h,0,180));
  s[kneeIdx].write(constrain(k,0,180));
}

/* ============================================================
   GAIT ENGINE (FORWARD)
   ============================================================ */
void footTrajectory(float p,float S,float h,float z0,float &x,float &z){
  if(p<beta){
    float t=p/beta;
    x=+S*0.5f-S*t;
    z=z0;
  }else{
    float t=(p-beta)/(1-beta);
    x=-S*0.5f+S*t;
    float lift=4*t*(1-t);
    z=z0-h*lift;
  }
}

float wrap01(float p){ if(p>=1)return p-1; if(p<0)return p+1; return p; }

void poseForward(float p){
  float pFL=wrap01(p-0.03f);
  float pBR=pFL;
  float pFR=wrap01(p+0.5f);
  float pBL=pFR;

  float x,z;

  footTrajectory (pFL,strideLen,clrFL,zStand,x,z);
  writeLeg(0,1,xNeutralF+x,z,true,true);

  footTrajectory (pFR,strideLen,clrFR,zStand,x,z);
  writeLeg(2,3,xNeutralF+x,z,true,false);

  footTrajectory (pBL,strideLen,clrBL,zStand,x,z);
  writeLeg(4,5,xNeutralB+x,z,false,true);

  footTrajectory (pBR,strideLen,clrBR,zStand,x,z);
  writeLeg(6,7,xNeutralB+x,z,false,false);
}

/* ============================================================
   GAIT ENGINE (BACKWARD)
   ============================================================ */
void footTrajectoryBackward(float p,float S,float h,float z0,float &x,float &z){
  if(p < beta){
    float t = p/beta;
    x = -S*0.5f + S*t;
    z = z0;
  } else {
    float t = (p-beta)/(1-beta);
    x = +S*0.5f - S*t;
    float lift=4*t*(1-t);
    z = z0 - h*lift;
  }
}

void poseBackward(float p){
  float pFL=wrap01(p-0.03f);
  float pBR=pFL;
  float pFR=wrap01(p+0.5f);
  float pBL=pFR;

  float x,z;

  footTrajectoryBackward(pFL,strideLen,clrFL,zStand,x,z);
  writeLeg(0,1,xNeutralF+x,z,true,true);

  footTrajectoryBackward(pFR,strideLen,clrFR,zStand,x,z);
  writeLeg(2,3,xNeutralF+x,z,true,false);

  footTrajectoryBackward(pBL,strideLen,clrBL,zStand,x,z);
  writeLeg(4,5,xNeutralB+x,z,false,true);

  footTrajectoryBackward(pBR,strideLen,clrBR,zStand,x,z);
  writeLeg(6,7,xNeutralB+x,z,false,false);
}

/* ============================================================
   STAND STILL POSE
   ============================================================ */
void standStill(){
  mpuUpdate();
  writeLeg(0,1,xNeutralF,zStand,true,true);
  writeLeg(2,3,xNeutralF,zStand,true,false);
  writeLeg(4,5,xNeutralB,zStand,false,true);
  writeLeg(6,7,xNeutralB,zStand,false,false);
}

/* ============================================================
   SETUP
   ============================================================ */
void setup(){
  Serial.begin(115200);
  Wire.begin();

  for(int i=0;i<8;i++) s[i].attach(SERVO_PIN[i]);

  pinMode(CH2,INPUT);

  mpuInit();

  // ✅ Start directly in standing pose
  standStill();
  delay(2000);
}

/* ============================================================
   MAIN LOOP
   ============================================================ */
void loop(){

  ch2Val = pulseIn(CH2,HIGH,30000);

  if(ch2Val<900 || ch2Val>2100){
    standStill();
    return;
  }

  // ✅ CENTER RANGE → STAND STILL
  if(ch2Val >= (mid-dz) && ch2Val <= (mid+dz)){
    standStill();
    return;
  }

  // ✅ FORWARD
  if(ch2Val > mid + dz){
    for(int k=0;k<=N;k++){
      mpuUpdate();
      poseForward((float)k/N);
      delay(dt);
    }
    return;
  }

  // ✅ BACKWARD
  if(ch2Val < mid - dz){
    for(int k=0;k<=N;k++){
      mpuUpdate();
      poseBackward((float)k/N);
      delay(dt);
    }
    return;
  }
}

