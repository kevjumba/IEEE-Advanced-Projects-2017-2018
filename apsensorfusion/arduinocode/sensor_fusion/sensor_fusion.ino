#include <Wire.h>
#include <sensor_fusion.h>

#define PWR_MGMT_1 107
#define GYRO_CONFIG 27
#define CONFIG 26
#define INT_STATUS 58

#define PWR_MGMT_WAKE 0b00000000
#define GYRO_FULL_SCALE 0b00011000
#define CONFIG_FULL_BANDWITH 0b00000000

#define ACCEL_X_1 59
#define ACCEL_X_2 60
#define ACCEL_Y_1 61
#define ACCEL_Y_2 62
#define ACCEL_Z_1 63
#define ACCEL_Z_2 64

#define GYRO_X_1 67
#define GYRO_X_2 68
#define GYRO_Y_1 69
#define GYRO_Y_2 70
#define GYRO_Z_1 71
#define GYRO_Z_2 72

#define ACCEL_LSB_SENSITIVITY 16384
#define GYRO_LSB_SENSITIVITY 16.4

uint8_t accel_x_1_buf[1];
uint8_t accel_x_2_buf[1];
uint8_t accel_y_1_buf[1];
uint8_t accel_y_2_buf[1];
uint8_t accel_z_1_buf[1];
uint8_t accel_z_2_buf[1];
uint8_t gyro_x_1_buf[1];
uint8_t gyro_x_2_buf[1];
uint8_t gyro_y_1_buf[1];
uint8_t gyro_y_2_buf[1];
uint8_t gyro_z_1_buf[1];
uint8_t gyro_z_2_buf[1];

//TODO: Should we use double?
float accel_x; // g's
float accel_y;
float accel_z;
float gyro_x; // degrees/sec
float gyro_y;
float gyro_z;

float accel_x_bias = 0.00;
float accel_y_bias = -0.01;
float accel_z_bias = 0.06;
float gyro_x_bias = -1.25;
float gyro_y_bias = 0.57;
float gyro_z_bias = -1.97;

unsigned long lastTime;
vector * rotated_result_ptr;
vector rotatedResult;

float theta = 0;
float angular_vel;

void readAccelerometer(int *buf) {
  
}

void readGyroscope(int *buf){
  
}

bool readReady() {
  uint8_t int_status[1];
  readReg(INT_STATUS, int_status, 1);
  // if the lsb of INT_STATUS has been set, then 
  if(int_status[0] & (0b00000001) == 1){
    return true;
  }
  return false;
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
  rotated_result_ptr = &rotatedResult;
  rotated_result_ptr->x = 0;
  rotated_result_ptr->y = 0; 
  rotated_result_ptr->z = 1;
  uint8_t write_buf[1];
  write_buf[0] = PWR_MGMT_WAKE;
  writeReg(PWR_MGMT_1, write_buf, 1);
  write_buf[0] = GYRO_FULL_SCALE;
  writeReg(GYRO_CONFIG, write_buf, 1);
  write_buf[0] = CONFIG_FULL_BANDWITH;
  writeReg(CONFIG, write_buf, 1);
  //calculateBias();

  //uint8_t read_buf[1];
  //readReg(PWR_MGMT_1, read_buf, 1);
  //Serial.println(read_buf[0], BIN);
  //readReg(GYRO_CONFIG, read_buf, 1);
  //Serial.println(read_buf[0], BIN);
  //readReg(CONFIG, read_buf, 1);
  //Serial.println(read_buf[0], BIN);
  lastTime = millis();
}

void loop() {
    vector gyro_unit;
    vector * gyro_unit_ptr;
    vector acc_unit;
    vector * acc_unit_ptr;
    vector * filtered_v;
    
    acc_unit_ptr = &acc_unit;
    gyro_unit_ptr = &gyro_unit;
    readAccelAndGyro(acc_unit_ptr, gyro_unit_ptr, true);
    
    unsigned long curTime = millis();
    float timePassed = (curTime - lastTime)/1000.0; //time in seconds
    theta = angular_vel*timePassed;
    lastTime = curTime;
    
    quaternion q;
    quaternion * q_ptr;
    q_ptr = &q;
    
    quaternion_create(gyro_unit_ptr, theta, q_ptr);
    quaternion_rotate(rotated_result_ptr, q_ptr, rotated_result_ptr);
    vector cp;
    vector * cp_ptr = &cp;
    cp_ptr->x = rotated_result_ptr->x;
    cp_ptr->y = rotated_result_ptr->y;
    cp_ptr->z = rotated_result_ptr->z;
//    
    filtered_v = filter(0.01, acc_unit_ptr, cp_ptr);
    printData(acc_unit_ptr, filtered_v);
//  Serial.println("Accel: " + String(accel_x) + ", " + String(accel_y) + ", " + String(accel_z));
//  Serial.println("Gyro: " + String(gyro_x) + ", " + String(gyro_y) + ", " + String(gyro_z));
    delay(50);
//  calculateBias();
}
void printData(vector * acc_unit_ptr, vector * filtered_v){
  Serial.print(acc_unit_ptr->x);
  Serial.print(" ");
  Serial.print(acc_unit_ptr->y);
  Serial.print(" ");
  Serial.print(acc_unit_ptr->z);
  Serial.print(" ");
  Serial.print(rotated_result_ptr->x);
  Serial.print(" ");
  Serial.print(rotated_result_ptr->y);
  Serial.print(" ");
  Serial.print(rotated_result_ptr->z);
  Serial.print(" ");
  Serial.print((*filtered_v).x);
  Serial.print(" ");
  Serial.print((*filtered_v).y);
  Serial.print(" ");
  Serial.print((*filtered_v).z);
  Serial.println();
}

bool readAccelAndGyro(vector * acc_unit, vector * gyro_unit, bool correctForBias) {
  if (readReady()) {
//    Serial.print("read ready!");
    readReg(ACCEL_X_1, accel_x_1_buf, 1);
    readReg(ACCEL_X_2, accel_x_2_buf, 1);
    readReg(ACCEL_Y_1, accel_y_1_buf, 1);
    readReg(ACCEL_Y_2, accel_y_2_buf, 1);
    readReg(ACCEL_Z_1, accel_z_1_buf, 1);
    readReg(ACCEL_Z_2, accel_z_2_buf, 1);
    accel_x = (int)((accel_x_1_buf[0] << 8) | accel_x_2_buf[0]) / (float)(ACCEL_LSB_SENSITIVITY);
    accel_y = (int)((accel_y_1_buf[0] << 8) | accel_y_2_buf[0]) / (float)(ACCEL_LSB_SENSITIVITY);
    accel_z = (int)((accel_z_1_buf[0] << 8) | accel_z_2_buf[0]) / (float)(ACCEL_LSB_SENSITIVITY);
    
    readReg(GYRO_X_1, gyro_x_1_buf, 1);
    readReg(GYRO_X_2, gyro_x_2_buf, 1);
    readReg(GYRO_Y_1, gyro_y_1_buf, 1);
    readReg(GYRO_Y_2, gyro_y_2_buf, 1);
    readReg(GYRO_Z_1, gyro_z_1_buf, 1);
    readReg(GYRO_Z_2, gyro_z_2_buf, 1);
    gyro_x = ((int)((gyro_x_1_buf[0] << 8) | gyro_x_2_buf[0]) / (float)(GYRO_LSB_SENSITIVITY));// - gyro_x_bias;
    gyro_y = ((int)((gyro_y_1_buf[0] << 8) | gyro_y_2_buf[0]) / (float)(GYRO_LSB_SENSITIVITY));// - gyro_y_bias;
    gyro_z = ((int)((gyro_z_1_buf[0] << 8) | gyro_z_2_buf[0]) / (float)(GYRO_LSB_SENSITIVITY));// - gyro_z_bias;
    if(correctForBias){
      gyro_x -= gyro_x_bias;
      gyro_y -= gyro_y_bias;
      gyro_z -= gyro_z_bias;
    }
//    Serial.print(gyro_x);
//    Serial.print(" ");
//    Serial.print(gyro_y);
//    Serial.print(" ");
//    Serial.print(gyro_z);
//    Serial.println();
    if (correctForBias) {
      accel_x -= accel_x_bias;
      accel_y -= accel_y_bias;
      accel_z -= accel_z_bias;
    }
//    Serial.println(accel_x);
//    Serial.println(accel_y);
//    Serial.println(accel_z);
//    Serial.println(gyro_x);÷
//    Serial.println(gyro_y);
//    Serial.println(gyro_z);
    acc_unit->x = accel_x;
    acc_unit->y = accel_y;
    acc_unit->z = accel_z;
    vector_normalize(acc_unit, acc_unit);
    gyro_unit->x = gyro_x * (PI/180);
    gyro_unit->y = gyro_y * (PI/180);
    gyro_unit->z = gyro_z * (PI/180);
    angular_vel = vector_normalize(gyro_unit, gyro_unit);
//    Serial.println("GYRO X");
//    Serial.println(gyro_axis_ptr->x);
//    Serial.println("GYRO Y");
//    Serial.println(gyro_axis_ptr->y);
//    Serial.println("GYRO Z");
//    Serial.println(gyro_axis_ptr->z);
    return true;
  }
  return false;
}

void calculateBias() {
  vector gyro_unit;
  vector * gyro_unit_ptr;
  vector acc_unit;
  vector * acc_unit_ptr;  
  acc_unit_ptr = &acc_unit;
  gyro_unit_ptr = &gyro_unit;
  
  Serial.println("Calculating Bias");
  float accel_x_avg = 0;
  float accel_y_avg = 0;
  float accel_z_avg = 0;
  float gyro_x_avg = 0;
  float gyro_y_avg = 0;
  float gyro_z_avg = 0;
  const int numSamples = 100;
  for (int i = 0; i < numSamples; i++) {
    while(!readAccelAndGyro(acc_unit_ptr, gyro_unit_ptr, false)) {
      delay(10);
    }
    Serial.println(numSamples);
    accel_x_avg += accel_x;
    accel_y_avg += accel_y;
    accel_z_avg += accel_z;
    gyro_x_avg += gyro_x;
    gyro_y_avg += gyro_y;
    gyro_z_avg += gyro_z;
    delay(50);
  }
  accel_x_avg /= numSamples;
  accel_y_avg /= numSamples;
  accel_z_avg /= numSamples;
  gyro_x_avg /= numSamples;
  gyro_y_avg /= numSamples;
  gyro_z_avg /= numSamples;
  accel_x_bias = accel_x_avg;
  accel_y_bias = accel_y_avg;
  accel_z_bias = accel_z_avg;
  gyro_x_bias = gyro_x_avg;
  gyro_y_bias = gyro_y_avg;
  gyro_z_bias = gyro_z_avg;
  Serial.println("float accel_x_bias = " + String(accel_x_avg) + ";");
  Serial.println("float accel_y_bias = " + String(accel_y_avg) + ";");
  Serial.println("float accel_z_bias = " + String(accel_z_avg-1) + ";");
  Serial.println("float gyro_x_bias = " + String(gyro_x_avg) + ";");
  Serial.println("float gyro_y_bias = " + String(gyro_y_avg) + ";");
  Serial.println("float gyro_z_bias = " + String(gyro_z_avg) + ";");
}

vector * filter(float a, vector * acc, vector * gyro){
  vector_multiply(acc, a, acc);
  vector_multiply(gyro, (1-a), gyro);
  vector_add(acc, gyro, acc);
  vector result;
  vector * result_ptr = &result;
  vector_normalize(acc, result_ptr);
  return result_ptr;
}

