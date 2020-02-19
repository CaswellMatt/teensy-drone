#include "MotorNoiseTest.h"
#include <arm_math.h>

#define FRONT_LEFT_TEST_INDEX  1
#define FRONT_RIGHT_TEST_INDEX 2
#define BACK_LEFT_TEST_INDEX   3
#define BACK_RIGHT_TEST_INDEX  4
#define ALL_MOTORS_TEST        5

MotorNoiseTest::MotorNoiseTest() {
  m_message = String("Test motor noise");
}

const String backLeftTestString   = "Test back left motor noise";
const String backRightTestString  = "Test back right motor noise";
const String frontLeftTestString  = "Test front left motor noise";
const String frontRightTestString = "Test front right motor noise";
const String AllMotorsTestString  = "Test all motors noise";

void MotorNoiseTest::setup() {

  // MotorControlManager::setup();

  const int singleMotorTestCount = 1;

  Vector (MARG::*sensor)() = &MARG::getRotationalRates;

  // auto backLeftMotorFunctionCall = [this, sensor]() { 
  //   runTestOnMotors(&MotorControlManager::backLeft, singleMotorTestCount, sensor); 
  // };

  // addOption(BACK_LEFT_TEST_INDEX, backLeftMotorFunctionCall, backLeftTestString);

  // auto backRightMotorFunctionCall = [this, sensor]() { 
  //   runTestOnMotors(&MotorControlManager::backRight, singleMotorTestCount, sensor); 
  // };
  // addOption(BACK_RIGHT_TEST_INDEX, backRightMotorFunctionCall, backRightTestString);

  // auto frontLeftMotorFunctionCall = [this, sensor]() { 
  //   runTestOnMotors(&MotorControlManager::frontLeft, singleMotorTestCount, sensor); 
  // };
  // addOption(FRONT_LEFT_TEST_INDEX, frontLeftMotorFunctionCall, frontLeftTestString);

  // auto frontRightMotorFunctionCall = [this, sensor]() { 
  //   runTestOnMotors(&MotorControlManager::frontRight, singleMotorTestCount, sensor); 
  // };
  // addOption(FRONT_RIGHT_TEST_INDEX, frontRightMotorFunctionCall, frontRightTestString);

  // auto runAllMotorsFunctionCall = [this, sensor]() { 
  //   MotorSignalController controller[4] = {
  //     MotorControlManager::frontRight, MotorControlManager::frontLeft, 
  //     MotorControlManager::backRight, MotorControlManager::backLeft
  //   };
  //   runTestOnMotors(controller, 4, sensor);
  // };
  addExit(this);
  addOption(this, &MotorNoiseTest::runAllMotorTest, AllMotorsTestString);
  m_marg = new MARG();

  timer = micros();
}

void MotorNoiseTest::printTitle() {
  DEBUG_SERIAL.println("Motor Noise Test");
}

void MotorNoiseTest::runTestOnMotors(Vector (MARG::*sensor)()) {
  const uint32_t BUFFER_SIZE = 2048;

  const uint32_t fftSize = BUFFER_SIZE/2;
  const uint32_t ifftFlag = 0;
  const uint32_t doBitReverse = 1;

  float32_t bufferX[BUFFER_SIZE];
  float32_t bufferY[BUFFER_SIZE];
  float32_t bufferZ[BUFFER_SIZE];

  uint16_t bufferIndex = 0;

  float32_t direction = 1;
  float32_t increment = 0.001;

  bool exit = false;

  const int signalSetPoint = 250;

  int iterationCount = 0;
  
  for (int i = 0; i < 1000; ++i) {
    while(micros() - timer < LOOPTIME_US);
    timer = micros();
    m_motorController.setSpeed(THROTTLE_MAP_START, THROTTLE_MAP_START, THROTTLE_MAP_START, THROTTLE_MAP_START);
  }

  while(!exit) {
    while(micros() - timer < LOOPTIME_US);
    timer = micros();
    m_marg->read();

    m_motorController.setSpeed(m_speed, m_speed, m_speed, m_speed);
  
    m_marg->read();
    if (m_speed >= signalSetPoint && iterationCount > 6000) {
      // buffer[bufferIndex] = sin(2 * PI * 20 * (bufferIndex/2) / fftSize); // frequence peak at 20hz for checking fft
      bufferX[bufferIndex] = (m_marg->*sensor)().v0;
      bufferY[bufferIndex] = (m_marg->*sensor)().v1;
      bufferZ[bufferIndex] = (m_marg->*sensor)().v2;
      
      bufferX[bufferIndex + 1] = 0;
      bufferY[bufferIndex + 1] = 0;
      bufferZ[bufferIndex + 1] = 0;
      
      bufferIndex+=2;

    } else if (m_speed < signalSetPoint) {
      increment *= 1.0005;
      m_speed += direction*increment;
    }
    
    iterationCount++;

    if (bufferIndex == BUFFER_SIZE) exit = true;

    if (bufferIndex > BUFFER_SIZE) {
      DEBUG_SERIAL.println("lost control of buffer index"); exit = true;
      delay(10000);
    }
  }

  arm_cfft_radix4_instance_f32 fftComplexInstance;
  arm_status returnCode = arm_cfft_radix4_init_f32(&fftComplexInstance, fftSize, ifftFlag, doBitReverse);

  if (returnCode != ARM_MATH_SUCCESS) {
    DEBUG_SERIAL.println("Incorrect fft init");
    return;
  }

  float32_t srcX[fftSize];
  float32_t destX[fftSize];

  float32_t srcY[fftSize];
  float32_t destY[fftSize];

  float32_t srcZ[fftSize];
  float32_t destZ[fftSize];  
  
  auto calculateFFT = [] (float32_t* inputBufferArray, 
                              arm_cfft_radix4_instance_f32& fftComplexInstance,
                              float32_t* srcArray, 
                              float32_t* destArray) {
    arm_copy_f32(inputBufferArray, srcArray, fftSize);

    arm_cfft_radix4_f32(&fftComplexInstance, inputBufferArray);
    arm_cmplx_mag_f32(inputBufferArray, destArray, fftSize);
  };

  calculateFFT(bufferX, fftComplexInstance, srcX, destX);
  calculateFFT(bufferY, fftComplexInstance, srcY, destY);
  calculateFFT(bufferZ, fftComplexInstance, srcZ, destZ);

  DEBUG_SERIAL.print("fftOutputX rawX");
  DEBUG_SERIAL.print(" fftOutputY rawY");
  DEBUG_SERIAL.println(" fftOutputZ rawZ");
  uint32_t halfFFT = static_cast<uint32_t>(fftSize/2);
  for (uint32_t i = 0; i < halfFFT; i++) {   
    int indexForRealValues = 2*i;

    DEBUG_SERIAL.print(destX[i], 5); DEBUG_SERIAL.print(" "); 
    DEBUG_SERIAL.print(srcX[indexForRealValues], 5); DEBUG_SERIAL.print(" "); 
    DEBUG_SERIAL.print(destY[i], 5); DEBUG_SERIAL.print(" "); 
    DEBUG_SERIAL.print(srcY[indexForRealValues], 5); DEBUG_SERIAL.print(" "); 
    DEBUG_SERIAL.print(destZ[i], 5); DEBUG_SERIAL.print(" "); 
    DEBUG_SERIAL.println(srcZ[indexForRealValues], 5);
  }

  m_speed=THROTTLE_MAP_START;

}

void MotorNoiseTest::runAllMotorTest() {
  Vector (MARG::*sensor)() = &MARG::getRotationalRates;
  runTestOnMotors(sensor);
}