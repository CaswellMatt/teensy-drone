#include "MotorNoiseTest.h"

#define FRONT_LEFT_TEST_INDEX 1
#define FRONT_RIGHT_TEST_INDEX 2
#define BACK_LEFT_TEST_INDEX 3
#define BACK_RIGHT_TEST_INDEX 4

MotorNoiseTest::MotorNoiseTest() {
  message = String("Test motor noise");
}

const String backLeftTestString = "Test back left motor noise";

void MotorNoiseTest::setup() {

  MotorControlManager::setup();

  auto backLeftMotorFunctionCall = [this]() { runMotorTest(MotorControlManager::backLeft); };
  addOption(BACK_LEFT_TEST_INDEX, backLeftMotorFunctionCall, backLeftTestString);

  auto backRightMotorFunctionCall = [this]() { runMotorTest(MotorControlManager::backRight); };
  addOption(BACK_RIGHT_TEST_INDEX, backRightMotorFunctionCall, backLeftTestString);

  auto frontLeftMotorFunctionCall = [this]() { runMotorTest(MotorControlManager::frontLeft); };
  addOption(FRONT_LEFT_TEST_INDEX, frontLeftMotorFunctionCall, backLeftTestString);

  auto frontRightMotorFunctionCall = [this]() { runMotorTest(MotorControlManager::frontRight); };
  addOption(FRONT_RIGHT_TEST_INDEX, frontRightMotorFunctionCall, backLeftTestString);

  timer = micros();
} 

void MotorNoiseTest::printTitle() {
  Serial.println("Motor Noise Test");
}

void MotorNoiseTest::runMotorTest(MotorSignalController& signalController) {


  float32_t direction = 1;
  float32_t increment = 0.001;
  int iterationCount = 0;
  bool exit = false;
  while(!exit) {
    while(micros() - timer < LOOPTIME_US);
    timer = micros();

    signalController.trigger(backLeftPulse);
    
    Serial.println(backLeftPulse);Serial.print(" ");


    if (iterationCount > 1000) {
      increment*= 1.001;
      Serial.print(direction*increment);Serial.print(" ");
      backLeftPulse+=direction*increment;
      if (backLeftPulse > 240) direction = -1;
      marg.read();
    }

    if (backLeftPulse < 125 || backLeftPulse > 250) exit = true;

    iterationCount++;

  }

  backLeftPulse=throttleMapStart;
}

