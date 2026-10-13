#include <Servo.h>

/*
 * Camera servo + two digital sensors.
 *
 * This sketch is meant to be MERGED into your existing rover Arduino firmware
 * that already controls motors/gripper. It intentionally does not replace
 * your motor code.
 *
 * Raspberry Pi -> Arduino command token for camera:
 *   sp01.00,   => move in positive direction
 *   sn01.00,   => move in negative direction
 *   sp00.00,   => stop
 *
 * Arduino -> Raspberry Pi sensor tokens:
 *   p1, => proximity active
 *   p0, => proximity inactive
 *   m1, => metal detector active
 *   m0, => metal detector inactive
 */

#include <Servo.h>

// ====== CHANGE THESE PINS TO YOUR ACTUAL WIRING ======
const int CAMERA_SERVO_PIN = 9;
const int PROXIMITY_PIN = 7;
const int METAL_DETECTOR_PIN = 8;

// Set to true if your sensor gives LOW when it detects an object/metal.
const bool PROXIMITY_ACTIVE_LOW = false;
const bool METAL_ACTIVE_LOW = false;

// ====== CAMERA LIMITS ======
const int CAMERA_START_ANGLE = 90;
const int CAMERA_MIN_ANGLE = 20;
const int CAMERA_MAX_ANGLE = 160;

// One step every this many milliseconds.
// Increase this number to make the camera slower.
const unsigned long SERVO_STEP_INTERVAL_MS = 30;

// If no camera command arrives for this long, stop the servo movement.
const unsigned long CAMERA_COMMAND_TIMEOUT_MS = 250;

Servo cameraServo;

int cameraAngle = CAMERA_START_ANGLE;
int cameraDirection = 0;  // +1, -1, 0
unsigned long lastServoStep = 0;
unsigned long lastCameraCommand = 0;
unsigned long lastSensorPublish = 0;

bool readProximity()
{
  const bool raw = digitalRead(PROXIMITY_PIN);
  return PROXIMITY_ACTIVE_LOW ? !raw : raw;
}

bool readMetal()
{
  const bool raw = digitalRead(METAL_DETECTOR_PIN);
  return METAL_ACTIVE_LOW ? !raw : raw;
}

void processCameraToken(const String &token)
{
  // Expected:
  // sp01.00, sn01.00, sp00.00
  if (token.length() < 2 || token.charAt(0) != 's')
  {
    return;
  }

  const char sign = token.charAt(1);
  float value = 0.0f;

  if (token.length() > 2)
  {
    value = token.substring(2).toFloat();
  }

  lastCameraCommand = millis();

  if (value < 0.01f)
  {
    cameraDirection = 0;
    return;
  }

  if (sign == 'p')
  {
    cameraDirection = +1;
  }
  else if (sign == 'n')
  {
    cameraDirection = -1;
  }
}

void readSerialTokens()
{
  static String token = "";

  while (Serial.available() > 0)
  {
    const char c = static_cast<char>(Serial.read());

    if (c == ',')
    {
      if (token.length() > 0)
      {
        processCameraToken(token);
        token = "";
      }
    }
    else if (c != '\r' && c != '\n')
    {
      token += c;

      // Safety against corrupted serial data.
      if (token.length() > 24)
      {
        token = "";
      }
    }
  }
}

void updateCameraServo()
{
  if (millis() - lastCameraCommand > CAMERA_COMMAND_TIMEOUT_MS)
  {
    cameraDirection = 0;
  }

  if (cameraDirection == 0)
  {
    return;
  }

  if (millis() - lastServoStep < SERVO_STEP_INTERVAL_MS)
  {
    return;
  }

  lastServoStep = millis();

  cameraAngle += cameraDirection;
  cameraAngle = constrain(
    cameraAngle,
    CAMERA_MIN_ANGLE,
    CAMERA_MAX_ANGLE);

  cameraServo.write(cameraAngle);

  if (cameraAngle == CAMERA_MIN_ANGLE || cameraAngle == CAMERA_MAX_ANGLE)
  {
    cameraDirection = 0;
  }
}

void publishSensors()
{
  // Publish at 10 Hz. The rover ROS2 hardware interface converts these
  // tokens to /proximity and /metal_detector Bool topics.
  if (millis() - lastSensorPublish < 100)
  {
    return;
  }

  lastSensorPublish = millis();

  Serial.print("p");
  Serial.print(readProximity() ? '1' : '0');
  Serial.print(",m");
  Serial.print(readMetal() ? '1' : '0');
  Serial.print(",\n");
}

void setup()
{
  Serial.begin(115200);

  pinMode(PROXIMITY_PIN, INPUT);
  pinMode(METAL_DETECTOR_PIN, INPUT);

  cameraServo.attach(CAMERA_SERVO_PIN);
  cameraServo.write(cameraAngle);

  lastCameraCommand = millis();

  Serial.println("CAMERA_SERVO_READY");
}

void loop()
{
  readSerialTokens();
  updateCameraServo();
  publishSensors();

  // Keep your existing motor/gripper code here.
}