
#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68

// Your requested L298N wiring
#define IN1 1
#define IN2 2
#define IN3 3
#define IN4 4

const float TILT_THRESHOLD = 8.0;
const int MOTOR_SPEED_UNUSED = 110; // ENA/ENB jumpers ON = fixed speed

float neutralAngle = 0;
bool sensorOK = false;

bool readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6) != 6) return false;

  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());

  return true;
}

float getTiltAngle(int16_t ay, int16_t az) {
  return atan2((float)ay, (float)az) * 180.0 / PI;
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void bothForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void bothReverse() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();

  Wire.begin();
  Wire.setClock(100000);
  delay(500);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);

  if (Wire.endTransmission() != 0) {
    Serial.println("MPU communication failed!");
    return;
  }

  delay(200);
  Serial.println("Keep MPU steady for calibration...");
  delay(3000);

  float total = 0;
  int samples = 0;

  for (int i = 0; i < 100; i++) {
    int16_t ax, ay, az;

    if (readAccel(ax, ay, az)) {
      total += getTiltAngle(ay, az);
      samples++;
    }
    delay(10);
  }

  if (samples < 80) {
    Serial.println("Sensor read failed!");
    return;
  }

  neutralAngle = total / samples;
  sensorOK = true;

  Serial.print("Neutral angle: ");
  Serial.println(neutralAngle);
  Serial.println("Continuous MPU + motor control started.");
}

void loop() {
  if (!sensorOK) {
    stopMotors();
    delay(100);
    return;
  }

  int16_t ax, ay, az;

  if (!readAccel(ax, ay, az)) {
    stopMotors();
    Serial.println("MPU read error!");
    delay(100);
    return;
  }

  float angle = getTiltAngle(ay, az);
  float tilt = angle - neutralAngle;

  Serial.print("AX: "); Serial.print(ax);
  Serial.print(" AY: "); Serial.print(ay);
  Serial.print(" AZ: "); Serial.print(az);
  Serial.print(" | Angle: "); Serial.print(angle);
  Serial.print(" | Tilt: "); Serial.print(tilt);
  Serial.print(" | Motor: ");

  if (tilt > TILT_THRESHOLD) {
    bothForward();
    Serial.println("FORWARD");
  } else if (tilt < -TILT_THRESHOLD) {
    bothReverse();
    Serial.println("REVERSE");
  } else {
    stopMotors();
    Serial.println("STOP");
  }

  delay(100);
}
