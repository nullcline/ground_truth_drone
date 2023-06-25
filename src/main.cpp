#include <Wire.h>
#include <SPI.h>
#include <stdint.h>
#include <LIDARLite.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <utility/imumaths.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  32
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
#define BNO_ADDRESS    0x28

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDRESS, &Wire);
uint8_t system_cal, gyro_cal, accel_cal, mag_cal = 0;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

LIDARLite lidar;
uint16_t cal_cnt = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// sensor data
uint16_t lidar_distance = 0;  // [cm]
int8_t temp = 0;              // [degrees C]
sensors_event_t euler_angles; // [euler angle degrees]
sensors_event_t angular_vel ; // [rad/s] CHECK THIS
sensors_event_t lin_accel;    // [m/s^2] acceleration without gravity CHECK THIS
sensors_event_t mag;          // [microteslas] CHECK THIS
sensors_event_t gravity;      // [m/s^2] CHECK THIS
sensors_event_t pressure;     // [hPa]

const unsigned long delay_interval = 10;
unsigned long last_interval = 0;

String generate_json(String event_type, String axis="", String value="-42069");

String get_event_string(sensors_event_t* event);

void setup() {
  Serial.begin(115200);
  // while ( !Serial ) delay(100);   // uncomment if you want to wait for Serial to be opened

  // bno (imu)
  if (!bno.begin()) {
    Serial.println("no bno055 detected");
    for(;;);
  }

  // bmp (barometer)
  if (!bmp.begin()) {
    Serial.println("no bmp280 detected");
    for(;;);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // operating mode
                  Adafruit_BMP280::SAMPLING_X2,     // temperature oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // filtering
                  Adafruit_BMP280::STANDBY_MS_500); // standby time

  // lidar
  lidar.begin(0, true);
  lidar.configure(3); // 3 = max range mode

  // screen
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("screen failed to init"));
    for(;;);
  }

  display.display();
  delay(500);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println("ROCK AND ROLL");
  delay(1000);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - last_interval >= delay_interval) {
    last_interval = currentMillis;
    String output = "";

    // gather data
    bmp_pressure->getEvent(&pressure);

    bno.getEvent(&euler_angles, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angular_vel, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&lin_accel, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&mag, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&gravity, Adafruit_BNO055::VECTOR_GRAVITY);
    temp = bno.getTemp();

    // take a measurement with receiver bias correction every 100 readings
    if ( cal_cnt == 0 ) {
      lidar_distance = lidar.distance();
    } else {
      lidar_distance = lidar.distance(false);
    }
    cal_cnt = (cal_cnt + 1) % 100;

    // calibration messages
    bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 0);

    if (system_cal == 3) {
      display.println("calibrated!");
      display.println("distance= " + String(lidar_distance) + "cm");
    }
    else {
      display.print("s=" + String(system_cal) + " g=" + String(gyro_cal));
      display.println(" a=" + String(accel_cal) + " m=" + String(mag_cal));
      display.println("distance= " + String(lidar_distance) + "cm");
    }
    display.display();

    // generating output
    output += get_event_string(&euler_angles);
    output += get_event_string(&angular_vel);
    output += get_event_string(&lin_accel);
    output += get_event_string(&mag);
    output += get_event_string(&gravity);
    output += generate_json("temp", "", String(temp));
    output += generate_json("pressure", "", String(pressure.pressure));
    output += generate_json("distance", "", String(lidar_distance));

    Serial.println(output);
  }
}

String generate_json(String event_type, String axis, String value) {
  // populates a string with the json format we read in python. the default -42069 makes it easy to spot errors
  String json = "";
  json += "\"" + event_type + "_" + axis + "\":" + value + ",";
  return json;
}

String get_event_string(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem

  String event_string = "";
  String event_type;

  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    event_type = "accel";
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    event_type = "orient";
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    event_type = "mag";
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    event_type = "gyro";
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    event_type = "rot";
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    event_type = "lin_accel";
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    event_type = "grav";
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
   ;
  }

  event_string += generate_json(event_type, "x", String(x));
  event_string += generate_json(event_type, "y", String(y));
  event_string += generate_json(event_type, "z", String(z));
  return event_string;
}