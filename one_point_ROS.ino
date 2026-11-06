#include <AccelStepper.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Point.h>

#define STEP_PIN1 2
#define DIR_PIN1 3
#define STEP_PIN2 4
#define DIR_PIN2 5
#define STEPS_PER_REVOLUTION 1600

const float L1 = 12;
const float L2 = 12;
const float THETA1_MIN = -25.0;
const float THETA1_MAX = 205.0;

float target_x = 0;
float target_y = 0;
bool new_target_received = false;

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);

void arm_callback(const geometry_msgs::Point& point) {
    target_x = point.x;
    target_y = point.y;
    new_target_received = true;
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Point> arm_subscriber("arm_target", arm_callback);

float constrainTheta1(float theta1) {
    if (theta1 < THETA1_MIN) return THETA1_MIN;
    if (theta1 > THETA1_MAX) return THETA1_MAX;
    return theta1;
}

float calculateTheta2(float x, float y, float L1, float L2) {
    float r2 = x*x + y*y;
    float cosTheta2 = (r2 - L1*L1 - L2*L2) / (2*L1*L2);
    if (cosTheta2 > 1) cosTheta2 = 1;
    if (cosTheta2 < -1) cosTheta2 = -1;
    return acos(cosTheta2) * 180.0 / M_PI;
}

float calculateTheta1(float x, float y, float L1, float L2, float theta2) {
    float theta2_rad = theta2 * M_PI / 180.0;
    float k1 = L1 + L2 * cos(theta2_rad);
    float k2 = L2 * sin(theta2_rad);
    float theta1 = atan2(y, x) - atan2(k2, k1);
    return constrainTheta1(theta1 * 180.0 / M_PI);
}

void moveToPosition(float targetX, float targetY) {
    float theta2 = calculateTheta2(targetX, targetY, L1, L2);
    float theta1 = calculateTheta1(targetX, targetY, L1, L2, theta2);

    stepper1.moveTo(- (STEPS_PER_REVOLUTION * theta1) / 360.0);
    stepper2.moveTo(- (STEPS_PER_REVOLUTION * theta2) / 360.0);

    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        nh.spinOnce();
    }
}

void setup() {
    Serial.begin(57600);

    stepper1.setMaxSpeed(1000);
    stepper1.setAcceleration(500);
    stepper2.setMaxSpeed(1000);
    stepper2.setAcceleration(500);
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);

    nh.initNode();
    nh.subscribe(arm_subscriber);
}

void loop() {
    if (new_target_received) {
        moveToPosition(target_x, target_y);
        new_target_received = false;
    }
    nh.spinOnce();
    delay(10);
}
