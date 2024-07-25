#include <micro_ros_arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "ForwardKinematicsDH.h"
#include "Joint.h"
#include "InverseKinematics.h"
#include <Eigen/Dense>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// Declarações de variáveis do Micro-ROS
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13
#define SERVOMIN  150 // Comprimento mínimo do pulso (de um total de 4096)
#define SERVOMAX  600 // Comprimento máximo do pulso (de um total de 4096)

// Definições dos canais dos servos no PCA9685
#define SERVO_1 0
#define SERVO_2 1
#define SERVO_3 2
#define SERVO_4 3
#define SERVO_5 4

// Pinos do motor de passo
#define STEPPER_DIR_PIN 5
#define STEPPER_STEP_PIN 18
#define STEPPER_ENABLE_PIN 19

// Instâncias do driver do PCA9685 e do motor de passo
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

// Macros para verificação de erros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Função para converter ângulo para pulso PWM
uint8_t pulseConverter(uint16_t angle, uint16_t servoMaxPulse, uint16_t servoMinPulse) {
  uint16_t pulse = min(max((angle * (servoMaxPulse - servoMinPulse) / 180.0) + servoMinPulse, servoMinPulse), servoMaxPulse);
  return pulse;
}

// Loop de erro para indicar problemas
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Callback do timer do Micro-ROS
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();

  // Definição dos parâmetros das juntas
  Joint Base(1, 0, 0, -90);
  Joint Shoulder(0, 19.5, 180, 0);
  Joint Elbow(0, 24.3, 25, 0);
  Joint Wrist1(0, 0, 10, 90);
  Joint Wrist2(35, 0, 0, 0);
  std::vector<Joint> joints = {Base, Shoulder, Elbow, Wrist1, Wrist2};
  ForwardKinematicsDH fk(joints);

  // Inicialização do driver PCA9685
  pwm.begin();
  pwm.setPWMFreq(60);  // Define a frequência PWM para 60 Hz

  // Configuração dos pinos do motor de passo
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, LOW);
  
  stepper.setMaxSpeed(1000);  // Define a velocidade máxima
  stepper.setAcceleration(500);  // Define a aceleração

  // Inicialização do Micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_publisher"));

  const unsigned int timer_timeout = 1000;  // Timer a cada 1000 ms
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {
  if (Serial.available() > 0) {
    // Lê a posição desejada do serial
    Serial.println("Type a position in x: ");
    while (Serial.available() == 0) {}
    float x = Serial.parseFloat();
    Serial.println("Type a position in y: ");
    while (Serial.available() == 0) {}
    float y = Serial.parseFloat();
    Serial.println("Type a position in z: ");
    while (Serial.available() == 0) {}
    float z = Serial.parseFloat();
    Eigen::Vector3d desired_position(x, y, z);

    // Atualiza a cinemática direta com a nova posição
    Joint Base(1, 0, 0, -90);
    Joint Shoulder(0, 19.5, 180, 0);
    Joint Elbow(0, 24.3, 25, 0);
    Joint Wrist1(0, 0, 10, 90);
    Joint Wrist2(35, 0, 0, 0);
    std::vector<Joint> joints = {Base, Shoulder, Elbow, Wrist1, Wrist2};
    ForwardKinematicsDH fk(joints);

    Eigen::Matrix4d end_effector = fk.compute_end_effector();
    for (int i = 0; i < end_effector.rows(); ++i) {
      for (int j = 0; j < end_effector.cols(); ++j) {
        end_effector(i, j) = std::round(end_effector(i, j) * 1000) / 1000;
      }
    }

    // Imprime a matriz do efetuador final
    Serial.println("End effector matrix: ");
    for (int i = 0; i < end_effector.rows(); ++i) {
      for (int j = 0; j < end_effector.cols(); ++j) {
        Serial.print(end_effector(i, j));
        Serial.print(" ");
      }
      Serial.println();
    }

    // Calcula a cinemática inversa para obter os ângulos das juntas
    InverseKinematics ik(fk);

    try {
      Eigen::VectorXd joint_angles = ik.compute_ik(desired_position);
      Serial.println("Joint angles: ");
      for (int i = 0; i < joint_angles.size(); ++i) {
        Serial.print(joint_angles[i]);
        Serial.print(" ");
      }
      Serial.println();
      
      // Atualiza os ângulos dos servos
      for (int i = 0; i < joint_angles.size(); ++i) {
        setServoAngle(i, joint_angles[i]);
      }

    } catch (const std::runtime_error& e) {
      Serial.println(e.what());
    }

    ik.error_counter();
  }

  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

// Função para definir o ângulo do servo
void setServoAngle(int servoChannel, double angle) {
  int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoChannel, 0, pulseLength);
}

