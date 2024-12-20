#include <motor.h>
#include <driver/gpio.h>

volatile int posi = 0;  // Encoder position
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void motor_setup() {
  gpio_set_direction(ENCA, GPIO_MODE_INPUT);
  gpio_set_direction(ENCB, GPIO_MODE_INPUT);

  attachInterrupt();

  gpio_set_direction(PWM, GPIO_MODE_OUTPUT);
  gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
  gpio_set_direction(IN2, GPIO_MODE_OUTPUT);

  // Setup PWM channel
  setup_motor_pwm(PWM, MOTOR_FREQ, MOTOR_RESOLUTION, MOTOR_CHANNEL);
}

void attachInterrupt() {
  // set rising edge
  gpio_set_intr_type(ENCA, GPIO_INTR_POSEDGE);

  // Initialize the ISR service
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);

  // Add the ISR handler
  gpio_isr_handler_add(ENCA, readEncoder, NULL);
}

void setup_motor_pwm(int PWM_PIN, int PWM_FREQ, ledc_timer_bit_t PWM_RES, ledc_channel_t PWM_CHANNEL) {
  ledc_timer_config_t timer_conf = {
      .duty_resolution = PWM_RES,
      .freq_hz = PWM_FREQ,
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .timer_num = LEDC_TIMER_0,
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t ledc_conf = {
      .channel = PWM_CHANNEL,
      .duty = 0,
      .gpio_num = PWM_PIN,
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .timer_sel = LEDC_TIMER_0,
  };
  ledc_channel_config(&ledc_conf);
}

void ledc_write(float duty_cycle) {
  uint32_t duty = (uint32_t)((1 << MOTOR_RESOLUTION) * duty_cycle); // 1<<resolution = 2^resolution
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CHANNEL, duty);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CHANNEL);
}


void loop_motor() {
  // int target = 250 * sin(prevT / 1e6);

  // // PID constants
  // float kp = 1;
  // float kd = 0.025;
  // float ki = 0.0;

  // long currT = micros();
  // float deltaT = ((float)(currT - prevT)) / 1.0e6;
  // prevT = currT;

  // int pos = posi;

  // int e = pos - target;
  // float dedt = (e - eprev) / deltaT;
  // eintegral += e * deltaT;

  // float u = kp * e + kd * dedt + ki * eintegral;
  // float pwr = fabs(u);
  // if (pwr > 255) pwr = 255;

  // int dir = (u < 0) ? -1 : 1;

  // setMotor(dir, pwr, IN1, IN2);
  setMotor(1, 255, IN1, IN2);

  // eprev = e;
}

void setMotor(int dir, int pwmVal, int in1, int in2) {
  ledc_write(pwmVal);
  if (dir == 1) {
    gpio_set_level(in1, 1);
    gpio_set_level(in2, 0);
  } else {
    gpio_set_level(in1, 0);
    gpio_set_level(in2, 1);
  }
}

void readEncoder() {
  int b = gpio_get_level(ENCB);
  posi += (b > 0) ? 1 : -1;
}

