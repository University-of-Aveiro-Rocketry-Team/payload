#include <motor.h>
#include <driver/gpio.h>

#define ENCA 2      // Encoder A (input)
#define ENCB 4      // Encoder B (input)
#define PWM 5       // Motor driver PWM (ESP32 GPIO) (output)
#define IN1 15      // Motor driver IN1 (output)
#define IN2 27      // Motor driver IN2 (output)

volatile int posi = 0;  // Encoder position
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  // TODO: change tx and rx pins!!
  motor_uart_config(9600, GPIO_NUM_15, GPIO_NUM_27);

  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT); //??
  gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);

  attachInterrupt();

  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

  // pinMode(ENCA, INPUT);
  // pinMode(ENCB, INPUT);
  // attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  // pinMode(IN1, OUTPUT);
  // pinMode(IN2, OUTPUT);
  
  // Setup PWM channel
  ledcAttachPin(PWM, 0);  // Attach PWM to GPIO 5 (channel 0)
  ledcSetup(0, 1000, 8);  // Channel 0, 1kHz, 8-bit resolution

  Serial.println("target pos");
}

void motor_uart_config(int baud_rate, int tx_pin, int rx_pin) {
  // Configure UART parameters
  uart_config_t uart_config = {
      .baud_rate = baud_rate,                  // Set baud rate
      .data_bits = UART_DATA_8_BITS,      // 8 data bits
      .parity = UART_PARITY_DISABLE,      // No parity
      .stop_bits = UART_STOP_BITS_1,      // 1 stop bit
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // No flow control
  };
  uart_driver_install(UART_NUM_0, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
  uart_param_config(UART_NUM_0, &uart_config);
  uart_set_pin(UART_NUM_0, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void attachInterrupt() {
  // set rising edge
  gpio_set_intr_type(GPIO_NUM_2, GPIO_INTR_POSEDGE);

  // Initialize the ISR service
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

  // Add the ISR handler
  gpio_isr_handler_add(GPIO_NUM_2, readEncoder, NULL);
}

void loop_motor() {
  int target = 250 * sin(prevT / 1e6);

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;

  int pos = posi;

  int e = pos - target;
  float dedt = (e - eprev) / deltaT;
  eintegral += e * deltaT;

  float u = kp * e + kd * dedt + ki * eintegral;
  float pwr = fabs(u);
  if (pwr > 255) pwr = 255;

  int dir = (u < 0) ? -1 : 1;

  setMotor(dir, pwr, PWM, IN1, IN2);

  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  ledcWrite(0, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  posi += (b > 0) ? 1 : -1;
}

