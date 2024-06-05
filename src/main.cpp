#include <Arduino.h>
#include "pinout.h"
#include "SimpleFOC.h"

HFIBLDCMotor motor = HFIBLDCMotor(3,0.7,35.0f,4e-3f);

#if defined(STM32F4xx) || defined(STM32F7xx)
BLDCDriver3PWM driver = BLDCDriver3PWM(MOT1_A, MOT1_B, MOT1_C, MOT1_EN);
LowsideCurrentSense currentsense = LowsideCurrentSense(0.01f, 50.0f, CS_A, CS_B);
#endif

#ifdef ARDUINO_B_G431B_ESC1
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentsense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
#endif

#ifdef ESP32
#include <soc/adc_periph.h>
#include <driver/adc.h>
BLDCDriver3PWM driver = BLDCDriver3PWM(26, 27, 33, 12); // c-> b1
LowsideCurrentSense currentsense = LowsideCurrentSense(0.01f, 50.0f, 35, 34);
#endif

Commander command = Commander(Serial);
void onMotor(char* cmd){command.motor(&motor,cmd);}

void process_hfi(){motor.process_hfi();}



float target_vel_ramp = 0;
float target_velocity = 0;
float rampRate=5;
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}
void setup() {
  #ifdef ESP32
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  pinMode(2,OUTPUT);
  #else
  pinMode(PC10,OUTPUT);
  Serial.begin(115200);
	#endif
  SimpleFOCDebug::enable(&Serial);
  
  while (!Serial.available()) {}

  driver.voltage_power_supply = 22;
  driver.pwm_frequency = 20000;
  driver.voltage_limit = driver.voltage_power_supply;
  driver.init();


  command.add('M',&onMotor,"motor");
  command.add('T', doTarget, "target velocity");
  motor.linkDriver(&driver);

  motor.linkCurrentSense(&currentsense);
  currentsense.linkDriver(&driver);
  // don't skip current sense align with sfoc shield
  // currentsense.skip_align = true;
  currentsense.init();
  motor.voltage_sensor_align=0.5;
  motor.current_limit = 3.0f;
  motor.P_angle.P = 3.0f;
  motor.P_angle.I = 1.0f;
  motor.P_angle.D = 0.1f;
  motor.P_angle.output_ramp = 0;
  motor.LPF_angle.Tf =  1/(25*_2PI);

  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 0.5f;
  motor.PID_velocity.D = 0.0005f;
  motor.PID_velocity.output_ramp = 2000;
  motor.LPF_velocity.Tf = 1/(25*_2PI); 
  

  motor.LPF_current_d.Tf = 1/(2000*_2PI);
  motor.LPF_current_q.Tf = 1/(2000*_2PI);
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;

  // 2804 140kv
  // motor.Ld = 2200e-6f;
  // motor.Lq = 3100e-6f;
  
  //5208 80kv
  // motor.Ld = 4000e-6f;
  // motor.Lq = 6500e-6f;

  //3548 790kv
  motor.phase_resistance=0.7;
  motor.Ld = 1.212e-3f;
  motor.Lq = 2.998e-3f;
  motor.error_saturation_limit=0.1;
  motor.hfi_v = 15.0;
  motor.polarity_alignment_voltage=1.0;
  motor.bemf_threshold=3.5;

  motor.init();
  motor.initFOC();


  motor.hfi_on = true;
  motor.sensor_direction = Direction::CCW;
  motor.current_setpoint.d = 0.00f;
  motor.ocp_protection_limit=5.5;

  delay(1000);
  motor.start_polarity_alignment=true;
  delay(1000);
}

uint32_t time_prev=0;
void loop() {
  //uint32_t time_now = micros();
  //if ((time_now-time_prev) > 1000){
  motor.move(target_vel_ramp);
   // time_prev = time_now;
  //}
  if(target_velocity>target_vel_ramp){
    target_vel_ramp+=(rampRate/1000000)*(micros() - time_prev);
  }
  if(target_velocity<target_vel_ramp){
    target_vel_ramp-=(rampRate/1000000)*(micros() - time_prev);
  }
  time_prev=micros();

  motor.loopFOC();
  command.run();

}
