#include "stm32f0xx_conf.h"
#include "stm32f0xx.h"
#include "main.h"

/******************************************************************************/
#define UART_TIMEOUT 10000//20000
#define OPCODE_DRIVER_ENABLE 0x02
#define OPCODE_DRIVER_DISABLE 0x01
#define OPCODE_DRIVER_MODE 0x03

#define HANDCONTROL_FULL 0x00
#define HANDCONTROL_DIR 0x01
#define HANDCONTROL_NONE 0x02

#define LOAD_SCREW 0x00
#define LOAD_LIFT 0x01
#define LOAD_GUN 0x02

/******************************************************************************/
void sys_init(void);
uint8_t _get_net_address(void);
void adc_init(void);
void tim1_pwm_init(void);
void usart_init(void);
void tim2_init(void);
void spi1_init(void);
uint8_t DipSwitchSetSpeed(void);
void transmit_dataM(uint8_t c[3]);
uint8_t receive_data(void);

void modbus_read_command();

void _set_stop_driver();
void _set_start_driver();
void _set_reverse_driver();
void _set_reset_driver();
void _set_frequency(uint8_t);
uint16_t resSetSpeed(void);

uint16_t readReg(uint8_t reg);
void setReg(uint16_t data);

unsigned short crc16modbus_tx(uint16_t *modbus_data, uint16_t length);
unsigned short crc16modbus_rx(uint8_t *modbus_data, uint8_t length);

uint16_t modbus_rx_buf[50], modbus_tx_buf[50];
uint8_t modbus_rx_counter = 0, modbus_tx_counter = 0, modbus_rx_length = 0, modbus_tx_length = 0, modbus_rx_flag = 0;
uint16_t modbus_freq = 0, modbus_freq_rd = 0;
uint16_t driver_frequency = 0;
/******************************************************************************/

/******************************************************************************/
GPIO_InitTypeDef GPIO;
TIM_TimeBaseInitTypeDef TIM;
TIM_OCInitTypeDef TIM_PWM;
NVIC_InitTypeDef NVIC_InitStructure;
ADC_InitTypeDef ADC__InitStructure;
/******************************************************************************/

/******************************************************************************/
uint8_t netAddress = 0;
uint16_t adc_ch9_value = 0;
uint16_t adcArray[5];
uint16_t uart_timeout_cnt = 1001;
uint8_t time_out_flag = 1;
uint16_t regMonitor = 0;
uint32_t delayUsCnt = 0;
uint16_t crc16_monitor = 0;

uint16_t uartRxTimeoutCnt = 0;
uint8_t uartRxCnt = 0, uartRxPacketStart = 0, uartRxPacketReady = 0, uartRxTemp = 0;
uint8_t uartRxBuf[20], uartTxBuf[20];
uint8_t uartTxCnt = 0, uartTxPacketStart = 0, uartTxPacketReady = 0, uartTxTemp = 0;

uint8_t OpCode = 0;
uint8_t optionSpeed = 0, optionHandControl = 0, optionLoad = 0;
uint16_t optionAdcThreshold = 2047;
uint16_t adc_value = 0;
uint16_t soc_array[100];
uint8_t soc_array_head = 0;
#define RS485 0
#define DISCRETE 1
//uint8_t type = RS485;
uint8_t type = DISCRETE;

#define FORWARD 0
#define BACKWARD 1

#define STOPPED 0
#define ACCELERATION 1
#define RUNNING 2
#define BRAKING 3
uint8_t discreteMotorState = STOPPED;
#define MAX_PWM_DUTY_CYCLE 420

#define DRIVER_MODE_MANUAL 0 
#define DRIVER_MODE_MODBUS 1

#define STATE_STOP 0
#define STATE_FORWARD_ROTATE 1
#define STATE REVERSE_ROTATE 2

#define ROTATE_STOPPED 0   
#define ROTATE_STABLE_FORWARD 1
#define ROTATE_STABLE_BACKWARD 2
#define ROTATE_ACCELERATE_FORWARD 3
#define ROTATE_ACCELERATE_BACKWARD 4
#define ROTATE_BRAKING_FORWARD 5
#define ROTATE_BRAKING_BACKWARD 6

struct faultStruct{
  uint8_t overCurrentFault;
  uint8_t overVoltageFaule;
  uint8_t gateFault;
  uint16_t regFault;
  uint8_t shortCircuitFault;
}faultInfo;

struct motorStruct{
  uint16_t freqModbus;//frequency from host
  uint16_t freqManual;//from onboard potentiometr
  uint8_t driverMode;//manual or auto, defines at startup mcu
  float windingAUCurrent;//SOA output
  float windingBVCurrent;//SOB output
  float windingCWCurrent;//SOC output
  float monitoringVoltage;//
  uint16_t rotateCurrentSpeed;//
  uint16_t rotatePreviousSpeed;
  uint16_t rotateDeltaSpeed;//max speed deviation
  uint8_t rotateMode;// accelerate, braking at lower frequency, braking with brake resistor, stopped, running
  uint8_t rotateModePrev;
  uint8_t rotateDirection;//
  uint16_t rotatePWMValue;//pwm output duty cycle
  uint16_t rotateExceptedSpeed;//by compare with rotatePWMValue
  float emfExceedSupplyVoltage;//when braking fast 
  uint8_t lockForChangeAction;//until action in progress
  uint16_t brakePWMValue; 
  float supplyVoltage;
  
}motorControl;

float maxVoltage = 0;
uint8_t freqLock = 0;
uint16_t resPwmCounter = 0;
uint32_t brakingChangeDutyCycleDelay = 0, accelerateChangeDutyCycleDelay = 0, stableRotateDutyCycleDelay = 0;
uint8_t accelarationStatus = 0;
uint8_t brakeResEnable = 0;
uint32_t net_address_update_cycle = 0;
uint32_t finite_state_machine_cycle = 0;
uint8_t hardfault_exception_raised = 0;

uint16_t speedControl = 0;
uint16_t regInfo[7], tAdcValue = 0;//settlingSpeed - max value 3000(equal to rpm)
uint16_t regMonitorRead = 0;
uint16_t butnTimerCnt = 0, butnSpeedPressCnt = 0, butnSpeedPressState = 0, butnSpeedPressPrestate = 0, butnSpeedState;  

uint32_t rpmTimer = 0, rpmCnt = 0, rpmValue = 0;
uint8_t hallAState = 0, hallAPrestate = 0; 
float expFilter_k = 0.5,old_value_float = 0;
uint8_t freqMeasFlag = 0;
uint32_t freqMeasCnt = 0, rpmFromFreq = 0;
float pulseTime = 0;
uint16_t hall_unbounce_timer = 0, hall_A_cnt = 0, actualRPM = 0, hall_A_sampling_timer = 0;
uint16_t settlingSpeed = 0, settlingSpeedPrev = 0;
uint32_t motorStepTimer = 0;
uint16_t rotateSumResult = 0,  rotateSum = 0,  rotateArray[20], rotateHead = 0;
uint16_t rotateArray_down[20], rotateSum_down = 0, rotateHead_down = 0;
uint16_t rpmCnt_down = 0;
uint16_t adc_voltage_monitor_filtered_value = 0;
/******************************************************************************/
void delay_10us(uint32_t delay){
    delayUsCnt = delay;
    while(delayUsCnt != 0)__NOP();
}
void set_speed(void){
  tAdcValue = (uint16_t)((4095 - (0x0fff&adcArray[4]))/10);//
  if(butnSpeedState == 1){
    TIM_PWM.TIM_Pulse = tAdcValue;
    TIM_OC1Init(TIM1, &TIM_PWM);    
  }
  else{
    TIM_PWM.TIM_Pulse = 0;
    TIM_OC1Init(TIM1, &TIM_PWM);    
  }
}

void configDRV8353to1PwmMode(void){  
    setReg(0x1040);
    setReg(0x1844);
    setReg(0x2744);
    setReg(0x2a66);  
  
}
void pwmUpdateValue(uint16_t value){
  TIM_PWM.TIM_Pulse = value;
  TIM_OC1Init(TIM1, &TIM_PWM);    
}
float vMonitoringEvaluate(uint16_t adcValue){
    uint16_t tAdc = adcValue;
    float tVoltage = 0;
    
    tVoltage = (float)((float)(tAdc)/4095)*68;
    return tVoltage;
}
//******************************************************************************
uint16_t adc_exp_filtering(uint16_t data_input){
	float din = (float) data_input;
	float k = 0.05;
	return ((uint16_t)(din*(k)+(float)(adc_voltage_monitor_filtered_value)*(1.0-k)));
}
uint16_t manual_rotate_speed_evaluate(uint16_t data_input){
	uint16_t tDataInput = data_input;
	if(tDataInput > 4050)tDataInput = 4050;
	if(tDataInput < 50)tDataInput = 50;
	uint8_t resultHzValue = (uint8_t)(data_input/82);
	return resultHzValue;
}
//******************************************************************************
uint8_t accelerateFromStoppedToFreq(uint16_t freq, uint8_t maxStep){
    uint8_t accelerateStatus = 0;
    uint16_t tRotateSpeedWanted = 0;
    uint16_t tDeltaEvaluated = 0;
    uint8_t upDownDir = 0;//up
    uint8_t softStep = 0;
    if(maxStep > 10) softStep = (uint8_t)(0.5*maxStep);
    else softStep = 3;
    if(motorControl.driverMode == DRIVER_MODE_MANUAL)tRotateSpeedWanted = motorControl.freqManual; 
    else tRotateSpeedWanted = motorControl.freqModbus;
    if(motorControl.rotateCurrentSpeed >= tRotateSpeedWanted){
        tDeltaEvaluated = motorControl.rotateCurrentSpeed - tRotateSpeedWanted;
        upDownDir = 1;//down
    }
    else{
      tDeltaEvaluated = tRotateSpeedWanted - motorControl.rotateCurrentSpeed;
      upDownDir = 0;//up
    }
     
    if(motorControl.rotateDeltaSpeed > tDeltaEvaluated){
        if(upDownDir == 0){
            if(tDeltaEvaluated > (0.4*motorControl.rotateDeltaSpeed)){
                if(motorControl.rotatePWMValue <= (MAX_PWM_DUTY_CYCLE-maxStep))motorControl.rotatePWMValue += maxStep;                      
            }
            else{
                if(motorControl.rotatePWMValue <= (MAX_PWM_DUTY_CYCLE-softStep))motorControl.rotatePWMValue += softStep;         
            }
                
        }
        else{
            if(tDeltaEvaluated > (0.4*motorControl.rotateDeltaSpeed)){
                if(motorControl.rotatePWMValue >= maxStep)motorControl.rotatePWMValue -= maxStep; 
            }
            else{
                if(motorControl.rotatePWMValue >= softStep)motorControl.rotatePWMValue -= softStep;
                          
            }          
        }
        
        accelerateStatus = 0;
        pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
    }
    else{
        accelerateStatus = 1;
    }
  
    return accelerateStatus;//when speed rich status = 1
}


void accelerateLightFromStopped(uint16_t freq){
    //uint16_t evPwm = motorControl.rotatePWMValue; 
    uint16_t evSpeed = 0;
    if(motorControl.rotateCurrentSpeed == 0){
        motorControl.rotatePWMValue += 50;
        pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
        
        while(motorControl.rotateCurrentSpeed < freq){
            while(accelerateChangeDutyCycleDelay != 0);      
            if((motorControl.rotateCurrentSpeed-evSpeed)>1){
                motorControl.rotatePWMValue += 10;
                pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
                evSpeed = motorControl.rotateCurrentSpeed;
            }
            else{
                motorControl.rotatePWMValue += 2;
                pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle              
            }
            if(motorControl.rotateCurrentSpeed < 5)accelerateChangeDutyCycleDelay = 2000;
            else if((motorControl.rotateCurrentSpeed >= 5)&&(motorControl.rotateCurrentSpeed<12)) accelerateChangeDutyCycleDelay = 3000;
            else accelerateChangeDutyCycleDelay = 5000;
        }
        accelarationStatus = 1;
    }
}

void brakingFromLowerFreq(void){
    //uint16_t evPwm = motorControl.rotatePWMValue; 
    uint16_t evSpeed = motorControl.rotateCurrentSpeed;
    
    if(motorControl.rotatePWMValue>=10)motorControl.rotatePWMValue -= 10;
    pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
    
    while(motorControl.rotateCurrentSpeed != 0){
        while(brakingChangeDutyCycleDelay != 0);
        if(evSpeed > motorControl.rotateCurrentSpeed){
          if(motorControl.rotateCurrentSpeed > 3){
            if(motorControl.rotatePWMValue>=20)motorControl.rotatePWMValue -= 20;
          }
          else{
            if(motorControl.rotatePWMValue>=10)motorControl.rotatePWMValue -= 10;
          }
          pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle   
          evSpeed = motorControl.rotateCurrentSpeed;
          if(motorControl.rotateCurrentSpeed > 3)brakeResEnable = 1; 
        }
        else{
          if(motorControl.rotatePWMValue>=3)motorControl.rotatePWMValue -= 3;
          pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle 
        }
        if(motorControl.rotateCurrentSpeed > 3)brakingChangeDutyCycleDelay = 6000;//200ms
        else brakingChangeDutyCycleDelay = 3000;
    }
    accelarationStatus = 0;
    brakeResEnable = 0;
    motorControl.rotatePWMValue = 0;
    pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle  
}

/******************************************************************************/
void brakingCallback(void){
	if(motorControl.rotateCurrentSpeed != 0){
		if(motorControl.rotateCurrentSpeed > 3)brakeResEnable = 1;
		if(motorControl.rotatePreviousSpeed > motorControl.rotateCurrentSpeed){
			if(motorControl.rotateCurrentSpeed > 7){
				if(motorControl.rotatePWMValue >= 15) motorControl.rotatePWMValue -= 15;
			}
			else{
				if(motorControl.rotatePWMValue >= 25) motorControl.rotatePWMValue -= 25;
				else motorControl.rotatePWMValue = 0;
			}
			pwmUpdateValue(motorControl.rotatePWMValue);
      
		}
		else{
			if(motorControl.rotatePWMValue >= 5) motorControl.rotatePWMValue -= 5;
			else motorControl.rotatePWMValue = 0;
			pwmUpdateValue(motorControl.rotatePWMValue);
		}
		if(motorControl.rotateCurrentSpeed > 7)brakingChangeDutyCycleDelay = 50000;
		else brakingChangeDutyCycleDelay = 25000;
		motorControl.rotatePreviousSpeed = motorControl.rotateCurrentSpeed;
	}
	else{
		accelarationStatus = 0;
		brakeResEnable = 0;
		motorControl.rotatePWMValue = 0;
		pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
	}
}
/******************************************************************************/
void fastDecay(uint8_t currentDirection){//need large current to stop
    uint8_t dir = currentDirection;
    if((motorControl.rotateCurrentSpeed >= 15)&&(motorControl.rotateCurrentSpeed < 30)){
        if(dir == FORWARD)DRIVER_DIR_REVERSE;
        if(dir == BACKWARD)DRIVER_DIR_FORWARD;
        motorControl.brakePWMValue = 240;
        pwmUpdateValue(motorControl.brakePWMValue);
        brakeResEnable = 1;
    }    
    if((motorControl.rotateCurrentSpeed > 3)&&(motorControl.rotateCurrentSpeed < 15)){
        if(dir == FORWARD)DRIVER_DIR_REVERSE;
        if(dir == BACKWARD)DRIVER_DIR_FORWARD;
        motorControl.brakePWMValue = 150;
        pwmUpdateValue(motorControl.brakePWMValue);
        brakeResEnable = 1;
    }
    if((motorControl.rotateCurrentSpeed > 1)&&(motorControl.rotateCurrentSpeed <= 3)){
        if(dir == FORWARD)DRIVER_DIR_REVERSE;
        if(dir == BACKWARD)DRIVER_DIR_FORWARD;
        motorControl.brakePWMValue = 30;
        pwmUpdateValue(motorControl.brakePWMValue);
    }    
    if(motorControl.rotateCurrentSpeed <= 1){
        pwmUpdateValue(0);
        motorControl.rotatePWMValue = 0;
        brakeResEnable = 0;
        if(dir == FORWARD)DRIVER_DIR_FORWARD;
        if(dir == BACKWARD)DRIVER_DIR_REVERSE;        
    }
      
}
void slowDecayOn(void){  //generate back EMF
    if((motorControl.rotateCurrentSpeed != 0)&&(motorControl.rotateCurrentSpeed < 7))BRAKE_ON;
    if(motorControl.rotateCurrentSpeed == 0) BRAKE_OFF;
                          
}
void slowDecayOff(){
    BRAKE_OFF;
}
/******************************************************************************/
void accelerateCallback(void){
	if(motorControl.rotateCurrentSpeed < motorControl.freqManual){
		if(motorControl.rotateCurrentSpeed == 0){
			if(motorControl.rotatePWMValue<50)motorControl.rotatePWMValue += 50;
			else motorControl.rotatePWMValue += 4;
			pwmUpdateValue(motorControl.rotatePWMValue);
		}
		if((motorControl.rotateCurrentSpeed>0)&&(motorControl.rotateCurrentSpeed <= 10)){
			if(motorControl.rotatePreviousSpeed >= motorControl.rotateCurrentSpeed){
				if((motorControl.rotatePreviousSpeed - motorControl.rotateCurrentSpeed)>1){
					if(motorControl.rotatePWMValue <= 480)motorControl.rotatePWMValue += 10;
					pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
				}
				else{
					if(motorControl.rotatePWMValue <= 480)motorControl.rotatePWMValue += 4;
					pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
				}
			}
		}
		if((motorControl.rotateCurrentSpeed>10)&&(motorControl.rotateCurrentSpeed <= 20)){
			if(motorControl.rotatePreviousSpeed >= motorControl.rotateCurrentSpeed){
				if((motorControl.rotatePreviousSpeed - motorControl.rotateCurrentSpeed)>1){
					if(motorControl.rotatePWMValue <= 480)motorControl.rotatePWMValue += 7;
					pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
				}
				else{
					if(motorControl.rotatePWMValue <= 480)motorControl.rotatePWMValue += 4;
					pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
				}
			}
		}
		if((motorControl.rotateCurrentSpeed>20)&&(motorControl.rotateCurrentSpeed <= 30)){
			if(motorControl.rotatePreviousSpeed >= motorControl.rotateCurrentSpeed){
				if((motorControl.rotatePreviousSpeed - motorControl.rotateCurrentSpeed)>1){
					if(motorControl.rotatePWMValue <= 480)motorControl.rotatePWMValue += 5;
					pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
				}
				else{
					if(motorControl.rotatePWMValue <= 480)motorControl.rotatePWMValue += 2;
					pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
				}
			}
		}
		if((motorControl.rotateCurrentSpeed>30)){
			if(motorControl.rotatePreviousSpeed >= motorControl.rotateCurrentSpeed){
				if((motorControl.rotatePreviousSpeed - motorControl.rotateCurrentSpeed)>1){
					if(motorControl.rotatePWMValue <= 480)motorControl.rotatePWMValue += 5;
					pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
				}
				else{
					if(motorControl.rotatePWMValue <= 480)motorControl.rotatePWMValue += 2;
					pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
				}
			}
		}
		if(motorControl.rotateCurrentSpeed < 5)accelerateChangeDutyCycleDelay = 18000;
		else if((motorControl.rotateCurrentSpeed >= 5)&&(motorControl.rotateCurrentSpeed<12)) accelerateChangeDutyCycleDelay = 20000;
		else accelerateChangeDutyCycleDelay = 30000;
		motorControl.rotatePreviousSpeed = motorControl.rotateCurrentSpeed;
	}
	else{//frequency reached
		accelarationStatus = 1;
	}
}
/******************************************************************************/
uint8_t brakingResistor(uint16_t value){//reserved param
    uint8_t actualState = 1;
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    /*
    if(resPwmCounter < 80){
        GPIO_SetBits(GPIOB, GPIO_Pin_10);
        actualState = 1;
    }
    if(resPwmCounter >= 80){
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);//change to defines
        actualState = 0;
    }*/
    return actualState;
}
uint8_t runMotorStable(uint16_t freq, uint8_t maxDeviation){
	if(motorControl.rotateCurrentSpeed < motorControl.freqManual){
		//if((motorControl.freqManual - motorControl.rotateCurrentSpeed) > 4);
		if((4>(motorControl.freqManual - motorControl.rotateCurrentSpeed))&&((motorControl.freqManual - motorControl.rotateCurrentSpeed) > 2)) motorControl.rotatePWMValue += 2;
		if((0 < (motorControl.freqManual - motorControl.rotateCurrentSpeed))&&((motorControl.freqManual - motorControl.rotateCurrentSpeed) <= 2)) motorControl.rotatePWMValue += 1;
	}
	else{//motorControl.rotateCurrentSpeed >= motorControl.freqManual
		//if((motorControl.rotateCurrentSpeed - motorControl.freqManual) > 4);
		if((4>(motorControl.rotateCurrentSpeed - motorControl.freqManual))&&((motorControl.rotateCurrentSpeed - motorControl.freqManual) > 2)) if(motorControl.rotatePWMValue>=2)motorControl.rotatePWMValue -= 2;
		if((0 < (motorControl.rotateCurrentSpeed - motorControl.freqManual))&&((motorControl.rotateCurrentSpeed - motorControl.freqManual) <= 2)) if(motorControl.rotatePWMValue>=1)motorControl.rotatePWMValue -= 1;
	}
	pwmUpdateValue(motorControl.rotatePWMValue);//update duty cycle
	return motorControl.rotatePWMValue;
}

//******************************************************************************
//******************************************************************************
uint8_t runMotor(uint16_t param){
	if(rpmValue < settlingSpeed){
		if((settlingSpeed - rpmValue) > 7);
		if((7>(settlingSpeed - rpmValue))&&((settlingSpeed - rpmValue) > 5)) tAdcValue += 5;
		if((1 < (settlingSpeed - rpmValue))&&((settlingSpeed - rpmValue) < 5)) tAdcValue += 1;
    
	}
	else{//rpmValue >= settlingSpeed
		if((rpmValue - settlingSpeed) > 7);
		if((7>(rpmValue - settlingSpeed))&&((rpmValue - settlingSpeed) > 5)) if(tAdcValue>=5)tAdcValue -= 5;
		if((1 < (rpmValue - settlingSpeed))&&((rpmValue - settlingSpeed) < 5)) if(tAdcValue>=2)tAdcValue -= 1;
        
	}
	TIM_PWM.TIM_Pulse = tAdcValue;
	TIM_OC1Init(TIM1, &TIM_PWM);
  
	return tAdcValue;
}

uint8_t stateMonitor(){//actual state monitoring (direction, stop) 
	uint8_t state = 0;

	if(motorControl.rotateMode == ROTATE_STOPPED){
		if(motorControl.rotateCurrentSpeed == 0);
		else {motorControl.rotatePWMValue = 0;pwmUpdateValue(motorControl.rotatePWMValue);}
	}
	if(motorControl.rotateMode == ROTATE_STABLE_FORWARD){
		if(motorControl.rotateModePrev == ROTATE_STABLE_FORWARD){
			runMotorStable(motorControl.freqManual, 1);
		}
		if(motorControl.rotateModePrev == ROTATE_ACCELERATE_FORWARD){
			if(motorControl.rotateCurrentSpeed >= motorControl.freqManual){
				motorControl.rotateModePrev = ROTATE_STABLE_FORWARD;
			}
			else{
				//accelerate until done
			}
      
		}
	}
	if(motorControl.rotateMode == ROTATE_STABLE_BACKWARD){
		if(motorControl.rotateModePrev == ROTATE_STABLE_BACKWARD){
			runMotorStable(motorControl.freqManual, 1);
		}
	}
	if(motorControl.rotateMode == ROTATE_ACCELERATE_FORWARD){
    
	}
	if(motorControl.rotateMode == ROTATE_ACCELERATE_BACKWARD){
    
	}
	if(motorControl.rotateMode == ROTATE_BRAKING_FORWARD){
    
	}
	if(motorControl.rotateMode == ROTATE_BRAKING_BACKWARD){
    
	}
	return state;
}


uint8_t controlBEMF(float valueEMF){
  uint8_t result = 0;//no emi
  
  if(vMonitoringEvaluate(adcArray[4]) > 55.0)result = 1;
  else result = 0;
  
  return result;
}

uint8_t faultDetect(){
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == RESET) return 1;
    else return 0;
}
void resetFaultSpi(){
    setReg(0x1041);
}

void send_reset_pulse(void){


}

void rotate_stable(void){
	if(motorControl.rotateCurrentSpeed == motorControl.freqModbus){

	}
	else if(motorControl.rotateCurrentSpeed > motorControl.freqModbus){
		if((motorControl.rotateCurrentSpeed - motorControl.freqModbus)>4) if(motorControl.rotatePWMValue >= 7)motorControl.rotatePWMValue -= 7;
		if(((motorControl.rotateCurrentSpeed - motorControl.freqModbus) > 2)&&((motorControl.rotateCurrentSpeed - motorControl.freqModbus)<=4)){
			if(motorControl.rotatePWMValue>=2) motorControl.rotatePWMValue -= 2;
		}
		if((motorControl.rotateCurrentSpeed - motorControl.freqModbus) <= 2)if(motorControl.rotatePWMValue>=1) motorControl.rotatePWMValue -= 1;
		pwmUpdateValue(motorControl.rotatePWMValue);
	}
	else{
		if((motorControl.freqModbus - motorControl.rotateCurrentSpeed) > 4) if(motorControl.rotatePWMValue < 470) motorControl.rotatePWMValue += 7;
		if(((motorControl.freqModbus - motorControl.rotateCurrentSpeed) > 2)&&((motorControl.freqModbus - motorControl.rotateCurrentSpeed)<=4)){
			if(motorControl.rotatePWMValue < 470) motorControl.rotatePWMValue += 2;
		}
		if((motorControl.freqModbus - motorControl.rotateCurrentSpeed) <= 2)if(motorControl.rotatePWMValue < 470) motorControl.rotatePWMValue += 1;
		pwmUpdateValue(motorControl.rotatePWMValue);
	}
	stableRotateDutyCycleDelay = 20000;//0.2sec update time

}
void rotate_acceleration_forward(){
	if(accelerateChangeDutyCycleDelay == 0)accelerateCallback();
}
void rotate_deceleration_forward(){
	if(brakingChangeDutyCycleDelay == 0)brakingCallback();
}
void rotate_acceleration_reverse(){
	if(accelerateChangeDutyCycleDelay == 0)accelerateCallback();
}
void rotate_deceleration_reverse(){
	if(brakingChangeDutyCycleDelay == 0)brakingCallback();
}
void rotate_stable_forward(){
	if(stableRotateDutyCycleDelay == 0)rotate_stable();
}
void rotate_stable_reverse(){
	if(stableRotateDutyCycleDelay == 0)rotate_stable();
}


void finite_state_machine(){
	if((motorControl.rotateMode == 0x00)&&(motorControl.rotateModePrev == 0x00)){
		motorControl.rotatePWMValue = 0;//reset pwm signal value to driver input

	}
	if((motorControl.rotateMode == 0x00)&&(motorControl.rotateModePrev == 0x01)){//braking procedure, (forward direction))
		rotate_deceleration_forward();
		if(motorControl.rotateCurrentSpeed == 0x00){
			motorControl.rotateModePrev = 0x00;
			motorControl.rotatePWMValue = 0;
			pwmUpdateValue(motorControl.rotatePWMValue);
		}
	}
	if((motorControl.rotateMode == 0x00)&&(motorControl.rotateModePrev == 0x02)){//braking procedure, (reverse direction)
		rotate_deceleration_reverse();
		if(motorControl.rotateCurrentSpeed == 0x00){
			motorControl.rotateModePrev = 0x00;
			motorControl.rotatePWMValue = 0;
			pwmUpdateValue(motorControl.rotatePWMValue);
		}
	}
	if((motorControl.rotateMode == 0x01)&&(motorControl.rotateModePrev == 0x00)){//start from idle state, forward direction
		if(motorControl.rotateCurrentSpeed == 0x00){
			DRIVER_DIR_FORWARD;
			motorControl.rotateDirection = 0;
		}
		rotate_acceleration_forward();
	}
	if((motorControl.rotateMode == 0x02)&&(motorControl.rotateModePrev == 0x00)){//start from idle state, reverse direction
		if(motorControl.rotateCurrentSpeed == 0x00){
			DRIVER_DIR_REVERSE;
			motorControl.rotateDirection = 1;
		}
		rotate_acceleration_reverse();
	}
	if((motorControl.rotateMode == 0x01)&&(motorControl.rotateModePrev == 0x02)){//change rotate direction from reverse to forward
		if(motorControl.rotateCurrentSpeed == 0x00){
			DRIVER_DIR_FORWARD;
			motorControl.rotateDirection = 0;
			motorControl.rotateModePrev = 0x01;
		}
		else{
			rotate_deceleration_reverse();
		}
	}
	if((motorControl.rotateMode == 0x02)&&(motorControl.rotateModePrev == 0x01)){//change rotate direction from forward to reverse
		if(motorControl.rotateCurrentSpeed == 0x00){
			DRIVER_DIR_REVERSE;
			motorControl.rotateDirection = 1;
			motorControl.rotateModePrev = 0x02;
		}
		else{
			rotate_deceleration_forward();
		}
	}
	if((motorControl.rotateMode == 0x01)&&(motorControl.rotateModePrev = 0x01)){//stable rotate, forward direction
		rotate_stable_forward();
	}
	if((motorControl.rotateMode == 0x02)&&(motorControl.rotateModePrev == 0x02)){//stable rotate, reverse direction
		rotate_stable_reverse();
	}

}

void set_default_motorControl(void){
	motorControl.driverMode = DRIVER_MODE_MODBUS;
	motorControl.rotateDirection = 0;
	motorControl.freqManual = 5;
	motorControl.rotatePWMValue = 0;
	motorControl.emfExceedSupplyVoltage = 55;
	motorControl.freqModbus = 5;
	motorControl.rotateDeltaSpeed = 2;
	motorControl.rotateMode = STATE_STOP;
	motorControl.rotateModePrev = STATE_STOP;
	motorControl.supplyVoltage = vMonitoringEvaluate(adcArray[0]);
}

/******************************************************************************/
int main()
{
	sys_init();//base peripheral initialization
	adc_init();//control current through motor +++
	usart_init();//rs485 uart2
	spi1_init();//driver settings
	tim2_init();//interrupt timer
	tim1_pwm_init();//control speed of motor rotation

	delay_10us(50000);//0.5sec

	set_default_motorControl();//default values for struct

	SPI1_CS_HIGH;
	DRIVER_ENABLE;//power up driver mcu
	delay_10us(1000);//10ms wakeup delay

	setReg(0x1040);//0x1040
	setReg(0x1833);
	setReg(0x2733);
	setReg(0x2a66);
	BRAKE_OFF;

	TIM_CtrlPWMOutputs(TIM1, ENABLE);//enable output for pwm
            
/******************************************************************************/
	while(1){
		if(net_address_update_cycle == 0){
			net_address_update_cycle = 50000;
			netAddress = _get_net_address();
		}

		if(finite_state_machine_cycle == 0){
			finite_state_machine_cycle = 10000;//100ms
			finite_state_machine();
		}
		if(uartRxPacketReady == 1){
			modbus_read_command();
			uartRxPacketReady=0;
		}

		/*
		if(netAddress == 0x00){
			BRAKE_OFF;
			tAdcValue = 0;
			TIM_PWM.TIM_Pulse = 0;
			TIM_OC1Init(TIM1, &TIM_PWM);
		}//BRAKE_ON;
		else{
			BRAKE_OFF;
			if(regInfo[2] != 64){
				delay_10us(10000);
				regInfo[2] = readReg(0x02);
			}
			if(regInfo[2] == 64){
				if(adcArray[4] > 3000){
					brakeResEnable = 0;
					if((motorControl.rotateCurrentSpeed >= motorControl.freqManual)||(freqLock == 1)){
						if(motorStepTimer == 0){
							runMotorStable(motorControl.freqManual, 1);
							motorStepTimer = 20000;
						}
						freqLock = 1;
					}
					else {
						if(accelerateChangeDutyCycleDelay == 0)accelerateCallback();
					}
				}
				if(adcArray[4] < 3000){
					if(motorControl.rotateCurrentSpeed > 0){
						//fastDecay(0);
						//slowDecayOn();
						if(motorControl.monitoringVoltage < (motorControl.supplyVoltage+2.0)){
							if(brakingChangeDutyCycleDelay == 0)brakingCallback();
						}
						freqLock = 0;
					}
					else{
						brakeResEnable = 0;
					}
				}
			}//reg2 == 64
		}//netAddress


*/
  
    /*
    if(type==DISCRETE)
    {
        //DipSwitchSetSpeed();
      speedControl =  resSetSpeed();
        TIM_PWM.TIM_Pulse = speedControl;//480;
        TIM_OC1Init(TIM1, &TIM_PWM);       
        //���������� ���������� //discrete control
        if((CCW_STATUS == 1) && (CW_STATUS == 1)){
          _set_stop_driver();
            }//standby

        if((CW_STATUS == 0) && (CCW_STATUS == 1)) { if(fwd_counter<100) fwd_counter++; } else fwd_counter=0;
        if(fwd_counter>=50)
                _set_start_driver();
            //backward
        
        if((CW_STATUS == 1) && (CCW_STATUS == 0)) { if(bkwd_counter<100) bkwd_counter++; } else bkwd_counter=0;
        if(bkwd_counter>=50)
              _set_reverse_driver();
            //forward
       
        if((CW_STATUS == 0) && (CCW_STATUS == 0)){
          BRAKE_ON;
          led_status = LED__STOP;
        }
        //--���������� ���������� //discrete control
    }
    else
      //RS485
    {

    }
    //--
 
    */
    
  }//infinite loop
}//main() 
/******************************************************************************/

/******************************************************************************/
void sys_init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//PWM for driver up to 200kHz
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
      
    //CW && CCW pinout  CW - 12  CWW - 11
    GPIO.GPIO_Mode = GPIO_Mode_IN;
    GPIO.GPIO_OType = GPIO_OType_PP;
    GPIO.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO);
      
    //net address of device PC13 - 1 PC14 - 2 PC15 - 3
    GPIO.GPIO_Mode = GPIO_Mode_IN;
    GPIO.GPIO_OType = GPIO_OType_PP;
    GPIO.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO);
      
    //Driver control pins //PIN__7 - ENABLE_DRIVER || PIN__15 - BRAKE_DRIVER || PIN__10 - DIRECTION
    GPIO.GPIO_Mode = GPIO_Mode_OUT;
    GPIO.GPIO_OType = GPIO_OType_PP;
    GPIO.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_15;
    GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO);
    GPIO.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO);
      
    //Driver Fault output (OD) must be pulled up
    GPIO.GPIO_Mode = GPIO_Mode_IN;
    GPIO.GPIO_OType = GPIO_OType_PP;
    GPIO.GPIO_Pin = GPIO_Pin_8;
    GPIO.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO);

    //Driver current measurement pin (from asc711 hall current sensor)
    GPIO.GPIO_Mode = GPIO_Mode_AN;
    GPIO.GPIO_OType = GPIO_OType_PP;
    //PA0: VinMonitor PA4: SOC PA5: SOB PA6:SOA PA7: SpeedChange
    GPIO.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO);

    //speedChangeButton
    //change to braking Resistor
    GPIO.GPIO_Mode = GPIO_Mode_OUT;
    GPIO.GPIO_OType = GPIO_OType_PP;
    GPIO.GPIO_Pin = GPIO_Pin_10;//status LED
    GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO);

    //HALL SENSORS
    GPIO.GPIO_Mode = GPIO_Mode_IN;
    GPIO.GPIO_OType = GPIO_OType_PP;
    GPIO.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;//HALL_C HALL_B HALL_A
    GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO);
}
/******************************************************************************/


/******************************************************************************/
void tim2_init(void){  
	NVIC_InitTypeDef NVIC_Init__Structure;
    NVIC_Init__Structure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_Init__Structure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init__Structure.NVIC_IRQChannelPriority = 1;//1;
    NVIC_Init(&NVIC_Init__Structure);

    TIM.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM.TIM_CounterMode = TIM_CounterMode_Up;
    TIM.TIM_Period = 9;//10kHz
    TIM.TIM_Prescaler = 47;//1MHz out clock
    TIM.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM);

    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}
/******************************************************************************/

/******************************************************************************/
void tim1_pwm_init(void){
	GPIO.GPIO_Pin = GPIO_Pin_8;
    GPIO.GPIO_Mode = GPIO_Mode_AF;
    GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO.GPIO_OType = GPIO_OType_PP;
    GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO);
      
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);//TIM1 CH1

    //TIM1 48MHz
    TIM.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM.TIM_CounterMode = TIM_CounterMode_Up;
    TIM.TIM_Period = 479;//479;//50khz
    TIM.TIM_Prescaler = 0;//48MHz out clock
    TIM.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM);

    TIM_PWM.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_PWM.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_PWM.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_PWM.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_PWM.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_PWM.TIM_OutputState = TIM_OutputState_Enable;
    TIM_PWM.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_PWM.TIM_Pulse = 0;
    TIM_OC1Init(TIM1, &TIM_PWM);

    TIM_Cmd(TIM1, ENABLE);//Enable timer
    //TIM_CtrlPWMOutputs(TIM17, ENABLE);//Enable pwm_output
 }
/******************************************************************************/
void spi1_init(){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    GPIO.GPIO_Pin = GPIO_Pin_4;
    GPIO.GPIO_Mode = GPIO_Mode_AF;
    GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO.GPIO_OType = GPIO_OType_PP;
    GPIO.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO);

    GPIO.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
    GPIO.GPIO_Mode = GPIO_Mode_AF;
    GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO.GPIO_OType = GPIO_OType_PP;
    GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_0);//SPI1 SCK
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_0);//SPI1 MISO
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_0);//SPI1 MOSI

    SPI_InitTypeDef SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 0x00;
    SPI_Init(SPI1, &SPI_InitStructure);
    SPI_Cmd(SPI1, ENABLE);
    
    //nChipSelect
    GPIO.GPIO_Mode = GPIO_Mode_OUT;
    GPIO.GPIO_OType = GPIO_OType_PP;
    GPIO.GPIO_Pin = GPIO_Pin_6;
    GPIO.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO);
}
/******************************************************************************/
void writeCMD(uint16_t cmd, uint16_t data){  
	SPI_I2S_SendData16(SPI1, cmd);
	//while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)== RESET);
	//while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_I2S_SendData16(SPI1, data);
}
uint16_t readData(uint16_t cmd){
	SPI1_CS_LOW;
	uint16_t tempData = 0;
	SPI_I2S_SendData16(SPI1, cmd);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)== RESET);
	tempData = SPI_I2S_ReceiveData16(SPI1);
	SPI1_CS_HIGH;
	return tempData;
}
/******************************************************************************/
uint8_t _get_net_address(void){
	uint8_t NetAdress = 0;
	NetAdress = (NET_ADRESS_BIT_0) | (NET_ADRESS_BIT_1 << 1) | (NET_ADRESS_BIT_2 << 2);//2;
    return NetAdress;
}
void setReg(uint16_t data){
	SPI1_CS_LOW;
	for(uint16_t dI = 0; dI < 7000; dI++)__NOP();
	//uint16_t tempData = (0x8000)|(reg<<11)|(0x7ff&data);
	//regMonitor = tempData;
	SPI_I2S_SendData16(SPI1, data);
	for(uint32_t dI = 0; dI < 100000; dI++)__NOP();
	//while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)== RESET)__NOP();
	regMonitorRead = SPI_I2S_ReceiveData16(SPI1);
	SPI1_CS_HIGH;
	for(uint16_t dI = 0; dI < 7000; dI++)__NOP();
}
uint16_t readReg(uint8_t reg){
	SPI1_CS_LOW;
	for(uint16_t dI = 0; dI < 7000; dI++)__NOP();
	uint16_t tempData = (0x8000)|(reg<<11);
	regMonitor = tempData;
	SPI_I2S_SendData16(SPI1, tempData);
	for(uint32_t dI = 0; dI < 100000; dI++)__NOP();
	//while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)== RESET)__NOP();
	regMonitorRead = SPI_I2S_ReceiveData16(SPI1);
	SPI1_CS_HIGH;
	for(uint16_t dI = 0; dI < 7000; dI++)__NOP();
  	return (regMonitorRead);
}
/******************************************************************************/
uint16_t spiCustomRead(uint8_t reg){
	SPI1_CS_LOW;
	//delay_ms(1);
  	SPI_SCLK_LOW;
  	return 0;
}
uint16_t expFilter(uint16_t new_value, uint16_t old_value){
	old_value_float = expFilter_k*((float)new_value) + (1-expFilter_k)*((float)old_value_float);
	uint16_t result = (uint16_t) old_value_float;
	return result;
}
uint16_t evaluateFreq(uint16_t input_cnt, uint8_t pol){
	uint16_t t_period = input_cnt*2;
	pulseTime = ((float) t_period)*10e-6;
	float t_freq = 1/pulseTime;
	return ((uint16_t)t_freq);
}

/******************************************************************************/
void adc_init(void){
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
    DMA_InitStructure.DMA_BufferSize = 5;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &adcArray;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, ENABLE);  
  
    RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC__InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC__InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC__InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC__InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC__InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
    ADC_Init(ADC1, &ADC__InitStructure);
     
    ADC_ChannelConfig(ADC1, ADC_Channel_0 , ADC_SampleTime_55_5Cycles);//Vin monitor
    ADC_ChannelConfig(ADC1, ADC_Channel_4 , ADC_SampleTime_55_5Cycles);//SOC
    ADC_ChannelConfig(ADC1, ADC_Channel_5 , ADC_SampleTime_55_5Cycles);//SOB
    ADC_ChannelConfig(ADC1, ADC_Channel_6 , ADC_SampleTime_55_5Cycles);//SOA
    ADC_ChannelConfig(ADC1, ADC_Channel_7 , ADC_SampleTime_55_5Cycles);//Speed regulation potentiometr

    ADC_GetCalibrationFactor(ADC1);
    ADC_Cmd(ADC1, ENABLE);
    //
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));
    ADC_StartOfConversion(ADC1);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);

}
/******************************************************************************/

/******************************************************************************/
void usart_init(void){
    USART_InitTypeDef USART_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);          
    GPIO.GPIO_Mode = GPIO_Mode_AF;
    GPIO.GPIO_OType = GPIO_OType_PP;
    GPIO.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO);    

    GPIO.GPIO_Mode = GPIO_Mode_OUT;
    GPIO.GPIO_OType = GPIO_OType_PP;
    GPIO.GPIO_Pin = GPIO_Pin_1;//DE for RS485
    GPIO.GPIO_Speed = GPIO_Speed_Level_2;
    GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO);
  
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;//1;
    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate = 38400;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);
    
    USART_Cmd(USART2, ENABLE);
    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART2, USART_IT_TC, ENABLE);
    
}
/******************************************************************************/

/******************************************************************************/
void USART2_IRQHandler(void){
	//RX interrupt
    if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET){
        uart_timeout_cnt = 0;
        modbus_rx_buf[uartRxCnt++] =  USART_ReceiveData(USART2);
        if(uartRxCnt == 2){
            if(modbus_rx_buf[1] == 0x10)modbus_rx_length = 13;//write block cmd
            else modbus_rx_length = 8;//write/read 1 reg
        }
        if((uartRxCnt > 2)&&(uartRxCnt == modbus_rx_length)){
            uartRxCnt = 0;
            uartRxPacketReady = 1;
        }
        if((uartRxCnt > 2)&&(uartRxCnt > modbus_rx_length)){//something gone wrong, clear counter
            uartRxCnt = 0;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE); 
    }
    //--RX interrupt
  
    //--TX interrupt
    if(USART_GetITStatus(USART2, USART_IT_TC)  == SET){   
        uartTxCnt++;
        if(uartTxCnt<modbus_tx_length){
          USART_SendData(USART2, modbus_tx_buf[uartTxCnt]);
        }
        else{
          GPIO_ResetBits(GPIOA, GPIO_Pin_1);//rx mode on
          USART_ITConfig(USART2, USART_IT_TC, DISABLE);
        }
        USART_ClearITPendingBit(USART2, USART_IT_TC);
    } 
}//USART2_IRWHandler()
/******************************************************************************/
void _send_error_code(uint8_t cmd, uint8_t error_code){
    uint16_t temp_crc_value = 0;
    modbus_tx_buf[0] = (uint8_t) netAddress;
    modbus_tx_buf[1] = (uint8_t) (0x80+(cmd&0x0f));
    modbus_tx_buf[2] = (uint8_t) error_code;
    temp_crc_value = crc16modbus_tx(modbus_tx_buf, 3);
    modbus_tx_buf[3] = (uint8_t)(temp_crc_value&0xff);//lo byte
    modbus_tx_buf[4] = (uint8_t)(temp_crc_value>>8);//hi byte
    modbus_tx_length = 5;
    //uartTxCnt=0;
    //GPIO_SetBits(GPIOA, GPIO_Pin_1); //DE to tx
    //USART_ITConfig(USART2, USART_IT_TC, ENABLE);
    //USART_SendData(USART2, modbus_tx_buf[uartTxCnt]);
}

/******************************************************************************/
void TIM2_IRQHandler(void){ 
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET){//10uSec timer update event period
		if(uart_timeout_cnt < UART_TIMEOUT) uart_timeout_cnt++;
		if(uart_timeout_cnt == UART_TIMEOUT){
			uart_timeout_cnt++;
			uartRxCnt = 0;
		}
		if(stableRotateDutyCycleDelay != 0)stableRotateDutyCycleDelay--;
		if(finite_state_machine_cycle != 0)finite_state_machine_cycle--;
		if(net_address_update_cycle != 0)net_address_update_cycle--;
		if(timeout != 0) timeout--;
		if(delayUsCnt != 0)delayUsCnt--;
		if(motorStepTimer != 0)motorStepTimer--;//change pwm pulse width with period
              
		butnTimerCnt++;
		if(SpeedEnableButtonState == SET)butnSpeedPressCnt++;
		if(butnTimerCnt >= 10000){
			if(butnSpeedPressCnt > 7000) butnSpeedState = 1;
			else butnSpeedState = 0;
			butnSpeedPressCnt = 0;
			butnTimerCnt = 0;
		}
    
		adc_voltage_monitor_filtered_value = adc_exp_filtering(adcArray[0]);
		motorControl.monitoringVoltage = vMonitoringEvaluate(adc_voltage_monitor_filtered_value);
		if(motorControl.monitoringVoltage > maxVoltage) maxVoltage = motorControl.monitoringVoltage;

		hall_unbounce_timer++;
		if(HALL_A_PIN)hall_A_cnt++;
		if(hall_unbounce_timer >= 50){//500uSec
			hall_unbounce_timer = 0;
			if(hall_A_cnt >= 30){
				hallAState = 1;
				if((hallAState == 1)&&(hallAPrestate == 0)){
					rpmCnt++;
					//freqMeasFlag = 1;
				}

			}
			else{
				hallAState = 0;
				if((hallAState == 0)&&(hallAPrestate == 1)) {
					rpmCnt_down++;
					if(freqMeasFlag == 1){
						//rpmFromFreq = evaluateFreq(freqMeasCnt, 3);
					}
					freqMeasFlag = 0;
					freqMeasCnt = 0;
				}
			}
			hall_A_cnt = 0;
			hallAPrestate = hallAState;
			hall_A_sampling_timer++;
			if(hall_A_sampling_timer >= 20){
				hall_A_sampling_timer = 0;
				//actualRPM = expFilter(rpmCnt, actualRPM);
				rotateArray_down[rotateHead_down++] = rpmCnt_down;
				rotateArray[rotateHead++] = rpmCnt;
				rpmCnt = 0;
				rpmCnt_down = 0;
				if(rotateHead == 10)rotateHead = 0;
				if(rotateHead_down == 10)rotateHead_down = 0;
				rotateSum = 0;
				rotateSum_down = 0;
				for(uint8_t iR = 0; iR < 10; iR++){
					rotateSum += rotateArray[iR];
					rotateSum_down += rotateArray_down[iR];
				}
				motorControl.rotateCurrentSpeed  = (rotateSum/3);

				soc_array[soc_array_head++] = adcArray[1];
				if(soc_array_head>99)soc_array_head=0;
			}
		}
		//if(freqMeasFlag == 1)freqMeasCnt++;//10uSec
		/*
		rpmTimer++;
		if(rpmTimer >= 1000){
			rpmTimer = 0;
			rotateArray[rotateHead++] = rpmCnt;
			if(rotateHead == 10)rotateHead = 0;
			rotateSum = 0;
			for(uint8_t iR = 0; iR < 10; iR++){
				rotateSum += rotateArray[iR];
			}
			rotateSumResult = (rotateSum/4);
			rpmValue = rotateSumResult;
			motorControl.rotateCurrentSpeed = rotateSumResult;
			rpmCnt = 0;
		}*/


		resPwmCounter++;
		if(resPwmCounter >= 100)resPwmCounter = 0;
		if(brakingChangeDutyCycleDelay != 0)brakingChangeDutyCycleDelay--;
		if(accelerateChangeDutyCycleDelay != 0)accelerateChangeDutyCycleDelay--;

		if(brakeResEnable == 1){
			brakingResistor(0);
		}
		else{
			GPIO_ResetBits(GPIOB, GPIO_Pin_10);
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}//TIM_IT_UPDATE flag set
}//TIM2_IRQHandler
/******************************************************************************/

/******************************************************************************/
void _set_stop_driver(void)
{
	motorControl.rotateModePrev = motorControl.rotateMode;
	motorControl.rotateMode = 0x00;//Stop
}
/******************************************************************************/

/******************************************************************************/
void _set_start_driver(void)
{
	motorControl.rotateModePrev = motorControl.rotateMode;
	motorControl.rotateMode = 0x01;//Start forward mode
	motorControl.rotateDirection = 0;//dir - forward
}
/******************************************************************************/

/******************************************************************************/
void _set_reverse_driver(void)
{
	motorControl.rotateModePrev = motorControl.rotateMode;
	motorControl.rotateMode = 0x02;//Start reverse mode
	motorControl.rotateDirection = 1;
}
/******************************************************************************/

/******************************************************************************/
void _set_reset_driver(void)
{
	motorControl.rotateModePrev = motorControl.rotateMode;
	motorControl.rotateMode = 0x00;//Stop mode
	motorControl.freqModbus = 10;//10Hz
	motorControl.rotateDirection = 0x00;//dir - forward
	motorControl.freqManual = 0x10;//10Hz
}
/******************************************************************************/

/******************************************************************************/
void _set_frequency(uint8_t freq){
    uint8_t t_freq = freq;
    if(t_freq <= 50) motorControl.freqModbus = freq;
    else motorControl.freqModbus = 20;
}
/******************************************************************************/

/******************************************************************************/
unsigned short crc16modbus_tx(uint16_t *modbus_data, uint16_t length){
    unsigned short crc16=0xffff;
    unsigned char byte=0, bit=0, c=0;

    for (byte=0;byte<length;byte++){
        crc16^= modbus_data[byte];//*(add + byte); //tx_buf[byte];
        for (bit=0;bit<8;bit++){
            if ((crc16&0x0001)==0x0001) c = 1; else c = 0;    
            crc16=(crc16>>1)&0x7fff;
            if (c==1) crc16=crc16^0xa001;
        }
    }
    return crc16;
}
/******************************************************************************/

/******************************************************************************/
unsigned short crc16modbus_rx(uint8_t *modbus_data, uint8_t length){
    unsigned short crc16_rx=0xffff;
    unsigned char byte_rx=0, bit_rx=0, c_rx=0;

    for (byte_rx=0;byte_rx<length;byte_rx++)
    {
        crc16_rx^= modbus_data[byte_rx];//*(add + byte); //tx_buf[byte];
        for (bit_rx=0;bit_rx<8;bit_rx++)
        {
            if ((crc16_rx&0x0001)==0x0001) c_rx = 1; else c_rx = 0;
                
            crc16_rx=(crc16_rx>>1)&0x7fff;
            if (c_rx==1) crc16_rx=crc16_rx^0xa001;
        }
    }

    return crc16_rx;
}
/******************************************************************************/


/******************************************************************************/
void modbus_read_command(){
  if(modbus_rx_buf[0] == _get_net_address()){
      if(modbus_rx_buf[1] == 0x03){// read reg cmd
    	  modbus_tx_length = 7;

    	  if((modbus_rx_buf[2] == 0xfd) && (modbus_rx_buf[3] == 0x00)){
          modbus_tx_buf[0] = netAddress;
          modbus_tx_buf[1] = 0x03;//read cmd
          modbus_tx_buf[2] = 0x02;//bytes count
          modbus_tx_buf[3] = (uint8_t)(motorControl.freqModbus>>8);//
          modbus_tx_buf[4] = (uint8_t)motorControl.freqModbus;//lo byte
          uint16_t temp_crc16 = crc16modbus_tx(modbus_tx_buf, 5);
          modbus_tx_buf[5] = (uint8_t)temp_crc16;
          modbus_tx_buf[6] = (uint8_t) (temp_crc16>>8);
        
          uartTxCnt=0;
          GPIO_SetBits(GPIOA, GPIO_Pin_1); //DE to tx
          USART_ITConfig(USART2, USART_IT_TC, ENABLE);
          USART_SendData(USART2, modbus_tx_buf[uartTxCnt]);

      }
    }//read_cmd
    
    if(modbus_rx_buf[1] == 0x06){//write reg cmd
    	modbus_tx_length = 8;
    	if(modbus_rx_buf[2] == 0xfa){
    		if(modbus_rx_buf[3] == 0x00){
    			if((modbus_rx_buf[4] == 0xc0) && (modbus_rx_buf[5] == 0x00)) _set_stop_driver();
    			if((modbus_rx_buf[4] == 0xc4) && (modbus_rx_buf[5] == 0x00)) _set_start_driver();
    			if((modbus_rx_buf[4] == 0xc6) && (modbus_rx_buf[5] == 0x00)) _set_reverse_driver();
    			if((modbus_rx_buf[4] == 0xe0) && (modbus_rx_buf[5] == 0x00)) _set_reset_driver();

    		}
    		if(modbus_rx_buf[3] == 0x01){//set frequency
    			uint16_t temp_freq = ((modbus_rx_buf[4]<<8) | modbus_rx_buf[5])/100;
    			if(temp_freq <= 50)_set_frequency((uint8_t)temp_freq);
    			//if(crc16modbus_tx(modbus_rx_buf, 6) == (modbus_rx_buf[6] | (modbus_rx_buf[7]<<8)))
    			//_set_frequency(temp_freq);
    			crc16_monitor = crc16modbus_tx(modbus_rx_buf, 6);
    		}

    		modbus_tx_buf[0] = modbus_rx_buf[0];
    		modbus_tx_buf[1] = modbus_rx_buf[1];
    		modbus_tx_buf[2] = modbus_rx_buf[2];
    		modbus_tx_buf[3] = modbus_rx_buf[3];
    		modbus_tx_buf[4] = modbus_rx_buf[4];
    		modbus_tx_buf[5] = modbus_rx_buf[5];
    		uint16_t t_crc16 = crc16modbus_tx(modbus_tx_buf, 6);
    		modbus_tx_buf[6] = (uint8_t)(t_crc16);
    		modbus_tx_buf[7] = (uint8_t)(t_crc16>>8);
    	}
    	else{
    		_send_error_code(modbus_rx_buf[4], 0x13);//
    	}

        uartTxCnt=0;
        GPIO_SetBits(GPIOA, GPIO_Pin_1); //DE to tx
        USART_ITConfig(USART2, USART_IT_TC, ENABLE);
        USART_SendData(USART2, modbus_tx_buf[uartTxCnt]);

    }//write_cmd
    if((modbus_rx_buf[1] == 0x10) && (modbus_rx_buf[2] == 0x18)){//write block cmd
    	modbus_tx_length = 8;
        if(((modbus_rx_buf[7] == 0xc4)||(modbus_rx_buf[7] == 0xc6)) && (modbus_rx_buf[8] == 0x00)){
            uint16_t temp_freq = ((modbus_rx_buf[9]<<8) | modbus_rx_buf[10])/100;
            if(temp_freq <= 50)_set_frequency((uint8_t)temp_freq);
            else _set_frequency(25);

            if(modbus_rx_buf[7] == 0xc4)_set_start_driver();
            if(modbus_rx_buf[7] == 0xc6)_set_reverse_driver();
        }
        else{
            //_send_error_code(netAddress, 0x13);
        }
        uartTxCnt=0;
        GPIO_SetBits(GPIOA, GPIO_Pin_1);//DE to tx
        USART_ITConfig(USART2, USART_IT_TC, ENABLE);
        USART_SendData(USART2, modbus_tx_buf[uartTxCnt]);


    }//write_block_cmd
  }//net_address
}//modbus_read_command


uint16_t resSetSpeed(void){
	uint16_t tempResValue = adc_value;
	uint16_t driverPwm = 0;
	if(tempResValue <= 200) driverPwm = 0;
	else{
		driverPwm = (uint16_t)(tempResValue/10);

	}
	return driverPwm;
}

uint8_t DipSwitchSetSpeed(void){
	uint16_t driver_pwm = 0;

	switch(netAddress){
  
		case 0:
		  {
			driver_pwm = 0;
			break;
		  }
		case 1:
		  {
			driver_pwm = 100;
		  }
		case 2:
		  {
			driver_pwm = 200;
			break;
		  }
		case 3:
		  {
			driver_pwm = 250;
			break;
		  }
		case 4:
		  {
			driver_pwm = 300;
			break;
		  }
		case 5:
		  {
			driver_pwm = 350;
			break;
		  }
		case 6:
		  {
			driver_pwm = 400;
			break;
		  }
		case 7:
		  {
			driver_pwm = 420;
			break;
		  }

		default:
		  driver_pwm = 0;
	}
	//TIM1->CCR4 = driver_pwm;
	TIM_PWM.TIM_Pulse = driver_pwm;//480;
	TIM_OC1Init(TIM1, &TIM_PWM);
	return driver_pwm;
}
