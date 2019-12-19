#include <ECE3.h>

////////////////motor control definitions//////////////////////////
const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int LED_RF = 41;
const int LED_FF = 51;

const int MOTORDEFAULT = 120;
////////////////////////////////////////////////////////////////////

//definition of turning constant
const int TURN = 1200;

//definition of PD constants
const int Kp = 4;
const int Kd = 53;
int prev_error=0; //this is the previous_error

int mergedinput=0; //this is the merged sensorfusion value





uint16_t sensorValues[8];
uint16_t normalValues[8];

void setup()
{
  ECE3_Init();
  //Serial.begin(9600);
  
  ////motor control setup////////
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);
  delay(2000);
}


void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);
  //check if black line is hit to indicate turning
  if (sensorValues[0] >= TURN && sensorValues[1] >= TURN && sensorValues[7] >= TURN && sensorValues[6] >= TURN)
  {
    turn();
    return;
  }
  
  mergedinput = sense();

  int error = mergedinput - 0; //calculate the error with 0 being the no error state
  int control  = Kp*error + Kd*(error-prev_error); //apply the the p control
  prev_error=error;
  
  int correct; //initialize value for correction to apply
  //Serial.print("error: ");
  //Serial.println(control);
  //delay(2000);
  
  ///////////////////checking of direction of control///////////////////////////
  if (control > 0 ) //apply motor correction to right wheel
  {
    correct = control/12; //mapping the control value to range of acceptable values of the motor
    //Serial.print("Right correction:");
    //Serial.println(correct);
    //delay(1000);
    analogWrite(left_pwm_pin, MOTORDEFAULT-correct);
    analogWrite(right_pwm_pin, MOTORDEFAULT+correct); 
  }
  else if (control < 0) //apply motor correction to left wheel
  {
    control = control*(-1); //change the value of control to a positive value
    correct = control/12; //mapping the control value to range of acceptable values of the motor
    //Serial.print("Left correction:");
    //Serial.println(correct);
    //delay(1000);
    analogWrite(left_pwm_pin, MOTORDEFAULT+correct);
    analogWrite(right_pwm_pin, MOTORDEFAULT-correct);
  }
  else if (control == 0) //no error, meaning both motors run at default speed
  {
    analogWrite(left_pwm_pin, 255);
    analogWrite(right_pwm_pin, 255);
  }
  delay(1);
}

int sense()
{
  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  for (unsigned char i = 0; i < 8; i++)
  {
    switch (i) {
      case 0:
        normalValues[i] = map(sensorValues[i], 0, 2500, 0, 150); //267
      case 1:
        normalValues[i] = map(sensorValues[i], 0, 2500, 0, 150); //290
      case 2:
        normalValues[i] = map(sensorValues[i], 0, 2500, 0, 150); //336
      case 3:
        normalValues[i] = map(sensorValues[i], 0, 2200, 0, 150); //290
      case 4:
        normalValues[i] = map(sensorValues[i], 0, 2200, 0, 150); //406
      case 5:
        normalValues[i] = map(sensorValues[i], 0, 2500, 0, 150); //398
      case 6:
        normalValues[i] = map(sensorValues[i], 0, 2500, 0, 150); //464
      case 7:
        normalValues[i] = map(sensorValues[i], 0, 2500, 0, 150); //511
    }  
  }
  ///////sensor fusion////////////////
  int mergedinputresult = normalValues[0]*(-4) + normalValues[1]*(-3) + normalValues[2]*(-2) + normalValues[3]*(-1) + normalValues[4]*1 + normalValues[5]*2 + normalValues[6]*3 + normalValues[7]*4;

  return(mergedinputresult);
  //Serial.print("Error: ");
  //Serial.println(mergedinput);
  //delay(1000);
  //Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  //Serial.println();
}

//function to execute the turn
void turn()
{
  delay(50);
  digitalWrite(right_dir_pin,HIGH);
  for (int i=0; i<750; i++)
  {
    //Serial.println("turn");
    delay(1);
    analogWrite(left_pwm_pin, 80);
    analogWrite(right_pwm_pin, 80);
  }
  digitalWrite(right_dir_pin,LOW);
  for (int q=0; q<300; q++)
  {
    //Serial.println("turn");
    delay(1);
    analogWrite(left_pwm_pin, 40);
    analogWrite(right_pwm_pin, 40);
  }
}
