#include "definitions.hpp"
//THIS COMMIT IS BEGGNINGING OF THE MERGING


//bar 1 interp 1.628 and 1.373
//bar 2 interp 1.628 and 1.373



//with bar 1 interpolation and qw make KD 90.5 and KP divider .185 or maybe .10?
//with bat 1 interpolation and qz try KP divider 1.22 for now, larger gyro_KP_divider makes gyro_PID_KP smaller which makes gyro_PID_OUT smaller, so more angle to slow down

//with bar 2 interpolation and qw make KD 90.5 and KP divider .125?
char *move_direction = (char*)malloc(5*sizeof(char));

//MOTOR Variables
Servo myservo1; // Create Servo object to control the servo
Servo myservo2;
Servo myservo3; 
Servo myservo4;
unsigned long current_time = 0;
int p_in = 255/2;
int micros_p_in = 1.5 * 1000;
int no_move_p_in = 1.5 * 1000; // no movement
float whlpair1_micro_p_in_max = 1.628 * 1000; // to move wheel forward, wheel 1 and 3 1.628
float whlpair2_micro_p_in_max = 1.628 * 1000; // to move wheel backward, wheel 1 and 3 1.628
float whlpair1_micro_p_in_min = 1.373 * 1000; // to move wheel forward, wheel 2 and 4 1.373 
float whlpair2_micro_p_in_min = 1.373 * 1000; // to move wheel backward, wheel 2 and 4 1.373
float offset2 = 0.00;


//GYRO PID LOOP VARIABLES
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_MPU6050 mpu;
unsigned long gyro_update_loop_timer = 0;
unsigned long gyro_PID_loop_timer = 0;
float find_mean = 0.00;
float find_mean_counter = 0.00;
float gyro_read_offset = 0.00006104 + 0.00006103; // make this 0 and take a sample in beggining of loop()

float gyro_degrees = 0.00; // extern     
float gyro_degrees2 = 0.00; // extern
float gyro_PID_error = 0.00; // extern
float gyro_PID_error_prev = 0.00; // extern
float offset1 = 0.000000; // this is to fix the Rz of the coordiante plane 
float gyro_desired = offset1; // extern

float gyro_PID_P = 0.00; // extern
float gyro_PID_I = 0.00; // extern
float gyro_PID_D = 0.00; // extern 

float gyro_KP_divider = .041; // extern .123 at 11.5 volts .138
float gyro_PID_KP = whlpair1_micro_p_in_max/gyro_KP_divider; // extern
float gyro_PID_KI = 0.00040; // extern .00020 at 11.5 volts
float gyro_PID_KD = 112.00; // extern 102.50 at 11.5 volts
float gyro_PID_out = 00.00; // extern
int print_gyro_values;
bool gyro_foward_flag = true; // extern NOT USED

//Logic variables
int end_flag = 0; // low is to end on our side, high is to end on their side
Direciton current_direction;
home_options home;

//COLOR SENSOR
int curr_low_bound = 0;
int curr_high_bound = 0;
int yellowBound[2] = {0, 200};  // 55
int blueBound[2] = {200, 690};   // 450
int blackBound[2] = {690, 1200}; // 800
States stateFL;
States stateRB;


//lever servo Variables
Servo lever_servo; 
unsigned long release_timer = 0;
unsigned long clamp_timer = 0;
int lever_action_state= 0;




void setup() {
  //pinMode(motor1_pin_servo_lib, OUTPUT);
  //analogWrite(motor1_pin_servo_lib, p_in);

  Serial.begin(9600);
  pinMode(game_start_input_pin, INPUT);
  pinMode(game_stat_output_pin, OUTPUT);
  //Servo set up
  lever_servo.attach(11); // 8 prob wont work we need to use 11 
  myservo1.attach(motor1_pin_servo_lib); // pin 3
  myservo2.attach(motor2_pin_servo_lib); // pin 5
  myservo3.attach(motor3_pin_servo_lib); // pin 6
  myservo4.attach(motor4_pin_servo_lib); // pin 10
  myservo1.writeMicroseconds(micros_p_in);
  myservo2.writeMicroseconds(micros_p_in);
  myservo3.writeMicroseconds(micros_p_in);
  myservo4.writeMicroseconds(micros_p_in);
  

  
  // Try to initialize BNO005!
  if(!bno.begin()){
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
  
  delay(1050);
}











void loop() {
  int counter = 0;
  digitalWrite(game_stat_output_pin, HIGH);
  while(digitalRead(game_start_input_pin) == LOW){
    ;;
  }
  Serial.print("nice, we in here");

  while(1){
    // Use Serial to test inputs to motors and speeds/calibrate motors

    if(Serial.available()){
        char incomingCharacter = Serial.read();
        if(incomingCharacter == 'o'){ // case one and two are for test code at bottom of void loop()
          p_in++;
          micros_p_in++;
          Serial.print(p_in);
          Serial.print("  ");
          Serial.println(micros_p_in);
          
        }else if(incomingCharacter == 'p'){
          p_in--;
          micros_p_in--;
          Serial.print(p_in);
          Serial.print("  ");
          Serial.println(micros_p_in);

        }else if(incomingCharacter == '1'){
          gyro_KP_divider += .001;
          Serial.print("kp_divider is ");
          Serial.print(gyro_KP_divider,5);
          gyro_PID_KP = whlpair1_micro_p_in_max/gyro_KP_divider; // maybe replace micro_p_in_max with 1
          Serial.print("\tkp is ");
          Serial.println(gyro_PID_KP,5);

        }else if(incomingCharacter == '2'){
          gyro_KP_divider -= .001;
          Serial.print("kp_divider is ");
          Serial.print(gyro_KP_divider,5);
          gyro_PID_KP = whlpair1_micro_p_in_max/gyro_KP_divider; // maybe replace micro_p_in_max with 1
          Serial.print("\tkp is ");
          Serial.println(gyro_PID_KP,5);

        }else if(incomingCharacter == '4'){
          gyro_PID_KI += .0001;
          Serial.print("gyro_PID_KI is ");
          Serial.println(gyro_PID_KI,5);

        }else if(incomingCharacter == '5'){
          gyro_PID_KI -= .0001;
          Serial.print("gyro_PID_KI is ");
          Serial.println(gyro_PID_KI,5);

        }else if(incomingCharacter == '7'){
          gyro_PID_KD += .5;
          Serial.print("gyro_PID_KD is ");
          Serial.println(gyro_PID_KD);

        }else if(incomingCharacter == '8'){
          gyro_PID_KD -= .5;
          Serial.print("gyro_PID_KD is ");
          Serial.println(gyro_PID_KD);

        }else if(incomingCharacter == 'z'){
          gyro_desired += .00010;
          Serial.print("offset1 is ");
          Serial.println(gyro_desired,8);

        }else if(incomingCharacter == 'x'){
          gyro_desired -= .00010;
          Serial.print("offset1 is ");
          Serial.println(gyro_desired,8);

        }else if(incomingCharacter == 'c'){
          offset2 += .5;
          Serial.print("offset2 is ");
          Serial.println(offset2);

        }else if(incomingCharacter == 'v'){
          offset2 -= .5;
          Serial.print("offset2 is ");
          Serial.println(offset2);

        }
      }	

    //motor test code
    // analogWrite(motor1_pin_servo_lib, p_in);
    myservo1.writeMicroseconds(1500); // when holding from correct position: left wheel -> 1.628 * 1000 for motor one to move forward coutner-clockwise
                                             //                                                   1.373 * 1000 for motor one to move backward clockwise
    // myservo2.writeMicroseconds(micros_p_in); // front wheel
    // myservo3.writeMicroseconds(micros_p_in); // right wheel
    // myservo4.writeMicroseconds(micros_p_in); // back wheel
  }
}


