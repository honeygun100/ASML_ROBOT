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
float offset1 = 0.00056700; // this is to fix the Rz of the coordiante plane, 0.00056700 is old value
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
//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
//uint16_t r, g, b, c, colorTemp, lux;

//lever servo Variables
Servo lever_servo; 
unsigned long release_timer = 0;
unsigned long clamp_timer = 0;
int lever_action_state= 0;




void setup() {
  //pinMode(motor1_pin_servo_lib, OUTPUT);
  //analogWrite(motor1_pin_servo_lib, p_in);

  Serial.begin(9600);
  pinMode(up_an_where_pin_input, INPUT);
  pinMode(up_and_where_pin_output, OUTPUT);
  pinMode(game_start_input_pin, INPUT);
  pinMode(game_start_output_pin, OUTPUT);
  //Servo set up
  lever_servo.attach(11); // 8 prob wont work we need to use 11 
  myservo1.attach(motor1_pin_servo_lib);
  myservo2.attach(motor2_pin_servo_lib);
  myservo3.attach(motor3_pin_servo_lib);
  myservo4.attach(motor4_pin_servo_lib);
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
  




  gyro_update_loop_timer = millis();
  gyro_PID_loop_timer = millis();


  // color sensor 
  pinMode(sensorFL, INPUT);
  pinMode(sensorRB, INPUT);
  // if (tcs.begin()) {
  //   Serial.println("Found sensor");
  // } else {
  //   Serial.println("No TCS34725 found, check connections");
  // while (1);
  // }
  delay(3000);
}











void loop() {
  // call functions to initialize
  //int test_flag = 0;
  int task = 1;
  int home_task = 1;
  //unsigned long step_delay = 1300;
  int print_colors = 0;
  print_gyro_values = 0;


  getColor();
  if(stateFL.blue == 1){
    home = blue;
  }else{
    home = yellow;
  }

  

  BNO005_get_standing_error(); //update the gyro_read_offset with in initial sample of gyro reading
  Serial.println("we are here");
  digitalWrite(game_start_output_pin, HIGH); // WAIT FOR PHYSICAL SWITCH TO FLIP
  while(digitalRead(game_start_input_pin) == LOW){
    ;;
  }
  digitalWrite(game_start_output_pin, LOW);
  Serial.println("also we  are here");

  //DURING THE 5 SECONDS EMILE NEEDS TO HOLD THE STRING
  release();

  
  int start_pos;
  digitalWrite(up_and_where_pin_output, HIGH);
  if(digitalRead(up_an_where_pin_input) == LOW){// FLIP SWITCH ACCORDING TO WHICH DIRECTION WE NEED TO MOVE FIRST.
    start_pos = 0;
    move_direction[4] = '\0';
    move_direction[0] = 'r';
    move_direction[1] = 'i';
    move_direction[2] = 'g';
    move_direction[3] = 't';
  }else{
    start_pos = 1;
    move_direction[4] = '\0';
    move_direction[0] = 'l';
    move_direction[1] = 'e';
    move_direction[2] = 'f';
    move_direction[3] = 't';
  }

  gyro_update_loop_timer = millis();
  gyro_PID_loop_timer = millis();
  //unsigned long delay_timer = millis();
  unsigned long end_game_timer = millis();
  unsigned long color_sampler_timer = millis();
  while(1){
    if(millis() - end_game_timer < 45000){
      // Use Serial to test inputs to motors and speeds/calibrate motors
      while(millis() - color_sampler_timer >= 50){
        getColor();
        color_sampler_timer = millis();
      }
      // getColor();

      if(print_colors == 1){
        Serial.print("FL:   | ");
        Serial.print("Curr: ");
        Serial.print(stateFL.curr);
        Serial.print(" | ");
        Serial.print("Opp: ");
        Serial.print(stateFL.opp);
        Serial.print(" | ");
        Serial.print("Blue: ");
        Serial.print(stateFL.blue);
        Serial.print(" | ");
        Serial.print("Yellow: ");
        Serial.print(stateFL.yellow);
        Serial.print(" | ");
        Serial.print("Black: ");
        Serial.print(stateFL.black);
        Serial.print(" | ");
        Serial.print("Period: ");
        Serial.print(stateFL.period);
        Serial.print(" | ");
        // Serial.print("------- RB:   | ");
        // Serial.print("Curr: ");
        // Serial.print(stateRB.curr);
        // Serial.print(" | ");
        // Serial.print("Opp: ");
        // Serial.print(stateRB.opp);
        // Serial.print(" | ");
        // Serial.print("Blue: ");
        // Serial.print(stateRB.blue);
        // Serial.print(" | ");
        // Serial.print("Yellow: ");
        // Serial.print(stateRB.yellow);
        // Serial.print(" | ");
        // Serial.print("Black: ");
        // Serial.print(stateRB.black);
        // Serial.print(" | ");
        // Serial.print("Period: ");
        // Serial.print(stateRB.period);
        // Serial.print(" | ");
      }

      if(Serial.available()){
        char incomingCharacter = Serial.read();
        if(incomingCharacter == '1'){ // case one and two are for test code at bottom of void loop()
          p_in++;
          micros_p_in++;
          Serial.print(p_in);
          Serial.print("  ");
          Serial.println(micros_p_in);
        }else if(incomingCharacter == '2'){
          p_in--;
          micros_p_in--;
          Serial.print(p_in);
          Serial.print("  ");
          Serial.println(micros_p_in);
        }else if(incomingCharacter == '3'){
          gyro_KP_divider += .001;
          Serial.print("kp_divider is ");
          Serial.print(gyro_KP_divider,5);
          gyro_PID_KP = whlpair1_micro_p_in_max/gyro_KP_divider; // maybe replace micro_p_in_max with 1
          Serial.print("\tkp is ");
          Serial.println(gyro_PID_KP,5);
        }else if(incomingCharacter == '4'){
          gyro_KP_divider -= .001;
          Serial.print("kp_divider is ");
          Serial.print(gyro_KP_divider,5);
          gyro_PID_KP = whlpair1_micro_p_in_max/gyro_KP_divider; // maybe replace micro_p_in_max with 1
          Serial.print("\tkp is ");
          Serial.println(gyro_PID_KP,5);
        }else if(incomingCharacter == '5'){
          gyro_PID_KI += .0001;
          Serial.print("gyro_PID_KI is ");
          Serial.println(gyro_PID_KI,5);
        }else if(incomingCharacter == '6'){
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
          offset1 += .0000010;
          Serial.print("offset1 is ");
          Serial.println(offset1);

        }else if(incomingCharacter == 'x'){
          offset1 -= .0000010;
          Serial.print("offset1 is ");
          Serial.println(offset1);

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


    
      // we are also tracking the move direction of the robot, so when game timer hits, we know the direction
      if(start_pos == 0){ //start position is on the back left of board, home checks are in the tasks
        if(task == 1){
          current_direction = forward;
          choose_direction_and_move();
          
          if(home == yellow){
            if(stateFL.blue == 1 && stateRB.blue == 1){
              control_move_time_delay(250);
              task = 2;
            }
          }else{
            if(stateFL.yellow == 1 && stateRB.yellow == 1){
              control_move_time_delay(250);
              task = 2;
            }
          }
        }

        
        else if(task == 2){
          current_direction = right;
          choose_direction_and_move();
          getColor();
          if(stateRB.black == 1){
            task = 3;
          }
        }

        else if(task == 3){
          current_direction = left;
          choose_direction_and_move();
          
          if(stateRB.black != 1){
            control_move_time_delay(300);
            move_direction[4] = '\0';
            move_direction[0] = 'l';
            move_direction[1] = 'e';
            move_direction[2] = 'f';
            move_direction[3] = 't';
            task = 4;
          }
        }

        else if(task == 4){
          current_direction = backward;
          choose_direction_and_move();
          
          if(home == yellow){
            if(stateFL.yellow == 1 && stateRB.yellow == 1){
              task = 5;
            }
          }else{
            if(stateFL.blue == 1 && stateRB.blue == 1){
              task = 5;
            }
          }
        }

        else if(task == 5){
          current_direction = left;
          choose_direction_and_move();
          getColor();
          if(stateFL.black == 1){
            task = 6;
          }
        }

        else if(task == 6){
          current_direction = right;
          choose_direction_and_move();
          
          if(stateFL.black != 1){
            control_move_time_delay(300);
            move_direction[4] = '\0';
            move_direction[0] = 'r';
            move_direction[1] = 'i';
            move_direction[2] = 'g';
            move_direction[3] = 't';
            task = 7;
          }
        }


        else if(task == 7){
          current_direction = backward;
          choose_direction_and_move();
          
          if(stateRB.black == 1){
            task = 8;
          }
        }

        else if(task == 8){
          current_direction = forward;
          choose_direction_and_move();
          
          if(home == yellow){
            if(stateRB.yellow == 1){
              control_move_time_delay(300);
              stop_all_wheels();
              exit(1);
            }
          }else{
            if(stateRB.blue == 1){
              control_move_time_delay(300);
              stop_all_wheels();
              exit(1);
            }
          }
        }

      }else{ //start position is on the back right of board, home checks are in the tasks : start_pos == 1

        if(task == 1){
          current_direction = forward;
          choose_direction_and_move();
          
          if(home == yellow){
            if(stateFL.blue == 1 && stateRB.blue == 1){
              control_move_time_delay(250);
              task = 2;
            }
          }else{
            if(stateFL.yellow == 1 && stateRB.yellow == 1){
              control_move_time_delay(250);
              task = 2;
            }
          }
        }

        
        else if(task == 2){
          current_direction = left;
          choose_direction_and_move();
          getColor();
          if(stateFL.black == 1){
            task = 3;
          }
        }

        else if(task == 3){
          current_direction = right;
          choose_direction_and_move();
          
          if(stateFL.black != 1){
            control_move_time_delay(300);
            move_direction[4] = '\0';
            move_direction[0] = 'r';
            move_direction[1] = 'i';
            move_direction[2] = 'g';
            move_direction[3] = 't';
            task = 4;
          }
        }

        else if(task == 4){
          current_direction = backward;
          choose_direction_and_move();
          
          if(home == yellow){
            if(stateFL.yellow == 1 && stateRB.yellow == 1){
              task = 5;
            }
          }else{
            if(stateFL.blue == 1 && stateRB.blue == 1){
              task = 5;
            }
          }
        }

        else if(task == 5){
          current_direction = right;
          choose_direction_and_move();
          getColor();
          if(stateRB.black == 1){
            task = 6;
          }
        }

        else if(task == 6){
          current_direction = left;
          choose_direction_and_move();
          
          if(stateRB.black != 1){
            control_move_time_delay(300);
            move_direction[4] = '\0';
            move_direction[0] = 'l';
            move_direction[1] = 'e';
            move_direction[2] = 'f';
            move_direction[3] = 't';
            task = 7;
          }
        }


        else if(task == 7){
          current_direction = backward;
          choose_direction_and_move();
          
          if(stateRB.black == 1){
            task = 8;
          }
        }

        else if(task == 8){
          current_direction = forward;
          choose_direction_and_move();
          
          if(home == yellow){
            if(stateRB.yellow == 1){
              control_move_time_delay(300);
              stop_all_wheels();
              exit(1);
            }
          }else{
            if(stateRB.blue == 1){
              control_move_time_delay(300);
              stop_all_wheels();
              exit(1);
            }
          }
        }
      }
      
    }else{ // When the game ends, get to the Final side of the board
      getColor();
      
      if(move_direction[0] == 'r'){
        if(home_task == 1){
          current_direction = left;
          choose_direction_and_move();
          //getColor();
          if(stateFL.black == 1){
            home_task = 2;
          }     
        }
      

        if(home_task == 2){
          current_direction = right;
          choose_direction_and_move();
          
          if(stateFL.black != 1){
            control_move_time_delay(300);
            home_task = 3;
          }
        }

        if(home_task == 3){
          current_direction = backward;
          choose_direction_and_move();
          
          if(stateRB.black == 1){
            home_task = 4;
          }
        }

        if(home_task == 4){
          current_direction = forward;
          choose_direction_and_move();
          
          if(home == yellow){ // this was home_task before but needs to be home
            if(stateRB.yellow == 1){
              control_move_time_delay(400);
              stop_all_wheels();
              exit(1);
            }
          }else{
            if(stateRB.blue == 1){
              control_move_time_delay(400);
              stop_all_wheels();
              exit(1);
            }
          }
        }
      }

      if(move_direction[0] == 'l'){
        if(home_task == 1){
          current_direction = right;
          choose_direction_and_move();
          //getColor();
          if(stateRB.black == 1){
            home_task = 2;
          }     
        }
      

        if(home_task == 2){
          current_direction = left;
          choose_direction_and_move();
          
          if(stateRB.black != 1){
            control_move_time_delay(300);
            home_task = 3;
          }
        }

        if(home_task == 3){
          current_direction = backward;
          choose_direction_and_move();
          
          if(stateRB.black == 1){
            home_task = 4;
          }
        }

        if(home_task == 4){
          current_direction = forward;
          choose_direction_and_move();
          
          if(home == yellow){  // this was home_task before but needs to be home
            if(stateRB.yellow == 1){
              control_move_time_delay(400);
              stop_all_wheels();
              exit(1);
            }
          }else{
            if(stateRB.blue == 1){
              control_move_time_delay(400);
              stop_all_wheels();
              exit(1);
            }
          }
        }
      }

    }



  
    //motor test code
    //analogWrite(motor1_pin_servo_lib, p_in);
    // myservo1.writeMicroseconds(micros_p_in); // when holding from correct position: left wheel -> 1.628 * 1000 for motor one to move forward coutner-clockwise
    //                                         //                                                   1.373 * 1000 for motor one to move backward clockwise
    // myservo2.writeMicroseconds(micros_p_in); // front wheel
    // myservo3.writeMicroseconds(micros_p_in); // right wheel
    // myservo4.writeMicroseconds(micros_p_in); // back wheel
    //Serial.println("");
  }
}






