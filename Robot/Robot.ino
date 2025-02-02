#include <Servo.h>
#include <NewPing.h>
#include <PID_v1.h>


class Claw {
  private:
    Servo servo;
    const int pin = 7;
    int pos = 0;
  public:

    Claw() {
      servo.attach(pin);  // Set pin mode to output
    };

    void close() {
       servo.write(180);
       delay(500);
    }

    void open() {
      for (pos = 0; pos <= 70; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
};

class ColorDetect {
  private:
   int outPin = 6; 
   int s0 = 2;
   int s1 = 3; 
   int s2 = 4; 
   int s3 = 5;
   int OE = 9;
   int curColor = 0; // 0 is black, 1 is red, 2 is blue and 3 is green


    int redMin = 27; // Red minimum value
    int redMax = 97; // Red maximum value
    int greenMin = 48; // Green minimum value
    int greenMax = 145; // Green maximum value
    int blueMin = 54; // Blue minimum value
    int blueMax = 132; // Blue maximum value

   int redPW = 0;
    int greenPW = 0;
    int bluePW = 0;

    // Variables for final Color values
    int redValue;
    int greenValue;
    int blueValue;


  public : 
    ColorDetect() {
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    pinMode(OE, INPUT);
    pinMode(outPin, INPUT);  //out from sensor becomes input to arduino



    // Variables for Color Pulse Width Measurements
    
    digitalWrite(s0, HIGH);
    digitalWrite(s1, HIGH);
    digitalWrite(OE, LOW);

    
    }

    /* read RGB components */
    int getRedPW() {
      // Set sensor to read Red only
      digitalWrite(s2,LOW);
      digitalWrite(s3,LOW);
      // Define integer to represent Pulse Width
      int PW;
      // Read the output Pulse Width
      PW = pulseIn(outPin, LOW);
      // Return the value
      return PW;
    }

    int getGreenPW() {
      // Set sensor to read Green only
      digitalWrite(s2,HIGH);
      digitalWrite(s3,HIGH);
      // Define integer to represent Pulse Width
      int PW;
      // Read the output Pulse Width
      PW = pulseIn(outPin, LOW);
      // Return the value
      return PW;
    }

    int getBluePW() {
      // Set sensor to read Blue only
      digitalWrite(s2,LOW);
      digitalWrite(s3,HIGH);
      // Define integer to represent Pulse Width
      int PW;
      // Read the output Pulse Width
      PW = pulseIn(outPin, LOW);
      // Return the value
      return PW;
    }

    void calibrate(){
      // Read Red Pulse Width
      redPW = getRedPW();
      // Delay to stabilize sensor
      delay(200);

      // Read Green Pulse Width
      greenPW = getGreenPW();
      // Delay to stabilize sensor
      delay(200);

      // Read Blue Pulse Width
      bluePW = getBluePW();
      // Delay to stabilize sensor
      delay(200);

      // Print output to Serial Monitor
      Serial.print("Red PW = ");
      Serial.print(redPW);
      Serial.print(" - Green PW = ");
      Serial.print(greenPW);
      Serial.print(" - Blue PW = ");
      Serial.println(bluePW);
    }

    int detect(){
      redPW = getRedPW();
      // Map to value from 0-255
      redValue = map(redPW, redMin,redMax,255,0);
      // Delay to stabilize sensor
      delay(200);

      // Read Green value
      greenPW = getGreenPW();
      // Map to value from 0-255
      greenValue = map(greenPW, greenMin,greenMax,255,0);
      // Delay to stabilize sensor
      delay(200);

      // Read Blue value
      bluePW = getBluePW();
      blueValue = map(bluePW, blueMin,blueMax,255,0);
      // Delay to stabilize sensor
      delay(200);

      Serial.print("Red = ");
      Serial.print(redValue);
      Serial.print(" - Green = ");
      Serial.print(greenValue);
      Serial.print(" - Blue = ");
      Serial.println(blueValue);
      int deadband = 20;
      if (abs(redValue-blueValue) <= deadband && abs(redValue-greenValue) <= deadband){
        return 0;
      }else if (redValue > blueValue && redValue > greenValue){
        return 1;
      }
      else if (blueValue > redValue && blueValue > greenValue){
        return 2;
      }
      else if (greenValue > redValue && greenValue > blueValue){
        return 3;
      }
    }

    bool colorChanged(){
      int newColor = detect();
      if (newColor != curColor){
        curColor = newColor;
        return true;
      }
      return false;
    };


};


class Drivetrain {
  private:
    int rightMotorPin = 8;
    int leftMotorPin = 9;
    int in1 = 10; 
    int in2 = 11; 
    int in3 = 13;
    int in4 = 12;
    int circumfirence = 3.14159*6.35; //in cm
    // int Setpoint, Input, Output;
    // double Kp=2, Ki=5, Kd=1;
    // int PIN_INPUT;
    //PID drivePID(Input, Output, Setpoint, Kp, Ki, Kd, DIRECT);
    

  public :
    // Constructor to initialize motor pins
    Drivetrain() {
    // Set motor pins as outputs
    pinMode(rightMotorPin, OUTPUT);
    pinMode(leftMotorPin, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    }
    double getDistance(int time, int speed){
      return speed/255 * 133 * 1/60 *3.14159*6.35 *time *1000;
    };

    void move(int speed) {
      analogWrite(rightMotorPin, speed+50);  // Set motor 1 speed
      analogWrite(leftMotorPin, speed);   // Set motor 2 speed

      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    };

    void moveBack(int speed) {
      analogWrite(rightMotorPin, speed+50);  // Set motor 1 speed
      analogWrite(leftMotorPin, speed);   // Set motor 2 speed

      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    };

    void timedMove(int speed, int time){
      if (speed<0){
        moveBack(speed);
      }else{
        move(speed);
      }
      delay(time);
    };

    // Turn the robot left
    void turn(int Rspeed, int Lspeed) {
      analogWrite(rightMotorPin, Rspeed);  // Set motor 1 speed
      analogWrite(leftMotorPin, Lspeed);   // Set motor 2 speed

      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    };

    void turn90(int direction){
      // positive is right, negative is left scalar value is speed
      int speed = 200;
      int rotateTime = (6.5/4)/(speed/255 * 133 *1/60*6.35);
      //6.5 is robot width
      
      analogWrite(rightMotorPin, speed);  // Set motor 1 speed
      analogWrite(leftMotorPin, speed);   // Set motor 2 speed
      if (direction>0){
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
      }else{
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
      }
      delay(rotateTime);
      
    };

    // Stop the robot
    void stop() {
      analogWrite(rightMotorPin, 0);  // Stop motor 1
      analogWrite(leftMotorPin, 0);   // Stop motor 2

      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    };
};

class LED{
  private:
    int pin = 9;
    int delayT = 100;
  public:
    LED(){
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    };
    void blink(){
      digitalWrite(pin, HIGH);
      delay(delayT);
      digitalWrite(pin, LOW);
      delay(delayT);
    };
};

//initailizing subsystems
Claw claw;
Drivetrain driveTrain;
NewPing sonar(A5, A4, 500);
ColorDetect colorSensor;
LED led;

// commands start here
void chal1(){
  int foundRing = false;
  int speed = 200;
  while (!foundRing){
    driveTrain.turn(-speed, speed);
    foundRing = colorSensor.colorChanged();
  }; // looks for the begining of the ring
  driveTrain.turn90(speed); // turns to face the center of the circle
  int ring = 1;
  while (true){
    driveTrain.move(speed);
    if (colorSensor.colorChanged()){
      ring+=1;
    }
    if (ring == 5){
      break;//keeps driving forward until it reaches the fifth color
    }
  }
  int timer = 0;
  while (true){
    timer+=1;
    driveTrain.move(speed);
    if (colorSensor.colorChanged()){
      break;// keeps going until 
    }
  }
  driveTrain.timedMove(-speed, timer/2); // move it backwards in half the amount of time to hit the center
  
}

void chal2(){
  int maxDistance = 16;
  int speed = 200;
  while (true){
    driveTrain.move(speed);
    int dis = sonar.ping_cm();
    Serial.print(dis);
    if (dis <=maxDistance){
      driveTrain.stop();
      if (colorSensor.detect() == 0){
        driveTrain.stop();
      }else if (colorSensor.detect() == 1){
        driveTrain.turn90(1);
        driveTrain.turn90(1);
      }else if (colorSensor.detect() == 2){
        driveTrain.turn90(-1);
      }else if (colorSensor.detect() == 3){
        driveTrain.turn90(1);
      }
    }
    delay(200);
  }
};


int checkDiff(int pos[], bool x_axis, int color){
  int timer = 0;
  int speed = 200;
  int distance;
  int maxDistance = 16;
  for (int c = 0; c<9; c++){
    if (color == (int)map[pos[0]][c]){
      distance = abs(c-pos[1])*9.1;
      if (x_axis){
        if (c>pos[1]){
          driveTrain.turn90(1);
        }else{
          driveTrain.turn90(-1);
        }
      }else{
        if (c<pos[1]){
          driveTrain.turn90(1);
          driveTrain.turn90(1);
        }
      }
          
      while (true){
        driveTrain.move(speed);
        timer+=1;
        if (colorSensor.colorChanged() || sonar.ping_cm()<=maxDistance){ 
          driveTrain.turn90(1);
          driveTrain.turn90(1);
          return timer;
        }

          if (driveTrain.getDistance(timer, speed)> distance){
            return 0;
          }
      }
    }

    if (color == (int)map[c][pos[1]]){
      distance = abs(c-pos[0])*9.1;
      if (!x_axis){
        if (c>pos[0]){
          driveTrain.turn90(1);
        }else{
          driveTrain.turn90(-1);
        }
      }else{
        if (c<pos[0]){
          driveTrain.turn90(1);
          driveTrain.turn90(1);
        }
      }
      while (true){
        driveTrain.move(speed);
        timer+=1;
        if (colorSensor.colorChanged() || sonar.ping_cm()<=maxDistance){ 
          driveTrain.turn90(1);
          driveTrain.turn90(1);
          return timer;
        }

        if (driveTrain.getDistance(timer, speed)> distance){
          return 0;
        }
      }
    }
  }
  return true;
}
void chal3(){
  int colorLst[] = {1, 2, 3, 2, 3};
  int colorIndex = 0;
  int speed = 200;
  int maxDistance = 30;
  int map[9][9]; 
  int timer = 0;
  int color;
  int pos[]= {0, 0};
  bool x_axis = true;
  
  while (true){
    
    if (colorIndex >= 5){
      break;
    }
    int sensor = sonar.ping_cm();
    if (sensor>maxDistance||sensor ==0){
      driveTrain.move(speed);
    }else{
      driveTrain.turn90(1);
      if (x_axis == true){
        x_axis = false;
      }else{
        x_axis = true;
      }
    }
    timer +=1;
    if (driveTrain.getDistance(timer, speed)> 9.1){
      int color = colorSensor.detect();
      if (color!=0){
        driveTrain.timedMove(speed, checkDiff(pos, x_axis, color));
      }
      
    }
  }
};


void setup() {
  Serial.begin(9600);
  Serial.println("help");
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:
  //claw.open();
  //claw.close();
  //claw.open();
  Serial.print("Color: ");
  Serial.println(colorSensor.detect());
  //colorSensor.calibrate();
  //Serial.write("help");
  //driveTrain.turn90(1);
  //delay(1000);
  //driveTrain.turn90(-1);
  //delay(1000);
  //chal2();
}
