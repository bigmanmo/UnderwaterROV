// Need the Servo library
#include <Servo.h>
#include <PS2X_lib.h>

#define PS2_DAT        6  //yellowwhite    
#define PS2_CMD        2  //orange, blue ground, whiteblue power, 
#define PS2_SEL        3  //whitegreen
#define PS2_CLK        4  //green

//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false

#define forward_spin 95
#define backward_spin 80

PS2X ps2x;

// This is our motor.
Servo motorR;
Servo motorL;
Servo motorB;
byte vibrate = 0;

int initial_x = 0;
int initial_y = 0;
int initial_z = 0;
int x_val_diff = 0;
int y_val_diff = 0;
int z_val_diff = 0;
int x_current = 0;
int y_current = 0;
int z_current = 0;

// Set everything up
void setup()
{
  
  // Put the motor to Arduino pin #9
  motorL.attach(8);
  motorR.attach(9);
  motorB.attach(10);

  motorL.write(90);
  motorR.write(90);
  motorB.write(90);

  // Required for I/O from Serial monitor
 
  Serial.begin(9600);
  Serial.println("before delay");
  delay(300);  //added delay to give wireless ps2 module some time to startup, before configuring it
   
  
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  ps2x.read_gamepad(false, vibrate);
  initial_x = ps2x.Analog(PSS_LX);
  Serial.println("initial x");
  Serial.println(initial_x);
  initial_y = ps2x.Analog(PSS_LY);
  Serial.println("initial y");
  Serial.println(initial_y);
  initial_z = ps2x.Analog(PSS_RY);
  Serial.println("initial z");
  Serial.println(initial_z);
}


void loop()
{
       ps2x.read_gamepad(false, vibrate);

       x_current = map(ps2x.Analog(PSS_LX), 0, 255, 0, 180);
       Serial.println("current x");
       Serial.println(x_current);
       y_current = map(ps2x.Analog(PSS_LY), 0, 255, 0, 180);
       Serial.println("current y");
       Serial.println(y_current);
       z_current = map(ps2x.Analog(PSS_RY), 0, 255, 0, 180);
       Serial.println("current z");
       Serial.println(z_current);
       
       if(ps2x.Button(PSB_R1))
       {
        if(x_current > initial_x)
        {
          x_val_diff = (x_current - initial_x) / 5;
          motorL.write(95);
          motorR.write(95 + x_val_diff);
        }
        else if(x_current < initial_x)
        {
          x_val_diff = (initial_x - x_current) / 10;
          motorL.write(95 + x_val_diff);
          motorR.write(95);
        }
        else
        {
          motorR.write(95);
          motorL.write(95);
        }
       }
       else if(ps2x.Button(PSB_L1))
       {
        if(x_current < initial_x)
        {
          x_val_diff = (initial_x - x_current) / 10;
          motorL.write(80);
          motorR.write(80 - x_val_diff);
        }
        else if(x_current > initial_x)
        {
          x_val_diff = (x_current - initial_x) / 10;
          motorL.write(80 - x_val_diff);
          motorR.write(80);
        }
        else
        {
          motorR.write(80);
          motorL.write(80);
        }
       }
       if(ps2x.ButtonReleased(PSB_L1) || ps2x.ButtonReleased(PSB_R1))
       {
        // Need to find zero point of these motors
          motorR.write(90);
          motorL.write(90);
       }
       if(z_current != initial_z)
       {
        //Need to find the zero point of the bottom motor and just add or subtract based on the analog stick
          z_val_diff = (z_current - initial_z) / 5;
          motorB.write(95 + z_val_diff);
          Serial.println("bottom mottor: ");
          Serial.println(95 + z_val_diff);
       }

       delay(1000);

//     }
}

