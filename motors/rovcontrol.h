// Need the Servo library
#include <Servo.h>
#include <PS2X_lib.h>

#ifndef _ROVCONTROL_H_
#define _ROVCONTROL_H_

#define PS2_DAT        2  //14    
#define PS2_CMD        6  //15
#define PS2_SEL        5  //16
#define PS2_CLK        4  //17

//#define PRESSURES   true
#define PRESSURES   false
//#define RUMBLE      true
#define RUMBLE      false

#define forward_spin 95
#define backward_spin 80

PS2X ps2x;

// This is our motor.
Servo motorR;
Servo motorL;
Servo motorB;
#define vibrate 0

int initial_x = 0;
int initial_y = 0;
int initial_z = 0;
int x_val_diff = 0;
int z_val_diff = 0;

// Set everything up
void setup()
{
  
  // Put the motor to Arduino pin #9
  motorL.attach(8);
  motorR.attach(9);
  motorB.attach(10);

  // Required for I/O from Serial monitor
 
  Serial.begin(9600);
  Serial.println("before delay");
  delay(300);  //added delay to give wireless ps2 module some time to startup, before configuring it
   
  
  //setup pins and settings: GamePad(clock, command, attention, data, PRESSURES?, RUMBLE?) check for error
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PRESSURES, RUMBLE);
  ps2x.read_gamepad(false, vibrate);
  initial_x = ps2x.Analog(PSS_LX);
  initial_y = ps2x.Analog(PSS_LY);
  initial_z = ps2x.Analog(PSS_RY);
}


void loop()
{
       ps2x.read_gamepad(false, vibrate);

       x_current = ps2x.Analog(PSS_LX);
       z_current = ps2x.Analog(PSS_RY);
       
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
          motorR.write(1);
          motorL.write(1);
       }
       if(z_current != initial_z)
       {
        //Need to find the zero point of the bottom motor and just add or subtract based on the analog stick
          z_val_diff = (z_current - initial_z) / 2
          motorB.write();
       }

       delay(100);

//     }
}





#endif _ROVCONTROL_H_