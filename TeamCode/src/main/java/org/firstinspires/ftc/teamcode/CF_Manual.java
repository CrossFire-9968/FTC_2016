package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/***
 * This file provides basic Telop driving for a robot with mecanum drive
 * The code is structured as an Iterative OpMode
 * <p/>
 * This OpMode uses the CrossFire hardware class to define the devices on the robot.
 * All device access is managed through the Crossfire_Hardware class.
 * <p/>
 * This OpMode takes joystick values from three independent axis and computes a
 * desired motor power for each of the mecanum drive motors (one per wheel) to
 * attain desired velocity, direction, and rotation of robot. If the calculated
 * desired power for any motor exceeds the maximum power limit (1.0F), then all
 * motor powers are proportionally reduced.  We do this to retain consistent
 * driving characteristics at the expense of vehicle speed and total power.
 */

@TeleOp(name = "CF_Manual", group = "Drivetrain")
//Disabled

public class CF_Manual extends OpMode
{
   Crossfire_Hardware robot = new Crossfire_Hardware();
   CF_SensorLibrary sensor = new CF_SensorLibrary();

   // Minimum joystick position before we assume value is good.
   // Near center, value could contain noise or offset that we want to ignore.
   private static final float joystickThreshold = 0.05f;

   // Steering priority gains allow for control effort to
   // emphasis one aspect of steering effort over another.
   // Gain values should be set to a value between 0 and 1;
   // Values greater than 1.0f will increase the likelihood of
   // clipping computed power level.
   private static final float forwardPriority = 1.0f;
   private static final float strafePriority = 1.0f;
   private static final float steerPriority = 1.0f;

   // Beacon button pusher servo increment rate
   private static final double beaconPusherRate = 0.005;

   //CF_SensorLibrary.sensorColor beaconColor;

   // Beacon hue thresholds
   private static final int RedUpperLimit_lowRange = 20;
   private static final int RedLowerLimit_lowRange = 0;
   private static final int RedUpperLimit_highRange = 360;
   private static final int RedLowerLimit_highRange = 325;
   private static final int BlueUpperLimit = 270;
   private static final int BlueLowerLimit = 220;

   public enum sensorColor
   {
      unknown, blue, red
   }

   sensorColor color = sensorColor.unknown;

   // hsvValues is an array that will hold the hue, saturation, and value information.
   float hsvValues[] = {0F, 0F, 0F};

//   private static final int onLine = 2;
//   private static final int offLine = 2;
//   OpticalDistanceSensor odsSensor1;
//   OpticalDistanceSensor odsSensor2;

   // we assume that the LED pin of the RGB sensor is connected to
   // digital port 5 (zero indexed).
   static final int LED_CHANNEL = 5;


   public void init()
   {
      robot.init(hardwareMap);
   }


   public void loop()
   {
      // Calculate and apply motor power to drive wheels
      RunMecanumWheels();

      // Adjust the beacon button servo
      ServiceServos();

      // convert the RGB values to HSV values.
      Color.RGBToHSV((robot.sensorRGB.red() * 255) / 800, (robot.sensorRGB.green() * 255) / 800, (robot.sensorRGB.blue() * 255) / 800, hsvValues);

      // Determine sensor color based on thresholds
      if ((hsvValues[0] >= BlueLowerLimit) && (hsvValues[0] <= BlueUpperLimit))
      {
         color = sensorColor.blue;
      }
      else if ((hsvValues[0] >= RedLowerLimit_highRange) && (hsvValues[0] <= RedUpperLimit_highRange) ||
         (hsvValues[0] >= RedLowerLimit_lowRange) && (hsvValues[0] <= RedUpperLimit_lowRange))
      {
         color = sensorColor.red;
      }

//      telemetry.clear();
//
//      telemetry.addData("Hue: ", hsvValues[0]);
//
//      if (color == sensorColor.blue)
//      {
//         telemetry.addData("Color: ", "blue");
//      }
//      else if (color == sensorColor.red)
//      {
//         telemetry.addData("Color: ", "red");
//      }
//      else
//      {
//         telemetry.addData("Color: ", "unknown");
//      }
//
//      telemetry.update();

      if (gamepad1.right_bumper)
      {
         robot.setBallKickerMode();
      }

      if (gamepad1.left_bumper)
      {
         robot.setBeaconMode();
      }
   }


   /***
    * This method calculates the individual motor powers required to drive teh mecanum
    * wheels based off the driver 1 controller.  This drive strategy uses the following
    * joystick assignments
    * <p/>
    * Left stick:
    * forward (+y)   - Forward drive
    * rearward (-y)  - Reverse drive
    * right (+x)     - Strafe right
    * left (-x)      - Strafe left
    * <p/>
    * Right stick:
    * forward (+y)   - Not used
    * rearward (-y)  - Not used
    * right (+x)     - Tank turn right
    * left (-x)      - Tank turn left
    */
   public void RunMecanumWheels()
   {
      double LFPower = 0.0;
      double RFPower = 0.0;
      double LRPower = 0.0;
      double RRPower = 0.0;
      double leftStickY = 0.0;
      double leftStickX = 0.0;
      double rightStickX = 0.0;
      // Calculate motor powers but only if any of the joystick commands are greater then
      // a minimum threshold.  Adjust this threshold if the motor has motion when the joystick
      // is not being used and in the center position.
      if ((Math.abs(gamepad1.left_stick_y) >= joystickThreshold) ||
         (Math.abs(gamepad1.left_stick_x) >= joystickThreshold) ||
         (Math.abs(gamepad1.right_stick_x) >= joystickThreshold))
      {
         leftStickY = robot.ScaleJoystickCommand(gamepad1.left_stick_y);
         leftStickX = robot.ScaleJoystickCommand(gamepad1.left_stick_x);
         rightStickX = robot.ScaleJoystickCommand(gamepad1.right_stick_x);


         // Calculate power for each mecanum wheel based on joystick inputs.  Each power is
         // based on three drive components: forward/reverse, strafe, and tank turn.
         if (robot.driveMode == Crossfire_Hardware.driveModeEnum.beaconMode)
         {
            LFPower = (forwardPriority * leftStickY) - (strafePriority * leftStickX) - (steerPriority * rightStickX);
            RFPower = (forwardPriority * leftStickY) + (strafePriority * leftStickX) + (steerPriority * rightStickX);
            LRPower = (forwardPriority * leftStickY) + (strafePriority * leftStickX) - (steerPriority * rightStickX);
            RRPower = (forwardPriority * leftStickY) - (strafePriority * leftStickX) + (steerPriority * rightStickX);
            telemetry.addData("Mode: ", "Beacon");

//            LFPower = (forwardPriority * gamepad1.left_stick_y) - (strafePriority * gamepad1.left_stick_x) - (steerPriority * gamepad1.right_stick_x);
//            RFPower = (forwardPriority * gamepad1.left_stick_y) + (strafePriority * gamepad1.left_stick_x) + (steerPriority * gamepad1.right_stick_x);
//            LRPower = (forwardPriority * gamepad1.left_stick_y) + (strafePriority * gamepad1.left_stick_x) - (steerPriority * gamepad1.right_stick_x);
////            RRPower = (forwardPriority * gamepad1.left_stick_y) - (strafePriority * gamepad1.left_stick_x) + (steerPriority * gamepad1.right_stick_x);
         }



         if (robot.driveMode == Crossfire_Hardware.driveModeEnum.ballKickerMode)
         {
            LFPower = (forwardPriority * -leftStickY) + (strafePriority * leftStickX) - (steerPriority * rightStickX);
            RFPower = (forwardPriority * -leftStickY) - (strafePriority * leftStickX) + (steerPriority * rightStickX);
            LRPower = (forwardPriority * -leftStickY) - (strafePriority * leftStickX) - (steerPriority * rightStickX);
            RRPower = (forwardPriority * -leftStickY) + (strafePriority * leftStickX) + (steerPriority * rightStickX);

//            LFPower = (forwardPriority * -gamepad1.left_stick_y) + (strafePriority * gamepad1.left_stick_x) - (steerPriority * gamepad1.right_stick_x);
//            RFPower = (forwardPriority * -gamepad1.left_stick_y) - (strafePriority * gamepad1.left_stick_x) + (steerPriority * gamepad1.right_stick_x);
//            LRPower = (forwardPriority * -gamepad1.left_stick_y) - (strafePriority * gamepad1.left_stick_x) - (steerPriority * gamepad1.right_stick_x);
//            RRPower = (forwardPriority * -gamepad1.left_stick_y) + (strafePriority * gamepad1.left_stick_x) + (steerPriority * gamepad1.right_stick_x);
            telemetry.addData("Mode: ", "Ball Kicker");
         }

         telemetry.update();


         // Find maximum power commanded to all the mecanum wheels.  Using the above power
         // equations, it is possible to calculate a power command greater than 1.0f (100%).
         // We want to find the max value so we can proportionally reduce motor powers.
         double maxPower = Math.max(LFPower, Math.max(RFPower, Math.max(LRPower, RRPower)));

         // If max power is greater than 1.0f (100% command), then proportionally reduce all motor
         // powers by the maximum power calculated.  This will equally reduce all powers so no
         // motor power is clipped and the robot responds predictably to joystick commands.  If we
         // don't, then one or more motor commands will clip, others will not, and the robot will not
         // behave predictably.  The end result of this reduction is the motor requesting max power
         // will set power to 1.0f (100%) and all other powers will be reduced by the same ratio.
         if (Math.abs(maxPower) > 1.0f)
         {
            LFPower /= maxPower; // Shorthand for LFPower = LFPower / maxPower
            RFPower /= maxPower;
            LRPower /= maxPower;
            RRPower /= maxPower;
         }

         // Update motor powers with new value.
         robot.setMecanumPowers(LFPower, RFPower, LRPower, RRPower);
      }
      else
      {
         // Explicitly set powers to zero.  May not be necessary but is good practice.
         robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);
      }
   }


   /***
    * Method operates the servo to push the beacon button.  To push
    * left-hand button, press and hold x to rotate serve CCW. To push
    * right-hand button, press and hold b button to rotate servo CW.
    */
   private void ServiceServos()
   {
      double ButtonPusherPosition = robot.GetButtonPusherPosition();
      double FlickerPosition = robot.GetFlickerPosition();

      // Rotate CCW
      if (gamepad2.x)
      {
         robot.SetButtonPusherPosition(ButtonPusherPosition + beaconPusherRate);
      }

      // Rotate CW
      else if (gamepad2.b)
      {
         robot.SetButtonPusherPosition(ButtonPusherPosition - beaconPusherRate);
      }

      // Up Theoretically
      if (gamepad2.a)
      {
         robot.SetFlickerPosition(0.08);
      }

      // Down Theoretically
      else if (gamepad2.y)
      {
         robot.SetFlickerPosition(0.90);
      }
   }


//   public void GetAdafruitColor()
//   {
//      // convert the RGB values to HSV values.
//      Color.RGBToHSV((robot.sensorRGB.red() * 255) / 800, (robot.sensorRGB.green() * 255) / 800, (robot.sensorRGB.blue() * 255) / 800, hsvValues);
//
//      // Determine sensor color based on thresholds
//      if ((hsvValues[0] >= BlueLowerLimit) && (hsvValues[0] <= BlueUpperLimit))
//      {
//         color = sensorColor.blue;
//      }
//      else if ((hsvValues[0] >= RedLowerLimit) && (hsvValues[0] <= RedUpperLimit))
//      {
//         color = sensorColor.red;
//      }
//   }
}
