package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/***
 * This file provides basic Telop driving for a robot with mecanum drive
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the CrossFire hardware class to define the devices on the robot.
 * All device access is managed through the Crossfire_Hardware class.
 *
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

   //Sets hue value limits for color sensor
   private static final int RedUpperLimit = 360;
   private static final int RedLowerLimit = 325;
   private static final int BlueUpperLimit = 270;
   private static final int BlueLowerLimit = 220;

   // hsvValues is an array that will hold the hue, saturation, and value information.
   float hsvValues[] = {0F,0F,0F};

   // values is a reference to the hsvValues array.
   final float values[] = hsvValues;

   public void init()
   {
      robot.init(hardwareMap);
   }

   public void loop()
   {
      RunMecanumWheels();
      ServiceServo();
      RunColorSensor();
   }

   public void RunMecanumWheels()
   {
      // Calculate motor powers but only if any of the joystick commands are greater then
      // a minimum threshold.  Adjust this threshold if the motor has motion when the joystick
      // is not being used and in the center position.
      if ((Math.abs(gamepad1.left_stick_y) >= joystickThreshold)  ||
          (Math.abs(gamepad1.left_stick_x) >= joystickThreshold)  ||
          (Math.abs(gamepad1.right_stick_x) >= joystickThreshold))
      {
         // Calculate power for each mecanum wheel based on joystick inputs.  Each power is
         // based on three drive components: forward/reverse, strafe, and tank turn.
         float LFPower = (forwardPriority * gamepad1.left_stick_y) - (strafePriority * gamepad1.left_stick_x) - (steerPriority * gamepad1.right_stick_x);
         float RFPower = (forwardPriority * gamepad1.left_stick_y) + (strafePriority * gamepad1.left_stick_x) + (steerPriority * gamepad1.right_stick_x);
         float LRPower = (forwardPriority * gamepad1.left_stick_y) + (strafePriority * gamepad1.left_stick_x) - (steerPriority * gamepad1.right_stick_x);
         float RRPower = (forwardPriority * gamepad1.left_stick_y) - (strafePriority * gamepad1.left_stick_x) + (steerPriority * gamepad1.right_stick_x);

         // Find maximum power commanded to all the mecanum wheels.  Using the above power
         // equations, it is possible to calculate a power command greater than 1.0f (100%).
         // We want to find the max value so we can proportionally reduce motor powers.
         float maxPower = Math.max(LFPower, Math.max(RFPower, Math.max(LRPower, RRPower)));

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

         // Send power levels to the phone
         telemetry.addData("LFPower",  "%.2f", LFPower);
         telemetry.addData("RFPower",  "%.2f", RFPower);
         telemetry.addData("LRPower",  "%.2f", LRPower);
         telemetry.addData("RRPower",  "%.2f", RRPower);
         updateTelemetry(telemetry);
      }
      else
      {
         // Explicitly set powers to zero.  May not be necessary but is good practice.
         robot.setMecanumPowers(0.0f, 0.0f, 0.0f, 0.0f);
      }
   }


   private void ServiceServo()
   {
      double ButtonPusherPosition = robot.GetButtonPusherPosition();

      if (gamepad1.x)
      {
         robot.SetButtonPusherPosition(ButtonPusherPosition + 0.005);
      }

      else if (gamepad1.b)
      {
         robot.SetButtonPusherPosition(ButtonPusherPosition - 0.005);
      }

//      telemetry.addData("Servo Position" , ButtonPusherPosition);
//      updateTelemetry(telemetry);
   }

   public void RunColorSensor()
   {
      // convert the RGB values to HSV values.
      telemetry.clear();
      Color.RGBToHSV((robot.sensorRGB.red() * 255) / 800, (robot.sensorRGB.green() * 255) / 800, (robot.sensorRGB.blue() * 255) / 800, hsvValues);

//      telemetry.addData("Clear", robot.sensorRGB.alpha());
//      telemetry.addData("Red  ", robot.sensorRGB.red());
//      telemetry.addData("Green", robot.sensorRGB.green());
//      telemetry.addData("Blue ", robot.sensorRGB.blue());
//      telemetry.addData("Hue", hsvValues[0]);

//      Get sensor color
//      if ((hsvValues[0] >= BlueLowerLimit) && (hsvValues[0] <= BlueUpperLimit))
//      {
//         telemetry.addData("Beacon is ", "Blue");
//      }
//
//      else if ((hsvValues[0] >= RedLowerLimit) && (hsvValues[0] <= RedUpperLimit))
//      {
//         telemetry.addData("Beacon is ", "Red");
//      }
//
//      else
//      {
//         telemetry.addData("Beacon is ", "Unknown");
//      }


      // change the background color to match the color detected by the RGB sensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.

//      //relativeLayout.post(new Runnable() {
//         public void run() {
//            relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
//         }
      //}//);
   }
}
