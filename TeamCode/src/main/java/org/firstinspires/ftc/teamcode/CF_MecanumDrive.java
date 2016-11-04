package org.firstinspires.ftc.teamcode;

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

@TeleOp(name = "CF_Mecanaum_Manual", group = "Drivetrain")  // @Autonomous(...) is the other common choice
//Disabled

public class CF_MecanumDrive extends OpMode
{
   Crossfire_Hardware robot = new Crossfire_Hardware();
   private float LFPower;
   private float RFPower;
   private float LRPower;
   private float RRPower;

   // Minimum joystick position before we assume value is good.
   // Near center, value could contain noise or offset that we want to ignore.
   private static final float joystickThreshold = 0.05f;

   public void init()
   {
      robot.init(hardwareMap);
   }

   public void loop()
   {
      float maxPower;

      // Calculate motor powers but only if any of the joystick commands are greater then
      // a minimum threshold.  Adjust this threshold if the motor has motion when the joystick
      // is not being used and in the center position.
      if ((Math.abs(gamepad1.left_stick_y) >= joystickThreshold)  ||
          (Math.abs(gamepad1.left_stick_x) >= joystickThreshold)  ||
          (Math.abs(gamepad1.right_stick_x) >= joystickThreshold))
      {
         // Calculate power for each mecanum wheel based on joystick inputs.  Each power is
         // based on three drive components: forward/reverse, strafe, and tank turn.
         LFPower = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
         RFPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
         LRPower = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
         RRPower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;

         // Find maximum power commanded to all the mecanum wheels.  Using the above power
         // equations, it is possible to calculate a power command greater than 1.0f (100%).
         // We want to find the max value so we can proportionally reduce motor powers.
         maxPower = Math.max(LFPower, Math.max(RFPower, Math.max(LRPower, RRPower)));

         // If max power is greater than 1.0f (100% command), then proportionally reduce all motor
         // powers by the maximum power calculated.  This will equally reduce all powers so no
         // motor power is clipped and the robot responds predictably to joystick commands.  If we
         // don't, then one or more motor commands will clip, others will not, and the robot will not
         // behave predictably.  The end result of this reduction is the motor requesting max power
         // will set power to 1.0f (100%) and all other powers will be reduced by the same ratio.
         if (maxPower > 1.0f)
         {
            LFPower /= maxPower; // Shorthand for LFPower = LFPower / maxPower
            RFPower /= maxPower;
            LRPower /= maxPower;
            RRPower /= maxPower;
         }

         // Update motor powers with new value.
         robot.setMecanumPowers(LFPower, RFPower, LRPower, RRPower);

         // Send power levels to the phone
         telemetry.addData("maxPower", "%.2f", maxPower);
         telemetry.addData("LFPower",  "%.2f", LFPower);
         telemetry.addData("RFPower",  "%.2f", RFPower);
         telemetry.addData("LRPower",  "%.2f", LRPower);
         telemetry.addData("RRPower",  "%.2f", RRPower);
      }
      else
      {
         // Explicitly set powers to zero.  May not be necessary but is good practice.
         robot.setMecanumPowers(0.0f, 0.0f, 0.0f, 0.0f);
      }
   }
}
