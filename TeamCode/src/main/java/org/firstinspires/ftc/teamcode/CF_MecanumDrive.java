package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by jd72958 on 11/7/2016.
 */

@TeleOp(name = "CF_Mecanum_Drive", group = "Drivetrain")
@Disabled

public class CF_MecanumDrive extends OpMode
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

   public void init()
   {
      robot.init(hardwareMap);
   }

   public void loop()
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
