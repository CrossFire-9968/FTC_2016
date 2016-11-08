package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by jd72958 on 11/1/2016.
 */

@TeleOp(name = "CF_Mecanaum_Manual", group = "Drivetrain")  // @Autonomous(...) is the other common choice
//Disabled

public class CF_MecanumDrive extends OpMode
{
   Crossfire_Hardware robot = new Crossfire_Hardware();
   private static final float joystickThreshold = 0.05f;
   private float LFPower;
   private float RFPower;
   private float LRPower;
   private float RRPower;

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
            LFPower = LFPower / maxPower;
            RFPower = RFPower / maxPower;
            LRPower = LRPower / maxPower;
            RRPower = RRPower / maxPower;
         }

         // Update motor powers with new value.
         robot.setMecanumPowers(LFPower, RFPower, LRPower, RRPower);
      }
      else
      {
         // Explicitly set powers to zero.  May not be necessary but is good practice.
         robot.setMecanumPowers(0.0f, 0.0f, 0.0f, 0.0f);
      }
   }
}
