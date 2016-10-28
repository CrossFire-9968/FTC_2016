package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Anne on 10/24/2016.
 */


@TeleOp(name = "Manual", group = "Manual")
// @Disabled

public abstract class CF_Manual extends OpMode
{
   Crossfire_Hardware robot = new Crossfire_Hardware();
   public float left_stick_y = 0.0f;
   public float right_stick_y = 0.0f;

   @Override
   public void init()
   {
      robot.init(hardwareMap);
   }

   /*
  * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
  */
   @Override
   public void init_loop()
   {
   }

   /*
    * Code to run ONCE when the driver hits PLAY
    */
   @Override
   public void start()
   {
   }

   /* Initialize standard Hardware interfaces */
   public void loop()
   {
      robot.leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
      robot.rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

      // Set all motors to zero power
      robot.leftMotor.setPower(0);
      robot.rightMotor.setPower(0);
      setLeftControls();
      setRightControls();
   }

   public void setLeftControls()
   {
      if (left_stick_y > 0.0)
      {
         robot.leftMotor.setPower(0.50);
      }

      else if (left_stick_y < 0.0)
      {
         robot.leftMotor.setPower(-0.50);
      }

      else
      {
         robot.leftMotor.setPower(0.0);
      }
   }

   public void setRightControls()
   {
      if (right_stick_y > 0.0)
      {
         robot.rightMotor.setPower(0.50);
      }

      else if (right_stick_y < 0.0)
      {
         robot.rightMotor.setPower(-0.50);
      }

      else
      {
         robot.rightMotor.setPower(0.0);
      }
   }

   @Override
   public void stop()
   {
   }
}
