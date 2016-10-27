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

public abstract class CF_Manual extends LinearOpMode
{
   public DcMotor leftMotor = null;
   public DcMotor rightMotor = null;
   HardwareMap hwMap = null;
   public float left_stick_y = 0.0f;
   public float right_stick_y = 0.0f;

   /* Initialize standard Hardware interfaces */
   public void runOpMode(HardwareMap ahwMap) throws InterruptedException
   {
      // Save reference to Hardware map
      hwMap = ahwMap;

      // Define and Initialize Motors
      leftMotor = hwMap.dcMotor.get("left_drive");
      rightMotor = hwMap.dcMotor.get("right_drive");
      //armMotor    = hwMap.dcMotor.get("left_arm");
      leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
      rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

      waitForStart();
      while (opModeIsActive())
      {
         // Set all motors to zero power
         leftMotor.setPower(0);
         rightMotor.setPower(0);
         setLeftControls();
         setRightControls();
      }
   }


   public void setLeftControls()
   {
      if (left_stick_y > 0.0)
      {
         leftMotor.setPower(0.50);
      }

      else if (left_stick_y < 0.0)
      {
         leftMotor.setPower(-0.50);
      }

      else
      {
         leftMotor.setPower(0.0);
      }
   }

   public void setRightControls()
   {
      if (right_stick_y > 0.0)
      {
         rightMotor.setPower(0.50);
      }

      else if (right_stick_y < 0.0)
      {
         rightMotor.setPower(-0.50);
      }

      else
      {
         rightMotor.setPower(0.0);
      }
   }
}
