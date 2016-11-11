package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Ryley on 9/20/16.
 */

public abstract class CF_Library extends LinearOpMode
{
   Crossfire_Hardware robot = new Crossfire_Hardware();



   public void encoderMove(int countLeft, int countRight, double leftPower, double rightPower)
   {
      robot.setMecanumEncoderTargetPosition(countLeft, countRight, countLeft, countRight);
      robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.setMecanumPowers(leftPower, rightPower, leftPower, rightPower);

      while (robot.MotorMecanumLeftFront.isBusy() &&
             robot.MotorMecanumRightFront.isBusy() &&
             robot.MotorMecanumLeftRear.isBusy() &&
             robot.MotorMecanumRightRear.isBusy())
      {
         double leftPos = robot.MotorMecanumLeftFront.getCurrentPosition();
         double rightPos = robot.MotorMecanumRightFront.getCurrentPosition();

         telemetry.addData("Right", rightPos);
         telemetry.addData("Left", leftPos);
         telemetry.update();

         try
         {
            idle();
         }
         catch (InterruptedException e)
         {
            telemetry.addData("Idle Failed", "Idle Failed");
         }

      }

      // Turn off motors and turn off RUN_TO_POSITION
      robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);
      robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
   }


   public void encoderStrafe(int countLeft, int countRight, float leftPower, float rightPower, int direction) throws InterruptedException
   {
      robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
      switch (direction)
      {
         case 0:
            robot.setMecanumPowers(leftPower, rightPower, leftPower, rightPower);
            robot.setMecanumEncoderTargetPosition(countLeft, countRight, countLeft, countRight);

            while (robot.MotorMecanumLeftFront.isBusy() &&
               robot.MotorMecanumRightFront.isBusy() &&
               robot.MotorMecanumLeftRear.isBusy() &&
               robot.MotorMecanumRightRear.isBusy())
            {
               idle();
            }
            break;

         case 1:
            robot.setMecanumPowers(leftPower, rightPower, leftPower, rightPower);
            robot.setMecanumEncoderTargetPosition(countLeft, countRight, countLeft, countRight);

            while (robot.MotorMecanumLeftFront.isBusy() &&
               robot.MotorMecanumRightFront.isBusy() &&
               robot.MotorMecanumLeftRear.isBusy() &&
               robot.MotorMecanumRightRear.isBusy())
            {
               idle();
            }
            break;

         default:
            break;
      }
   }
//
//   public void go(float power)
//   {
//      setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
//      robot.RightRearMotor.setPower(0.8f);
//   }

}