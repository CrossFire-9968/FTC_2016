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
      robot.setMecanumPowers(leftPower, rightPower, leftPower, rightPower);

      robot.MotorMecanumLeftFront.setTargetPosition(countLeft);
      robot.MotorMecanumLeftRear.setTargetPosition(countLeft);
      robot.MotorMecanumRightFront.setTargetPosition(countRight);
      robot.MotorMecanumRightRear.setTargetPosition(countRight);

      robot.MotorMecanumLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.MotorMecanumLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.MotorMecanumRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.MotorMecanumRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      while (robot.MotorMecanumLeftFront.isBusy()   &&
             robot.MotorMecanumLeftRear.isBusy()    &&
             robot.MotorMecanumRightFront.isBusy()  &&
             robot.MotorMecanumRightRear.isBusy())
      {
         double rightPos = robot.MotorMecanumRightFront.getCurrentPosition();
         double leftPos = robot.MotorMecanumLeftFront.getCurrentPosition();


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

      robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);

      robot.MotorMecanumLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.MotorMecanumLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.MotorMecanumRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.MotorMecanumRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   }
}
