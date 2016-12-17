package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Ryley on 9/20/16.
 */

public abstract class CF_Library extends LinearOpMode {
   Crossfire_Hardware robot   = new Crossfire_Hardware();
   static double TIMEOUT = 28;
   public void setPower(float power) {
      robot.MotorMecanumLeftFront.setPower(power);
      robot.MotorMecanumRightFront.setPower(power);
      robot.MotorMecanumLeftRear.setPower(power);
      robot.MotorMecanumRightRear.setPower(power);
   }

   public void setLeftPower(float power) {
      robot.MotorMecanumLeftFront.setPower(power);
      robot.MotorMecanumLeftRear.setPower(power);
   }

   public void setRightPower(float rpower) {
      robot.MotorMecanumRightFront.setPower(rpower);
      robot.MotorMecanumRightRear.setPower(rpower);
   }

   public void setMode(DcMotor.RunMode mode) {
      robot.MotorMecanumLeftFront.setMode(mode);
      robot.MotorMecanumRightFront.setMode(mode);
      robot.MotorMecanumLeftRear.setMode(mode);
      robot.MotorMecanumRightRear.setMode(mode);
   }

   public void encoderMove(int countLeft, int countRight, double leftPower, double rightPower, ElapsedTime time){

      robot.MotorMecanumLeftFront.setPower(leftPower);
      robot.MotorMecanumRightFront.setPower(rightPower);
      robot.MotorMecanumLeftRear.setPower(leftPower);
      robot.MotorMecanumRightRear.setPower(rightPower);

      setMode(DcMotor.RunMode.RUN_TO_POSITION);

      robot.MotorMecanumLeftFront.setTargetPosition(countLeft);
      robot.MotorMecanumRightFront.setTargetPosition(countRight);
      robot.MotorMecanumLeftRear.setTargetPosition(countLeft);
      robot.MotorMecanumRightRear.setTargetPosition(countRight);

      while(robot.MotorMecanumLeftFront.isBusy() && robot.MotorMecanumRightFront.isBusy() && robot.MotorMecanumLeftRear.isBusy() && robot.MotorMecanumRightRear.isBusy() && time.seconds() < TIMEOUT) {
         double leftPos = robot.MotorMecanumLeftFront.getCurrentPosition();
         double rightPos = robot.MotorMecanumRightFront.getCurrentPosition();
         telemetry.addData("Right",rightPos);
         telemetry.addData("Left",leftPos);
         telemetry.update();
         try {
            idle();
         }
         catch(InterruptedException e) {
            telemetry.addData("Idle Failed", "Idle Failed");
         }

      }
      setPower(0.0f);
      setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   }
   public void encoderStrafeRight(int count, float power, ElapsedTime time) throws InterruptedException{
      setLeftPower(power);
      setRightPower(power);
      setPower(power);
      robot.MotorMecanumLeftFront.setTargetPosition(count);
      robot.MotorMecanumRightFront.setTargetPosition(count * -1);
      robot.MotorMecanumLeftRear.setTargetPosition(count * -1);
      robot.MotorMecanumRightRear.setTargetPosition(count);
      setMode(DcMotor.RunMode.RUN_TO_POSITION);
      while(robot.MotorMecanumLeftFront.isBusy() && robot.MotorMecanumRightFront.isBusy() && robot.MotorMecanumLeftRear.isBusy() && robot.MotorMecanumRightRear.isBusy() && time.seconds() < TIMEOUT) {
         double leftPos = robot.MotorMecanumLeftRear.getCurrentPosition();
         double rightPos = robot.MotorMecanumRightRear.getCurrentPosition();
         telemetry.addData("Right",rightPos);
         telemetry.addData("Left",leftPos);
         telemetry.update();
         try {
            idle();
         }
         catch(InterruptedException e) {
            telemetry.addData("Idle Failed", "Idle Failed");
         }
      }

      setPower(0.0f);
      setMode(DcMotor.RunMode.RUN_USING_ENCODER);

   }
   public void encoderStrafeLeft(int count, float power, ElapsedTime time) throws InterruptedException{
      setLeftPower(power);
      setRightPower(power);
      setPower(power);
      robot.MotorMecanumLeftFront.setTargetPosition(count * -1);
      robot.MotorMecanumRightFront.setTargetPosition(count);
      robot.MotorMecanumLeftRear.setTargetPosition(count);
      robot.MotorMecanumRightRear.setTargetPosition(count * -1);
      setMode(DcMotor.RunMode.RUN_TO_POSITION);
      while(robot.MotorMecanumLeftFront.isBusy() && robot.MotorMecanumRightFront.isBusy() && robot.MotorMecanumLeftRear.isBusy() && robot.MotorMecanumRightRear.isBusy() && time.seconds() < TIMEOUT) {
         double leftPos = robot.MotorMecanumLeftRear.getCurrentPosition();
         double rightPos = robot.MotorMecanumRightRear.getCurrentPosition();
         telemetry.addData("Right",rightPos);
         telemetry.addData("Left",leftPos);
         telemetry.update();
         try {
            idle();
         }
         catch(InterruptedException e) {
            telemetry.addData("Idle Failed", "Idle Failed");
         }
      }

      setPower(0.0f);
      setMode(DcMotor.RunMode.RUN_USING_ENCODER);

   }

}