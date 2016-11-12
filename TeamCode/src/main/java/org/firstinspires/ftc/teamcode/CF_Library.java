package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/**
 * Created by Ryley on 9/20/16.
 */

public abstract class CF_Library extends LinearOpMode {
    Crossfire_Hardware robot   = new Crossfire_Hardware();
    public void setPower(float power) {
        robot.LeftFrontMotor.setPower(power);
        robot.RightFrontMotor.setPower(power);
        robot.LeftRearMotor.setPower(power);
        robot.RightRearMotor.setPower(power);
    }

    public void setLeftPower(float power) {
        robot.LeftFrontMotor.setPower(power);
        robot.LeftRearMotor.setPower(power);
    }

    public void setRightPower(float rpower) {
        robot.RightFrontMotor.setPower(rpower);
        robot.RightRearMotor.setPower(rpower);
    }

    public void setMode(DcMotor.RunMode mode) {
        robot.LeftFrontMotor.setMode(mode);
        robot.RightFrontMotor.setMode(mode);
        robot.LeftRearMotor.setMode(mode);
        robot.RightRearMotor.setMode(mode);
    }

    public void encoderMove(int countLeft, int countRight, double leftPower, double rightPower){

        robot.LeftFrontMotor.setPower(leftPower);
        robot.RightFrontMotor.setPower(rightPower);
        robot.LeftRearMotor.setPower(leftPower);
        robot.RightRearMotor.setPower(rightPower);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.LeftFrontMotor.setTargetPosition(countLeft);
        robot.RightFrontMotor.setTargetPosition(countRight);
        robot.LeftRearMotor.setTargetPosition(countLeft);
        robot.RightRearMotor.setTargetPosition(countRight);

        while(robot.LeftFrontMotor.isBusy() && robot.RightFrontMotor.isBusy() && robot.LeftRearMotor.isBusy() && robot.RightRearMotor.isBusy()) {
            double leftPos = robot.LeftFrontMotor.getCurrentPosition();
            double rightPos = robot.RightFrontMotor.getCurrentPosition();
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
    public void encoderStrafeRight(int count, float power) throws InterruptedException{
        setLeftPower(power);
        setRightPower(power);
        setPower(power);
        robot.LeftFrontMotor.setTargetPosition(count);
        robot.RightFrontMotor.setTargetPosition(count * -1);
        robot.LeftRearMotor.setTargetPosition(count * -1);
        robot.RightRearMotor.setTargetPosition(count);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(robot.LeftFrontMotor.isBusy() && robot.RightFrontMotor.isBusy() && robot.LeftRearMotor.isBusy() && robot.RightRearMotor.isBusy()) {
            double leftPos = robot.LeftRearMotor.getCurrentPosition();
            double rightPos = robot.RightRearMotor.getCurrentPosition();
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
    public void encoderStrafeLeft(int count, float power) throws InterruptedException{
        setLeftPower(power);
        setRightPower(power);
        setPower(power);
        robot.LeftFrontMotor.setTargetPosition(count * -1);
        robot.RightFrontMotor.setTargetPosition(count);
        robot.LeftRearMotor.setTargetPosition(count);
        robot.RightRearMotor.setTargetPosition(count * -1);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(robot.LeftFrontMotor.isBusy() && robot.RightFrontMotor.isBusy() && robot.LeftRearMotor.isBusy() && robot.RightRearMotor.isBusy()) {
            double leftPos = robot.LeftRearMotor.getCurrentPosition();
            double rightPos = robot.RightRearMotor.getCurrentPosition();
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
