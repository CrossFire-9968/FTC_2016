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

    public void setRightPower(float power) {
        robot.RightFrontMotor.setPower(power);
        robot.RightRearMotor.setPower(power);
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
    public void encoderStrafe(int countLeft, int countRight, float leftPower, float rightPower, int direction) throws InterruptedException{
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        switch(direction) {
            case 0:
                setLeftPower(leftPower);
                setRightPower(rightPower);
                robot.LeftFrontMotor.setTargetPosition(countLeft);
                robot.RightFrontMotor.setTargetPosition(countRight);
                robot.LeftRearMotor.setTargetPosition(countLeft);
                robot.RightRearMotor.setTargetPosition(countRight);
                while(robot.LeftFrontMotor.isBusy() && robot.RightFrontMotor.isBusy() && robot.LeftRearMotor.isBusy() && robot.RightRearMotor.isBusy()) {
                    idle();
                }
                break;
            case 1:
                setLeftPower(leftPower);
                setRightPower(rightPower);
                robot.LeftFrontMotor.setTargetPosition(countLeft);
                robot.RightFrontMotor.setTargetPosition(countRight);
                robot.LeftRearMotor.setTargetPosition(countLeft);
                robot.RightRearMotor.setTargetPosition(countRight);
                while(robot.LeftFrontMotor.isBusy() && robot.RightFrontMotor.isBusy() && robot.LeftRearMotor.isBusy() && robot.RightRearMotor.isBusy()) {
                    idle();
                }
                break;
            default:
                break;
        }
    }
    public void go(float power) {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        robot.RightRearMotor.setPower(0.8f);
    }
}
