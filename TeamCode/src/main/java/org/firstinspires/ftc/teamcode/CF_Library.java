package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/**
 * Created by Ryley on 9/20/16.
 */

public abstract class CF_Library extends LinearOpMode {
    Crossfire_Hardware robot   = new Crossfire_Hardware();
    public void encoderMove(int countLeft, int countRight, double leftPower, double rightPower){

        robot.leftMotor.setPower(leftPower);
        robot.rightMotor.setPower(rightPower);
        robot.leftMotor.setTargetPosition(countLeft);
        robot.rightMotor.setTargetPosition(countRight);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) {
            double rightPos = robot.rightMotor.getCurrentPosition();
            double leftPos = robot.leftMotor.getCurrentPosition();
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
        robot.leftMotor.setPower(0.0f);
        robot.rightMotor.setPower(0.0f);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
