package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Ryley on 9/20/16.
 */

public abstract class CF_Library_Test extends LinearOpMode {
    Crossfire_Hardware robot   = new Crossfire_Hardware();
    public void setPower(float power) {
        robot.MotorMecanumLeftFront.setPower(power);
        robot.MotorMecanumRightFront.setPower(power);
        robot.MotorMecanumLeftRear.setPower(power);
        robot.MotorMecanumRightRear.setPower(power);
    }
    public void setFrontPower(float power) {
        robot.MotorMecanumRightFront.setPower(power);
        robot.MotorMecanumLeftFront.setPower(power);
    }
    public void setRearPower(float power) {
        robot.MotorMecanumLeftRear.setPower(power);
        robot.MotorMecanumRightRear.setPower(power);
    }
    public void setLeftPower(float power) {
        robot.MotorMecanumLeftFront.setPower(power);
        robot.MotorMecanumLeftRear.setPower(power);
    }

    public void setRightPower(float power) {
        robot.MotorMecanumRightFront.setPower(power);
        robot.MotorMecanumRightRear.setPower(power);
    }

    public void setMode(DcMotor.RunMode mode) {
        robot.MotorMecanumLeftFront.setMode(mode);
        robot.MotorMecanumRightFront.setMode(mode);
        robot.MotorMecanumLeftRear.setMode(mode);
        robot.MotorMecanumRightRear.setMode(mode);
    }

    public void encoderMove(int countLeft, int countRight, double leftPower, double rightPower){

        boolean keepGoing = true;
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.MotorMecanumLeftFront.setPower(leftPower);
        robot.MotorMecanumRightFront.setPower(rightPower);
        robot.MotorMecanumLeftRear.setPower(leftPower);
        robot.MotorMecanumRightRear.setPower(rightPower);

        robot.MotorMecanumLeftFront.setTargetPosition(countLeft);
        robot.MotorMecanumRightFront.setTargetPosition(countRight);
        robot.MotorMecanumLeftRear.setTargetPosition(countLeft);
        robot.MotorMecanumRightRear.setTargetPosition(countRight);


        while(keepGoing) {
            //telemetry.addData("Right",rightPos);
            //telemetry.addData("Left",leftPos);
            //telemetry.update();
            if((!robot.MotorMecanumRightRear.isBusy() && !robot.MotorMecanumLeftFront.isBusy()) ||  (!robot.MotorMecanumRightFront.isBusy() && !robot.MotorMecanumLeftRear.isBusy())) {
                keepGoing = false;
                //setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            try {
                idle();
            }
            catch(InterruptedException e) {
                //telemetry.addData("Idle Failed", "Idle Failed");
            }
        }
        setPower(0.0f);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void encoderStrafeLeftNew(int count, float power, BNO055IMU inertial) throws InterruptedException{
        boolean keepGoing = true;
        Orientation ang;
        double error;
        double effort;
        double leftPower;
        double rightPower;
        double kP = 0.025;
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int target;
        target = ((Math.abs(robot.MotorMecanumLeftFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumLeftRear.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightRear.getCurrentPosition())) / 4) + count;


//        robot.MotorMecanumLeftFront.setPower(power);
//        robot.MotorMecanumRightFront.setPower(power);
//        robot.MotorMecanumLeftRear.setPower(power);
//        robot.MotorMecanumRightRear.setPower(power);
//
//        robot.MotorMecanumLeftFront.setTargetPosition(count * -1);
//        robot.MotorMecanumRightFront.setTargetPosition(count);
//        robot.MotorMecanumLeftRear.setTargetPosition(count);
//        robot.MotorMecanumRightRear.setTargetPosition(count * -1);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(keepGoing) {

            if(isStopRequested()) {
                keepGoing = false;
            }
            //error = inertial.getAngularVelocity().toAngleUnit(AngleUnit.DEGREES).thirdAngleRate;
            ang = inertial.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.XYZ);
            Position pos = inertial.getPosition();
            error = ang.thirdAngle;
            effort = kP * error;

            robot.MotorMecanumLeftFront.setPower((-1 * power) + effort);
            robot.MotorMecanumRightFront.setPower(power - effort);
            robot.MotorMecanumLeftRear.setPower(power + effort);
            robot.MotorMecanumRightRear.setPower((-1 * power) - effort);

//            setLeftPower((float)frontPower);
//            setRightPower((float)rearPower);
            telemetry.clearAll();
            telemetry.addData("error", effort);
            telemetry.addData("ang", ang);
            telemetry.addData("pos", (Math.abs(robot.MotorMecanumLeftFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumLeftRear.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightRear.getCurrentPosition())) / 4);
            telemetry.update();
            if(((Math.abs(robot.MotorMecanumLeftFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumLeftRear.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightRear.getCurrentPosition())) / 4)  > target) {
                keepGoing = false;
                //setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
//            try {
//                idle();
//            }
//            catch(InterruptedException e) {
//                telemetry.addData("Idle Failed", "Idle Failed");
//            }
        }

        setPower(0.0f);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void encoderStrafeRightNew(int count, float power, BNO055IMU inertial) throws InterruptedException{
        boolean keepGoing = true;
        Orientation ang;
        double error;
        double effort;
        double leftPower;
        double rightPower;
        double kP = 0.025;
        int target = ((Math.abs(robot.MotorMecanumLeftFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumLeftRear.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightRear.getCurrentPosition())) / 4)+ count;

//        robot.MotorMecanumLeftFront.setPower(power);
//        robot.MotorMecanumRightFront.setPower(power);
//        robot.MotorMecanumLeftRear.setPower(power);
//        robot.MotorMecanumRightRear.setPower(power);
//
//        robot.MotorMecanumLeftFront.setTargetPosition(count * -1);
//        robot.MotorMecanumRightFront.setTargetPosition(count);
//        robot.MotorMecanumLeftRear.setTargetPosition(count);
//        robot.MotorMecanumRightRear.setTargetPosition(count * -1);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(keepGoing) {

            if(isStopRequested()) {
                keepGoing = false;
            }
            //error = inertial.getAngularVelocity().toAngleUnit(AngleUnit.DEGREES).thirdAngleRate;
            ang = inertial.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.XYZ);
            error = ang.thirdAngle;
            effort = kP * error;
            leftPower = power - effort;
            rightPower = power + effort;
            robot.MotorMecanumLeftFront.setPower(power - effort);
            robot.MotorMecanumRightFront.setPower((-1 * power) + effort);
            robot.MotorMecanumLeftRear.setPower((-1 * power) - effort);
            robot.MotorMecanumRightRear.setPower(power + effort);

//            setLeftPower((float)frontPower);
//            setRightPower((float)rearPower);
            telemetry.clearAll();
            telemetry.addData("front", leftPower);
            telemetry.addData("rear", rightPower);
            telemetry.addData("error", effort);
            telemetry.addData("ang", ang);
            telemetry.addData("pos", ((Math.abs(robot.MotorMecanumLeftFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumLeftRear.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightRear.getCurrentPosition())) / 4));
            //telemetry.update();
            if(((Math.abs(robot.MotorMecanumLeftFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumLeftRear.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightRear.getCurrentPosition())) / 4)  > target) {
                keepGoing = false;
                //setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
//            try {
//                idle();
//            }
//            catch(InterruptedException e) {
//                telemetry.addData("Idle Failed", "Idle Failed");
//            }
        }

        setPower(0.0f);

    }
    public void encoderMoveNew(int count, float power, BNO055IMU inertial) {
        boolean keepGoing = true;
        Orientation ang;
        double error;
        double effort;
        double leftPower;
        double rightPower;
        double kP = 0.025;
        int target = ((Math.abs(robot.MotorMecanumLeftFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumLeftRear.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightRear.getCurrentPosition())) / 4)+ count;

//        robot.MotorMecanumLeftFront.setPower(power);
//        robot.MotorMecanumRightFront.setPower(power);
//        robot.MotorMecanumLeftRear.setPower(power);
//        robot.MotorMecanumRightRear.setPower(power);
//
//        robot.MotorMecanumLeftFront.setTargetPosition(count * -1);
//        robot.MotorMecanumRightFront.setTargetPosition(count);
//        robot.MotorMecanumLeftRear.setTargetPosition(count);
//        robot.MotorMecanumRightRear.setTargetPosition(count * -1);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(keepGoing) {
            if(isStopRequested()) {
                keepGoing = false;
            }

            ang = inertial.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.XYZ);
            error = ang.thirdAngle;
            effort = kP * error;
            leftPower = power + effort;
            rightPower = power - effort;
            robot.MotorMecanumLeftFront.setPower(leftPower);
            robot.MotorMecanumLeftRear.setPower(leftPower);
            robot.MotorMecanumRightFront.setPower(rightPower);
            robot.MotorMecanumRightRear.setPower(rightPower);

            telemetry.clearAll();
            telemetry.addData("ang", ang);
            telemetry.addData("pos", ((Math.abs(robot.MotorMecanumLeftFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumLeftRear.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightRear.getCurrentPosition())) / 4));
            //telemetry.update();

            if(((Math.abs(robot.MotorMecanumLeftFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightFront.getCurrentPosition()) + Math.abs(robot.MotorMecanumLeftRear.getCurrentPosition()) + Math.abs(robot.MotorMecanumRightRear.getCurrentPosition())) / 4) > target) {
                keepGoing = false;
            }

        }
        setPower(0.0f);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void encoderStrafeRight(int count, float power) throws InterruptedException{
        boolean keepGoing = true;
        double ang;
        double error;
        double effort;
        double leftPower;
        double rightPower;
        double kP = 0.00045;

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.MotorMecanumLeftFront.setPower(power);
        robot.MotorMecanumRightFront.setPower(power);
        robot.MotorMecanumLeftRear.setPower(power/* + 0.05f*/);
        robot.MotorMecanumRightRear.setPower(power/* + 0.05f*/);
//      setFrontPower(power);
//      setRearPower(power + 0.05f);
        robot.MotorMecanumLeftFront.setTargetPosition(count);
        robot.MotorMecanumRightFront.setTargetPosition(count * -1);
        robot.MotorMecanumLeftRear.setTargetPosition(count * -1);
        robot.MotorMecanumRightRear.setTargetPosition(count);
        while(keepGoing) {
            //telemetry.update();
            if((!robot.MotorMecanumRightRear.isBusy() && !robot.MotorMecanumLeftFront.isBusy()) ||  (!robot.MotorMecanumRightFront.isBusy() && !robot.MotorMecanumLeftRear.isBusy())) {
                keepGoing = false;
                //setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
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
    public void encoderStrafeLeftDualPower(int countOne, float powerOne, int countTwo, float powerTwo) {
        boolean keepGoing = true;
        setPower(powerOne);
        robot.MotorMecanumLeftFront.setTargetPosition(countOne + countTwo);
        robot.MotorMecanumRightFront.setTargetPosition((countOne + countTwo) * -1);
        robot.MotorMecanumLeftRear.setTargetPosition((countOne + countTwo) * -1);
        robot.MotorMecanumRightRear.setTargetPosition(countOne + countTwo);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(keepGoing) {
            double leftPos = robot.MotorMecanumLeftRear.getCurrentPosition();
            double rightPos = robot.MotorMecanumRightRear.getCurrentPosition();

            if(!robot.MotorMecanumRightRear.isBusy() || !robot.MotorMecanumRightFront.isBusy() || !robot.MotorMecanumLeftFront.isBusy() || !robot.MotorMecanumLeftRear.isBusy()) {
                keepGoing = false;
                //setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(leftPos > countOne && rightPos > countOne) {
                setPower(powerTwo);
            }
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
        boolean keepGoing = true;
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setRightPower(power);
        setLeftPower(power);
        setPower(power);

        robot.MotorMecanumLeftFront.setTargetPosition(count * -1);
        robot.MotorMecanumRightFront.setTargetPosition(count);
        robot.MotorMecanumLeftRear.setTargetPosition(count);
        robot.MotorMecanumRightRear.setTargetPosition(count * -1);

        while(keepGoing) {
            double leftPos = robot.MotorMecanumLeftRear.getCurrentPosition();
            double rightPos = robot.MotorMecanumRightRear.getCurrentPosition();

            telemetry.addData("Right",rightPos);
            telemetry.addData("Left",leftPos);
            //telemetry.update();
            if((!robot.MotorMecanumRightRear.isBusy() && !robot.MotorMecanumLeftFront.isBusy()) ||  (!robot.MotorMecanumRightFront.isBusy() && !robot.MotorMecanumLeftRear.isBusy())) {
                keepGoing = false;
                setPower(0.0f);
                //setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
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
    public void strafe(double rearPower, double frontPower) {
        robot.MotorMecanumRightFront.setPower((float) frontPower);
        robot.MotorMecanumLeftFront.setPower((float) (-1 * frontPower));
        robot.MotorMecanumLeftRear.setPower((float) rearPower);
        robot.MotorMecanumRightRear.setPower((float) (-1 * rearPower));
    }
//
//   public void driveToBeacon (int pictureNumber, VuforiaTrackables beaconsArray) {
//      robot.SetButtonPusherPosition(0.45f);
//      OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beaconsArray.get(pictureNumber).getListener()).getRawPose();
//   }

}