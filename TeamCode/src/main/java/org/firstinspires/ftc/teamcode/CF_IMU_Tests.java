package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cController;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.concurrent.TimeUnit;

/**
 * Created by Ryley on 3/5/17.
 */
@Autonomous(name = "CF_IMU_Tests", group = "test")
//@Disabled
public class CF_IMU_Tests extends CF_Library_Test {


    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    Acceleration accel;
    Orientation ang;

    @Override public void runOpMode() throws  InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorMecanumLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorMecanumLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

        //while(opModeIsActive()) {
            encoderMove(1000, 1000, 0.2f, 0.2f);
            //            telemetry.clear();
//           // accel = imu.getAcceleration();
//            ang = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.XYZ);
//            //encoderStrafeLeft(8000, 0.20f);
//            encoderStrafeRightNew(5000, 0.5f, imu);
//            TimeUnit.MILLISECONDS.sleep(20);
//            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            telemetry.addData("Ang", ang.thirdAngle);
//            telemetry.addData("Type", ang.angleUnit);
//            telemetry.update();
       // }

    }

}
