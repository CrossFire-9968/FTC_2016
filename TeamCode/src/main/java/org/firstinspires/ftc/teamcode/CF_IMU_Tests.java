package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
public class CF_IMU_Tests extends CF_Library {
    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;
    Acceleration accel;
    AngularVelocity ang;

    @Override public void runOpMode() throws  InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

        while(opModeIsActive()) {

            telemetry.clear();
//            accel = imu.getAcceleration();
            ang = imu.getAngularVelocity().toAngleUnit(AngleUnit.DEGREES);
            encoderStrafeLeftNew(3000, (float) 0.5, imu);
            telemetry.addData("Ang Vel", ang.thirdAngleRate);
            telemetry.update();
            idle();
        }


    }
}
