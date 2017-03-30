package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
@Autonomous(name = "CF_Color_Test", group = "test")
//@Disabled
public class CF_Color_Test extends CF_Library_Test {


    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    Acceleration accel;
    Orientation ang;

    @Override public void runOpMode() throws  InterruptedException {
        robot.init(hardwareMap);

        //setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorMecanumLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorMecanumLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ColorSensor sensorRGBright = hardwareMap.colorSensor.get("AdafruitRGBright");
        ColorSensor sensorRGBleft = hardwareMap.colorSensor.get("AdafruitRGBleft");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.clearAll();
            telemetry.addData("LeftRed",sensorRGBleft.red());
            telemetry.addData("LeftBlue",sensorRGBleft.blue());
            telemetry.addData("RightRed",sensorRGBright.red());
            telemetry.addData("RightBlue",sensorRGBright.blue());
            telemetry.update();
        }

    }

}
