package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Ryley on 12/11/16.
 */
@Autonomous(name="CF_AutoTest", group="Test")
//@Disabled
public class CF_AutoTest extends CF_Library{
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        ColorSensor sensorRGBright;
        ColorSensor sensorRGBleft;

        robot.MotorMecanumLeftFront.setDirection(DcMotor.Direction.FORWARD);     // Set to REVERSE if using AndyMark motors
        robot.MotorMecanumLeftRear.setDirection(DcMotor.Direction.FORWARD);      // Set to REVERSE if using AndyMark motors
        robot.MotorMecanumRightFront.setDirection(DcMotor.Direction.REVERSE);    // Set to FORWARD if using AndyMark motors
        robot.MotorMecanumRightRear.setDirection(DcMotor.Direction.REVERSE);

        sensorRGBright = hardwareMap.colorSensor.get("AdafruitRGBright");
        sensorRGBleft = hardwareMap.colorSensor.get("AdafruitRGBleft");
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("left", sensorRGBleft.red());
            telemetry.addData("right", sensorRGBright.red());
            telemetry.update();
        }
    }

}
