package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Crossfire_Hardware.sensorColor;

/***
 * This file provides basic Telop driving for a robot with mecanum drive
 * The code is structured as an Iterative OpMode
 * <p/>
 * This OpMode uses the CrossFire hardware class to define the devices on the robot.
 * All device access is managed through the Crossfire_Hardware class.
 * <p/>
 * This OpMode takes joystick values from three independent axis and computes a
 * desired motor power for each of the mecanum drive motors (one per wheel) to
 * attain desired velocity, direction, and rotation of robot. If the calculated
 * desired power for any motor exceeds the maximum power limit (1.0F), then all
 * motor powers are proportionally reduced.  We do this to retain consistent
 * driving characteristics at the expense of vehicle speed and total power.
 */

@TeleOp(name = "CF_CRServo_Test", group = "Drivetrain")
//@Disabled

public class CF_CRServo_Test extends OpMode
{
    Crossfire_Hardware_Test robot = new Crossfire_Hardware_Test();
    float posLeft = 0.5f;
    float posRight = 0.5f;
    // Beacon button pusher servo increment rate
    private static final double beaconPusherRate = 0.005;

    public void init()
    {
        robot.init(hardwareMap);
    }


    /***
     * Method is the main loop for manual opmode.  Things in this method are iteratively
     * run until the stop button is pushed.
     */
    public void loop()
    {
        // Adjust the beacon button servo
        ServiceServos();
        //telemetry.addData("pos: ", robot.Loader.getPosition());
        telemetry.update();
        //robot.Loader.setPosition(1.0f);

    }


    private void ServiceServos()
    {        //Unknown
        if(gamepad2.x) {
            posLeft += 0.03;
        }
        if(gamepad2.b) {
            posRight -= 0.03;
        }
        if(gamepad2.y) {
            posLeft -= 0.03;
        }
        if(gamepad2.a) {
            posRight += 0.03;
        }
        telemetry.addData("Left: ", posLeft);
        telemetry.addData("Right", posRight);
        telemetry.update();
        if (gamepad2.dpad_left)
        {
            //robot.Loader.setPower(-0.2f);
            robot.Loader.setPosition(0.10f);
        }

        //Also unknown
        if (gamepad2.dpad_up)
        {
            //robot.Loader.setPower(0.00f)
            robot.Loader.setPosition(0.00f);
        }

        //Also unknown. What a surprise.
        if (gamepad2.dpad_right)
        {
            //robot.Loader.setPower(0.2f);
            robot.Loader.setPosition(0.12f);
        }
        telemetry.addData("Position", robot.Loader.getPosition());
    }


}
