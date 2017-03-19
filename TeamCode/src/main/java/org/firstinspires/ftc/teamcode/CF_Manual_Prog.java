package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
import com.vuforia.ObjectTracker;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Crossfire_Hardware.sensorColor;
import org.firstinspires.ftc.robotcore.internal.VuforiaTrackablesImpl;

/***
 * This file provides basic Telop driving for a robot with mecanum wheels.
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

@TeleOp(name = "CF_Manual_Prog", group = "Drivetrain")
//@Disabled

public class CF_Manual_Prog extends OpMode
{
    Crossfire_Hardware_Proggie robot = new Crossfire_Hardware_Proggie();

    // Joystick threshold sets a minimum value for the controller joysticks
// to reach before the robot will begin to move.
    private static final float joystickThreshold = 0.003f;

    //    When both of the joysticks used for driving are held at the same time, the
//    robot cannot fully do both functions. Priority sets the team's driving
//    priority - whether strafing, turning, or driving straight should have more
//    importance, causing to robot to perform more of one action than of the others.
    private static final float forwardPriority = 1.0f;
    private static final float strafePriority = 1.0f;
    private static final float steerPriority = 1.0f;
    private static final float forwardGain_Scoop = 1.0f;
    private static final float strafeGain_Scoop = 1.0f;
    private static final float steerGain_Scoop = 1.0f;
    private static final float forwardGain_Lifter = 0.5f;
    private static final float strafeGain_Lifter = 1.0f;
    private static final float steerGain_Lifter = 0.5f;

    // Beacon button pusher servo increment rate
    private static final double beaconPusherRate = 0.005;

    private sensorColor beaconColor = sensorColor.unknown;
    int pictureNumber;

    int state = 0;
    float shooterPower = -0.28f;
    boolean firstLastRightButton = false;
    boolean firstButtonRight = false;
    boolean firstLastLeftButton = false;
    boolean firstButtonLeft = false;

    boolean secondLastRightButton = false;
    boolean secondButtonRight = false;

    //these variables set positions for the Loader servo.
    float basePos = 0.0f;
    float Pos = basePos;

    // These variables are for the Vuforia navigation
    OpenGLMatrix pose = null;

    final int stopCount = 200;
    int pic = -1;
    boolean seeable;
    double kP = 0.0025;
    double power = 0.5;
    double effort;
    int error;
    double leftPower;
    double rightPower;
    boolean spinnerFlag = false;
    boolean shooterFlag = false;
    VuforiaTrackables beacons;

    boolean runShooter = false;

    ColorSensor sensorRGBright;
    ColorSensor sensorRGBleft;

    float hsvValuesright[] = {0F, 0F, 0F};
    float hsvValuesleft[] = {0F, 0F, 0F};

    VectorF translation;

    /***
     *This method inits all hardware on the robot as well as the camera on the drivers'
     * phone.
     */
    public void init()
    {
        //This line inits all robot hardware
        robot.init(hardwareMap);
    }


    /***
     * Method is the main loop for manual opmode.  Things in this method are iteratively
     * run until the stop button is pushed.
     */
    public void loop()
    {
        // Calculate and apply motor power to drive wheels
        RunMecanumWheels();


        telemetry.addData("RightFront:", robot.MotorMecanumRightFront.getCurrentPosition());
        telemetry.update();
    }


    /***
     * This method calculates the individual motor powers required to drive the mecanum
     * wheels based off the driver 1 controller.  This drive strategy uses the following
     * joystick assignments
     * <p/>
     * Left stick:
     * forward (+y)   - Forward drive
     * rearward (-y)  - Reverse drive
     * right (+x)     - Strafe right
     * left (-x)      - Strafe left
     * <p/>
     * Right stick:
     * forward (+y)   - Not used
     * rearward (-y)  - Not used
     * right (+x)     - Tank turn right
     * left (-x)      - Tank turn left
     */
    public void RunMecanumWheels()
    {
        double LFPower = 0.0;
        double RFPower = 0.0;
        double LRPower = 0.0;
        double RRPower = 0.0;
        double leftStickY;
        double leftStickX;
        double rightStickX;
        // Calculate motor powers but only if any of the joystick commands are greater then
        // a minimum threshold.  Adjust this threshold if the motor has motion when the joystick
        // is not being used and in the center position.
        if ((Math.abs(gamepad1.left_stick_y) >= joystickThreshold) ||
                (Math.abs(gamepad1.left_stick_x) >= joystickThreshold) ||
                (Math.abs(gamepad1.right_stick_x) >= joystickThreshold))
        {
            leftStickY = robot.ScaleJoystickCommand(gamepad1.left_stick_y);
            leftStickX = robot.ScaleJoystickCommand(gamepad1.left_stick_x);
            rightStickX = robot.ScaleJoystickCommand(gamepad1.right_stick_x);


            // Calculate power for each mecanum wheel based on joystick inputs.  Each power is
            // based on three drive components: forward/reverse, strafe, and tank turn.
            //There are three drive modes. This mode, Beacon, drives the robot with the beacon
            //pusher servo in the front.
            telemetry.addData("Mode: ", "Beacon");
            if (robot.driveMode == Crossfire_Hardware_Proggie.driveModeEnum.beaconMode)
            {
                LFPower = (forwardPriority * leftStickY) + (strafePriority * leftStickX) - (steerPriority * rightStickX);
                RFPower = (forwardPriority * leftStickY) - (strafePriority * leftStickX) + (steerPriority * rightStickX);
                LRPower = (forwardPriority * leftStickY) - (strafePriority * leftStickX) - (steerPriority * rightStickX);
                RRPower = (forwardPriority * leftStickY) + (strafePriority * leftStickX) + (steerPriority * rightStickX);
            }

            //The strafe drive mode sets the side of the robot with the cap ball lifter
            //as the front. To drive forward, the robot strafes.
            telemetry.addData("Mode: ", "Strafe");
            if (robot.driveMode == Crossfire_Hardware_Proggie.driveModeEnum.ballLifterMode)
            {
                LFPower = (forwardGain_Lifter * -leftStickX) - (strafeGain_Lifter * -leftStickY) + (steerGain_Lifter * -rightStickX);
                RFPower = (forwardGain_Lifter * -leftStickX) + (strafeGain_Lifter * -leftStickY) - (steerGain_Lifter * -rightStickX);
                LRPower = (forwardGain_Lifter * -leftStickX) + (strafeGain_Lifter * -leftStickY) + (steerGain_Lifter * -rightStickX);
                RRPower = (forwardGain_Lifter * -leftStickX) - (strafeGain_Lifter * -leftStickY) - (steerGain_Lifter * -rightStickX);
                telemetry.addData("leftStickX", leftStickX);
            }

            //The scoop drive mode sets the particle ball gatherer as the front of the robot.
            telemetry.addData("Mode: ", "Scoop");
            if (robot.driveMode == Crossfire_Hardware_Proggie.driveModeEnum.scooperMode)
            {
                LFPower = (forwardGain_Scoop * -leftStickY) - (strafeGain_Scoop * leftStickX) + (steerGain_Scoop * -rightStickX);
                RFPower = (forwardGain_Scoop * -leftStickY) + (strafeGain_Scoop * leftStickX) - (steerGain_Scoop * -rightStickX);
                LRPower = (forwardGain_Scoop * -leftStickY) + (strafeGain_Scoop * leftStickX) + (steerGain_Scoop * -rightStickX);
                RRPower = (forwardGain_Scoop * -leftStickY) - (strafeGain_Scoop * leftStickX) - (steerGain_Scoop * -rightStickX);
            }

            // Find maximum power commanded to all the mecanum wheels.  Using the above power
            // equations, it is possible to calculate a power command greater than 1.0f (100%).
            // We want to find the max value so we can proportionally reduce motor powers.
            double maxPower = Math.max(LFPower, Math.max(RFPower, Math.max(LRPower, RRPower)));

            // If max power is greater than 1.0f (100% command), then proportionally reduce all motor
            // powers by the maximum power calculated.  This will equally reduce all powers so no
            // motor power is clipped and the robot responds predictably to joystick commands.  If we
            // don't, then one or more motor commands will clip, others will not, and the robot will not
            // behave predictably.  The end result of this reduction is the motor requesting max power
            // will set power to 1.0f (100%) and all other powers will be reduced by the same ratio.
            if (Math.abs(maxPower) > 1.0f)
            {
                LFPower /= maxPower; // Shorthand for LFPower = LFPower / maxPower
                RFPower /= maxPower;
                LRPower /= maxPower;
                RRPower /= maxPower;
            }

            // Update motor powers with new value.
            robot.setMecanumPowers(LFPower, RFPower, LRPower, RRPower);
        }
        else
        {
            // Explicitly set powers to zero.  May not be necessary but is good practice.
            robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);
        }
    }

    // Checks for a requested stop
    private boolean checkForStop()
    {

        if(gamepad1.back)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // This is a method to drive to a certain encoder count
    public void encoderMove(int countLeft, int countRight, double leftPower, double rightPower)
    {
        robot.MotorMecanumLeftFront.setPower(leftPower);
        robot.MotorMecanumRightFront.setPower(rightPower);
        robot.MotorMecanumLeftRear.setPower(leftPower);
        robot.MotorMecanumRightRear.setPower(rightPower);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.MotorMecanumLeftFront.setTargetPosition(countLeft);
        robot.MotorMecanumRightFront.setTargetPosition(countRight);
        robot.MotorMecanumLeftRear.setTargetPosition(countLeft);
        robot.MotorMecanumRightRear.setTargetPosition(countRight);

        while(robot.MotorMecanumLeftFront.isBusy() && robot.MotorMecanumRightFront.isBusy() && robot.MotorMecanumLeftRear.isBusy() && robot.MotorMecanumRightRear.isBusy())
        {
            double leftPos = robot.MotorMecanumLeftFront.getCurrentPosition();
            double rightPos = robot.MotorMecanumRightFront.getCurrentPosition();
            telemetry.addData("Right",rightPos);
            telemetry.addData("Left",leftPos);
            try
            {
                idle();
            }
            catch(InterruptedException e)
            {
                telemetry.addData("Idle Failed", "Idle Failed");
            }

        }
        setPower(0.0f);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    // Sets the mode of the drive motors
    public void setMode(DcMotor.RunMode mode)
    {
        robot.MotorMecanumLeftFront.setMode(mode);
        robot.MotorMecanumRightFront.setMode(mode);
        robot.MotorMecanumLeftRear.setMode(mode);
        robot.MotorMecanumRightRear.setMode(mode);
    }


    // Sets the power of the drive motors
    public void setPower(float power)
    {
        robot.MotorMecanumLeftFront.setPower(power);
        robot.MotorMecanumRightFront.setPower(power);
        robot.MotorMecanumLeftRear.setPower(power);
        robot.MotorMecanumRightRear.setPower(power);
    }


    // Idles
    public final void idle() throws InterruptedException
    {
        // Abort the OpMode if we've been asked to stop
        if (checkForStop())
            throw new InterruptedException();

        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }
}
