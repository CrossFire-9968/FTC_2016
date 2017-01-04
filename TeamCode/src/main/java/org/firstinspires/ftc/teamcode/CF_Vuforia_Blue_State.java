package org.firstinspires.ftc.teamcode;

import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.TimeUnit;

/**
 * Created by Ryley on 12/22/16.
 */
@Autonomous(name="CF_Vuforia_Blue_State", group ="Blue")
//@Disabled
public class CF_Vuforia_Blue_State extends CF_Library{
    OpenGLMatrix pose = null;
    VectorF translation = null;

    final int stopCount = 110;
    boolean breakLoop = false;
    int x = stopCount + 100;
    int y;

    final int FIRSTPICTURE = 2; // swap back
    final int SECONDPICTURE = 0;
    int beaconFlagFirst = 0;

    ColorSensor sensorRGBright;
    ColorSensor sensorRGBleft;

    private SensorManager sensorManager;

    private enum driveState {
        FIRSTSTRAFE, DRIVETOFIRSTBEACON, PUSHFIRSTBEACON, SECONDSTRAFE, DRIVETOSECONDBEACON, PUSHSECONDBEACON
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.MotorMecanumLeftFront.setDirection(DcMotor.Direction.FORWARD);
        robot.MotorMecanumLeftRear.setDirection(DcMotor.Direction.FORWARD);
        robot.MotorMecanumRightFront.setDirection(DcMotor.Direction.REVERSE);
        robot.MotorMecanumRightRear.setDirection(DcMotor.Direction.REVERSE);

        final float speed = 0.5f;


        // Instantiates a paramaters file for Vuforia
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        // Sets camera direction
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        // License key
        params.vuforiaLicenseKey = "AU2Vspr/////AAAAGSWZlF6AQEHFh9mbNlt5KlFGl/PX8qeeKea7jh5Xk8Ei573/nsoAjsJu9Cbi2MlRCuEIkZHQJoDGAxXmNgioA+0+DbRC6mG+1QbBu8ACMw0pBk6x3h+wvvqDeyZmjV0Fdji5Bk2bV3AaZ0AanljM2nuosjfFYOeUsoFqjE0+MQfJCOoG2ED2hxhJM88dhMaAH45kQqJ99Pn9c/F8whHUkRLeh71wW3O8qGdHEieX7WQO86VfVadHTrg0Ut8ALwiU/qVqB9pJPn+oVe9rYCixcJztb7XOp4T4Mo0IPUwVtkTUZtZTW1mAOPdbbWx3RX1OohA6q6BBU7ozDdQ1W33/L/mdETevYMf7rKPrb82Zbw8r";
        // Sets the thing you see on the screen.  Coule be AXES, TEAPOT, BUILDINGS, or NONE
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        // Creates a new instance of Vuforia named vuforia(lower case) with parameters = params
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        // Lets Vuforia see more than one thing at a time
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        // Gets a beacon class and puts the pictures into it
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        // Instantiates color sensors
        sensorRGBright = hardwareMap.colorSensor.get("AdafruitRGBright");
        sensorRGBleft = hardwareMap.colorSensor.get("AdafruitRGBleft");

        // Instantiates switch enumeration
        driveState State = driveState.FIRSTSTRAFE;

        // Run initializing routine
        initalize();

        while(opModeIsActive() && !breakLoop) {
            switch(State) {
                case FIRSTSTRAFE:
                    System.out.println("FIRST STRAFE");
                    // This is to get the robot more or less lined up with the picture
                    this.encoderStrafeLeft(3000, speed);
                    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.encoderMove(1500, 1500, speed, speed);
                    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.encoderStrafeLeft(2000, speed);

                    // Sleep to give the robot time to see the picture
                    TimeUnit.SECONDS.sleep((long)0.5);

                    // Increment the state to the next state
                    if(robot.MotorMecanumLeftFront.getPower() == 0.0) {
                        State = driveState.DRIVETOFIRSTBEACON;
                    }
                    break;
                case DRIVETOFIRSTBEACON:
                    // Get pose and translation
                    System.out.println("DRIVE TO FIRST BEACON");
                    pose = ((VuforiaTrackableDefaultListener) beacons.get(FIRSTPICTURE).getListener()).getRawPose();

                    if(pose != null && x > stopCount) {
                        translation = pose.getTranslation();
                        y = (int) translation.get(1);
                        x = (int) translation.get(2);

                        // Drive to beacon using PID controller
                        pidDrive(y);
                    }

                    // Stop if close enough to picture
                    if(x <= stopCount) {
                        //State = driveState.PUSHFIRSTBEACON;
                        setPower(0.0f);
                        breakLoop = true;
                    }
                    break;
                case PUSHFIRSTBEACON:
                    System.out.println("PUSH FIRST BEACON");
                    // Push the beacon button
                    pushBeaconButton();
                    State = driveState.SECONDSTRAFE;
                    break;
                case SECONDSTRAFE:
                    System.out.println("SECOND STRAFE");
                    // Move close to the second picture
                    encoderMove(-1500, -1500, 0.2f, 0.2f);
                    encoderStrafeLeft(4500, speed);
                    State = driveState.DRIVETOSECONDBEACON;
                    break;
                case DRIVETOSECONDBEACON:
                    System.out.println("DRIVE TO SECOND BEACON");
                    // Get pose and translation
                    pose = ((VuforiaTrackableDefaultListener) beacons.get(SECONDPICTURE).getListener()).getRawPose();

                    if(pose != null && x > stopCount) {
                        translation = pose.getTranslation();
                        x = (int) translation.get(2);
                        y = (int) translation.get(1);

                        // Drive to beacon using PID controller
                        pidDrive(y);
                    }

                    // Stop if close enough to picture
                    if( x <= stopCount) {
                        State = driveState.PUSHSECONDBEACON;
                        break;
                    }
                case PUSHSECONDBEACON:
                    System.out.println("PUSH SECOND BEACON");
                    // Push the beacon button
                    pushBeaconButton();
                    break;
                default:
                    System.out.println("DEFAULT");
                    // Turn on the boolean that breaks out of the loop
                    breakLoop = true;
                    break;
            }
            telemetry.update();
        }
        // Stop op mode when the loop has been broken out of
        requestOpModeStop();




    }

    private void initalize() throws InterruptedException {
        // Sends telemetry message to signify robot waiting
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // Reset motor encoders
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run using encoders
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Sends telemetry message to signify encoder reset
        if(robot.MotorMecanumLeftFront.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            telemetry.addData("Encoder Reset", "Encoder Reset");
            telemetry.update();
        } else {
            telemetry.addData("Encoder NOT Reset", "Encoder NOT Reset");
        }

        // Wait for start button to be pressed
        waitForStart();
    }

    private void pidDrive(int yDist) {
        int error;

        double kP = 0.0005;
        double power = 0.2;
        double effort;
        double leftPower;
        double rightPower;
        // Make the error equal to the y distance
        error = yDist;
        // Multiply the error by our gain to get a control effort
        effort = kP * error;
        // Set the right and left powers by subtracting or adding the control effort from a base power
        leftPower = power + effort;
        rightPower = power - effort;
        // Set powers
        robot.MotorMecanumLeftFront.setPower(leftPower);
        robot.MotorMecanumRightFront.setPower(rightPower);
        robot.MotorMecanumLeftRear.setPower(leftPower);
        robot.MotorMecanumRightRear.setPower(rightPower);
    }
    public void pushBeaconButton() throws InterruptedException{
        //if (beaconFlagFirst == 0) {
        if (sensorRGBright.blue() > sensorRGBright.red() && sensorRGBleft.red() > sensorRGBleft.blue()) {
            telemetry.update();
            robot.SetButtonPusherPosition(0.00);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TimeUnit.SECONDS.sleep(1);
            this.encoderMove(300, 300, 0.2f, 0.2f);
            beaconFlagFirst = 1;
            TimeUnit.SECONDS.sleep(1);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int count = 100;
            while(sensorRGBleft.red() > sensorRGBleft.blue() && count <= 600) {
                this.encoderMove(count, count, 0.3f, 0.3f);
                count+=50;
            }
            this.encoderMove(-1500, -1500, 0.2f, 0.2f);
            //requestOpModeStop();
        } else if (sensorRGBright.red() > sensorRGBright.blue() && sensorRGBleft.blue() > sensorRGBleft.red()) {
            telemetry.update();
            robot.SetButtonPusherPosition(0.90);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TimeUnit.SECONDS.sleep(1);
            this.encoderMove(300, 300, 0.2f, 0.2f);
            beaconFlagFirst = 1;
            TimeUnit.SECONDS.sleep(1);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int count = 100;
            while(sensorRGBright.red() > sensorRGBright.blue() && count <= 600){
                this.encoderMove(count, count, 0.3f, 0.3f);
                count+=50;
            }
            this.encoderMove(-1500, -1500, 0.2f, 0.2f);
            //requestOpModeStop();
        } else {
            telemetry.addData("unknown", "unknown");
            telemetry.update();
        }
        //}
    }
}
