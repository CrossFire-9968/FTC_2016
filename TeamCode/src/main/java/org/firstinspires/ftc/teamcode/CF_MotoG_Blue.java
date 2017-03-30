package org.firstinspires.ftc.teamcode;

import android.hardware.SensorManager;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.TimeUnit;

/**
 * Created by Ryley on 12/22/16.
 */
@Autonomous(name="CF_MotoG_Blue", group ="Blue")
//@Disabled
public class CF_MotoG_Blue extends CF_Library_Test{
    // Position
    OpenGLMatrix pose;
    VectorF translation = null;

    // This is how far from the picture to stop
    final int stopCount = 200;
    boolean breakLoop = false;

    // If x is smaller than stopCount, it doesn't work, so we instantiate
    // it to a value greater than stopCount
    int x = stopCount + 100;
    int y;

    // Gains for the PID controllers.  Only one is in use currently
    double kPy = 0.00097;
    double kPangle = 0.0095;
    double kPangleSmall = 0.0009;
    double kIangle = 0.000085;
    double kIangleBig = 0.00023;
    double integral = 0.0;
    float effortAngle;
    int angle;
    int errorY;
    double errorAngle;
    int ySquare;
    double turnFront;
    double turnRear;

    // This matrix holds the data for the Vuforia matrix
    float[] data;

    // Picture constants
    int FIRSTPICTURE = 0;
    int SECONDPICTURE = 2;

    int beaconFlagFirst = 0;

    // Color sensor variables
    ColorSensor sensorRGBright;
    ColorSensor sensorRGBleft;

    // Enumeration for the state machine
    private enum driveState {
        FIRSTSTRAFE, SQUARETOFIRSTBEACON, BALLONE, BALLTWO, DRIVETOFIRSTBEACON, PUSHFIRSTBEACON, SECONDSTRAFE, DRIVETOSECONDBEACON, PUSHSECONDBEACON, END
    }

    // Timer so the robot will stop when 30 seconds are elapsed
    private ElapsedTime runtime = new ElapsedTime();

    // Number of seconds for the total time of the auto mode
    final int endTime = 29;

    BNO055IMU imu;
    Orientation ang;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init robot object
        robot.init(hardwareMap);

        // Set drive motor direction
        // This is a default speed
        final float speed = 0.8f;

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

        // Instantiates IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Instantiates color sensors
        sensorRGBright = hardwareMap.colorSensor.get("AdafruitRGBright");
        sensorRGBleft = hardwareMap.colorSensor.get("AdafruitRGBleft");

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Instantiates switch enumeration
        driveState State = driveState.FIRSTSTRAFE;

        // Run initializing routine
        beacons.activate();

        // Run initializing routine
        initalize();

        // Reset runtime
        runtime.reset();
        while(opModeIsActive() && !breakLoop && runtime.seconds() < endTime) {
            switch(State) {
                case FIRSTSTRAFE:
                    // Runs the check time method
                    checkTime();

                    System.out.println("FIRST STRAFE");
                    // This is to get the robot more or less lined up with the picture
                    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    this.encoderStrafeLeftNew(2650, speed, imu);
                    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    this.encoderMoveNew(1100, 0.7f, imu);
                    //this.encoderMove(1100, 1100, speed, speed);
                    robot.Shooter.setPower(-0.32f);
                    //robot.Shooter.setPower(-0.3f);
                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.encoderStrafeLeftNew(2400, speed, imu);
                    System.out.println("DONE STRAFING");

                    // Increment the state to the next state
                    State = driveState.BALLONE;

                    break;
                case SQUARETOFIRSTBEACON:
                    // This is a currently unused method
                    // Runs the check time method
                    checkTime();

                    // Wait to acquire the picture
                    TimeUnit.SECONDS.sleep(2);
                    // Gets position
                    pose = ((VuforiaTrackableDefaultListener) beacons.get(2).getListener()).getRawPose();

                    if(pose != null) {
                        // This runs the pid controller
                        translation = pose.getTranslation();
                        data = pose.getData();
                        angle = (int) Math.toDegrees(Math.acos(data[2]));

                        ySquare = (int) translation.get(1);

                        errorY = ySquare;

                        // Multiplies the gain by the error to get the effort
                        turnRear = kPy * errorY;
                        turnFront = kPy * errorY * -1;

                        errorAngle = 90 - angle;

                        // Integral
                        if(errorAngle < 10 && errorAngle > -10) {
                            integral = 0;
                        } else {
                            integral = integral + errorAngle;
                        }
                        if(integral * kIangle > 1.0) {
                            integral = 1000;
                        }

                        if(x < 700) {
                            effortAngle = (float) ((errorAngle * kPangleSmall) + (integral * kIangleBig));
                        }
                        else{
                            effortAngle = (float) ((errorAngle * kPangle) + (integral * kIangle));
                        }

                        if((errorAngle < 10 || -10 < errorAngle) && (errorY < 10 || -10 < errorY)) {
                            setPower(0.0f);
                            State = driveState.BALLONE;
                        }else {
                            strafe(effortAngle + turnFront, effortAngle + turnRear);
                        }
                        telemetry.addData("error", errorY);
                        telemetry.addData("turnRear", turnRear);
//                        telemetry.update();
                    }
                    if(pose == null) {
                        State = driveState.BALLONE;
                    }
                    pose = null;
                    break;
                case BALLONE:
                    // Runs checkTime method
                    checkTime();
                    TimeUnit.MILLISECONDS.sleep(1000);
                    System.out.println("BALL ONE");
                    // Increment the first ball
                    robot.SetLoaderPosition(0.015);
                    TimeUnit.SECONDS.sleep(2);
                    State = driveState.BALLTWO;
                    break;
                case BALLTWO:
                    // Runs checkTime method
                    checkTime();
                    // Increment the second ball
                    robot.SetLoaderPosition(0.0f);
                    TimeUnit.MILLISECONDS.sleep(500);
                    // Turns off the shooter
                    robot.Shooter.setPower(0.0f);
                    System.out.println("DONE SHOOTING");
                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    State = driveState.DRIVETOFIRSTBEACON;
                    break;
                case DRIVETOFIRSTBEACON:
                    // Runs the checkTime method
                    checkTime();
                    System.out.println("DRIVE TO FIRST BEACON");

                    // Get pose
                    robot.SetButtonPusherPosition(0.45f);
                    pose = ((VuforiaTrackableDefaultListener) beacons.get(FIRSTPICTURE).getListener()).getRawPose();

                    if(pose != null && x > stopCount) {
                        System.out.println("SEEABLE");
                        // Gets the translation matrix out of the position matrix
                        translation = pose.getTranslation();
                        // Gets the useful elements out of the translation matrix
                        y = (int) translation.get(1);
                        x = (int) translation.get(2);
                        System.out.println("X: " + x);
                        telemetry.addData("x: ", x);
                        //telemetry.update();

                        // Drive to beacon using PID controller
                        System.out.println("PID DRIVE");
                        // This runs the PID controller
                        pidDrive(y);
                    }

                    // Stops under certain conditions
                    if(pose == null) {
                        // Stops if can't see the picture
                        setPower(0.0f);
                    }
                    if(x <= stopCount) {
                        // Stops if close enough to the picture
                        setPower(0.0f);
                        State = driveState.PUSHFIRSTBEACON;

                        //breakLoop = true;
                    }
                    break;
                case PUSHFIRSTBEACON:
                    // Runs the checkTime method
                    checkTime();
                    System.out.println("PUSH FIRST BEACON");
                    // Push the beacon button
                    pushBeaconButton();
                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    // Backs up after pushing the button
                    this.encoderMoveNew(1800, (speed * -1), imu);
                    State = driveState.SECONDSTRAFE;
                    //breakLoop = true;
                    break;
                case SECONDSTRAFE:
                    // Runs the checkTime method
                    checkTime();
                    System.out.println("SECOND STRAFE");
                    // Move close to the second picture
                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    // Strafes left to the next beacon
                    encoderStrafeLeftNew(4950, speed, imu);
                    //encoderStrafeLeftDualPower(3750, 0.7f, 1000, 0.4f);
                    State = driveState.DRIVETOSECONDBEACON;
                    x = stopCount + 10;
                    setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    break;
                case DRIVETOSECONDBEACON:
                    // Runs the checkTime method
                    checkTime();
                    System.out.println("DRIVE TO SECOND BEACON");
                    // Get pose and translation
                    robot.SetButtonPusherPosition(0.45f);
                    //setMode(norm);

                    // Gets the position matrix
                    pose = ((VuforiaTrackableDefaultListener) beacons.get(SECONDPICTURE).getListener()).getRawPose();
//                    telemetry.clearAll();
//                    telemetry.addData("x", x);
//                    telemetry.addData("StopCount:", stopCount);
//                    telemetry.update();
                    if(pose != null && x > stopCount) {
                        telemetry.clearAll();
                        System.out.println("SEEABLE");
                        // Gets the transation matrix out of the pose matrix
                        translation = pose.getTranslation();
                        // Gets the elements out of the translation matrix
                        y = (int) translation.get(1);
                        x = (int) translation.get(2);
                        System.out.println("X: " + x);
                        telemetry.addData("x: ", x);
                        //telemetry.update();

                        // Drive to beacon using PID controller
                        System.out.println("PID DRIVE");
                        // Runs the PID controller
                        pidDrive(y);
                    }

                    // Stop if close enough to picture
                    if(pose == null) {
                        setPower(0.0f);
                    }
                    if(x <= stopCount) {
                        setPower(0.0f);
                        State = driveState.PUSHSECONDBEACON;

                        //breakLoop = true;
                    }
                    break;
                case PUSHSECONDBEACON:
                    // Runs the checkTime method
                    checkTime();
                    System.out.println("PUSH SECOND BEACON");
                    TimeUnit.MILLISECONDS.sleep(10);
                    // Push the beacon button
                    pushBeaconButton();
                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    // Moves backward after pushing the beacon
                    this.encoderMoveNew(500, -0.4f, imu);
                    State = driveState.END;
                    break;
                case END:
                    checkTime();
                    System.out.println("END");
                    setPower(0.0f);
                    breakLoop = true;
                    break;
                default:
                    checkTime();
                    System.out.println("DEFAULT");
                    // Turn on the boolean that breaks out of the loop
                    breakLoop = true;
                    break;
            }
            System.out.println("TIME: " + runtime.seconds());
            if(isStopRequested()) {
                setPower(0.0f);
            }
            checkTime();
//            telemetry.update();
        }
        // Stop op mode when the loop has been broken out of
        requestOpModeStop();




    }

    private void checkTime() {
        // Kills the robot if time is over the endTime
        if(runtime.seconds() >= endTime) {
            breakLoop = true;
            requestOpModeStop();
        }
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
        double kP = 0.0015; // 0.00045 // 0.00090
        double power = 0.15;
        double effort;
        double leftPower;
        double rightPower;
        // Make the error equal to the y distance
        error = yDist;
        // Multiply the error by our gain to get a control effort
        effort = kP * error;
        // Set the right and left powers by subtracting or adding the control effort from a base power
        rightPower = power + effort;
        leftPower = power - effort;
        // Set powers
        robot.MotorMecanumLeftFront.setPower(leftPower);
        robot.MotorMecanumRightFront.setPower(rightPower);
        robot.MotorMecanumLeftRear.setPower(leftPower);
        robot.MotorMecanumRightRear.setPower(rightPower);
    }
    // Pushes the correct beacon button depending on the color
    public void pushBeaconButton() throws InterruptedException{
        //if (beaconFlagFirst == 0) {
        TimeUnit.MILLISECONDS.sleep(750);
        telemetry.clearAll();
        telemetry.addData("Blue Right", sensorRGBright.blue());
        telemetry.addData("Red Right", sensorRGBright.red());
        telemetry.addData("Blue Left", sensorRGBleft.blue());
        telemetry.addData("Red Left", sensorRGBleft.red());
        telemetry.update();
        // Checks if the blue is on the right and red is on the left
        if ((sensorRGBright.blue() > sensorRGBright.red()) && (sensorRGBleft.red() > sensorRGBleft.blue())) {
//            telemetry.update();
            // Sets the button pusher servo position
            robot.SetButtonPusherPosition(0.00);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TimeUnit.SECONDS.sleep(1);
            this.encoderMove(300, 300, 0.2f, 0.2f);
            beaconFlagFirst = 1;
            TimeUnit.MILLISECONDS.sleep(250);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int count = 100;
            while((sensorRGBleft.red() > sensorRGBleft.blue()) && (count <= 160)) {
                this.encoderMove(count, count, 0.3f, 0.3f);
                count+=50;
            }
            //requestOpModeStop();
            // Checks if the red is on the right and the blue is on the left
        } else if ((sensorRGBright.red() > sensorRGBright.blue()) && (sensorRGBleft.blue() > sensorRGBleft.red())) {
            // telemetry.update();
            // Sets the button pusher servo position
            robot.SetButtonPusherPosition(0.90);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TimeUnit.SECONDS.sleep(1);
            this.encoderMove(300, 300, 0.2f, 0.2f);
            beaconFlagFirst = 1;
            TimeUnit.MILLISECONDS.sleep(250);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int count = 100;
            while((sensorRGBright.red() > sensorRGBright.blue()) && (count <= 160)){
                this.encoderMove(count, count, 0.3f, 0.3f);
                count+=50;
            }
        } //else {
//            telemetry.addData("unknown", "unknown");
//            //telemetry.update();
//        }
        //}
    }
}
