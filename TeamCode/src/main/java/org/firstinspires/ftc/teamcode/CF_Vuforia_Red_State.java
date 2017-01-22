package org.firstinspires.ftc.teamcode;

import android.hardware.SensorManager;

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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.TimeUnit;

/**
 * Created by Ryley on 12/22/16.
 */
@Autonomous(name="CF_Vuforia_Red_State", group ="Red")
//@Disabled
public class CF_Vuforia_Red_State extends CF_Library{
    OpenGLMatrix pose;
    VectorF translation = null;

    final int stopCount = 120;
    boolean breakLoop = false;
    int x = stopCount + 100;
    int y;

    double kPy = 0.00097;

    double kPangle = 0.0095;
    double kPangleSmall = 0.0009;
    double kIangle = 0.000085;
    double kIangleBig = 0.00023;

    double integral = 0.0;

    float effortAngle;

    float[] data;

    int angle;
    int errorY;
    double errorAngle;

    int ySquare;

    double turnFront;
    double turnRear;


    int FIRSTPICTURE = 3;
    int SECONDPICTURE = 1;

    int beaconFlagFirst = 0;

    ColorSensor sensorRGBright;
    ColorSensor sensorRGBleft;

    private SensorManager sensorManager;

    private enum driveState {
        FIRSTSTRAFE, SQUARETOFIRSTBEACON, BALLONE, BALLTWO, DRIVETOFIRSTBEACON, PUSHFIRSTBEACON, SECONDSTRAFE, DRIVETOSECONDBEACON, PUSHSECONDBEACON, END
    }

    private ElapsedTime runtime = new ElapsedTime();

    final int endTime = 29;


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

        beacons.activate();

        DcMotor.RunMode norm = robot.MotorMecanumLeftFront.getMode();

        initalize();

        runtime.reset();
        while(opModeIsActive() && !breakLoop && runtime.seconds() < endTime) {
            switch(State) {
                case FIRSTSTRAFE:
                    checkTime();
                    System.out.println("FIRST STRAFE");
                    // This is to get the robot more or less lined up with the picture
                    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.encoderStrafeRight(3000, speed);
                    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //robot.Shooter.setPower(-1.0f);
                    this.encoderMove(1500, 1500, 0.6f, 0.6f);
                    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.Shooter.setPower(-0.32f);
                    this.encoderStrafeRight(1900, speed);
                    System.out.println("DONE STRAFING");
                    // Sleep to give the robot time to see the picture

                    //TimeUnit.SECONDS.sleep((long)0.5);

                    // Increment the state to the next state
                    State = driveState.SQUARETOFIRSTBEACON;

                    break;
                case SQUARETOFIRSTBEACON:
                    checkTime();
                    pose = ((VuforiaTrackableDefaultListener) beacons.get(2).getListener()).getRawPose();
                    TimeUnit.SECONDS.sleep(2);
                    if(pose != null) {
                        translation = pose.getTranslation();
                        data = pose.getData();
                        angle = (int) Math.toDegrees(Math.acos(data[2]));

                        ySquare = (int) translation.get(1);

                        errorY = ySquare;

                        turnRear = kPy * errorY;
                        turnFront = kPy * errorY * -1;

                        errorAngle = 90 - angle;

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
                    checkTime();
                    System.out.println("BALL ONE");
                    robot.SetLoaderPosition(0.015);
                    TimeUnit.SECONDS.sleep(2);
                    State = driveState.BALLTWO;
                    break;
                case BALLTWO:
                    checkTime();
                    robot.SetLoaderPosition(0.0f);
                    TimeUnit.SECONDS.sleep(1);
                    robot.Shooter.setPower(0.0f);
                    System.out.println("DONE SHOOTING");
                    State = driveState.DRIVETOFIRSTBEACON;
                    break;
                case DRIVETOFIRSTBEACON:
                    checkTime();
                    System.out.println("DRIVE TO FIRST BEACON");

                    // Get pose and translation
                    robot.SetButtonPusherPosition(0.45f);
                    //setMode(norm);

                    pose = ((VuforiaTrackableDefaultListener) beacons.get(FIRSTPICTURE).getListener()).getRawPose();

                    if(pose != null && x > stopCount) {
                        System.out.println("SEEABLE");
                        translation = pose.getTranslation();
                        y = (int) translation.get(1);
                        x = (int) translation.get(2);
                        System.out.println("X: " + x);
                        telemetry.addData("x: ", x);
                        //telemetry.update();

                        // Drive to beacon using PID controller
                        System.out.println("PID DRIVE");
                        pidDrive(y);
                    }

                    // Stop if close enough to picture
                    if(pose == null) {
                        setPower(0.0f);
                    }
                    if(x <= stopCount) {
                        setPower(0.0f);
                        State = driveState.PUSHFIRSTBEACON;

                        //breakLoop = true;
                    }
                    break;
                case PUSHFIRSTBEACON:
                    checkTime();
                    System.out.println("PUSH FIRST BEACON");
                    // Push the beacon button
                    pushBeaconButton();
                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.encoderMove(-1800, -1800, 0.6f, 0.6f);
                    State = driveState.SECONDSTRAFE;
                    //breakLoop = true;
                    break;
                case SECONDSTRAFE:
                    checkTime();
                    System.out.println("SECOND STRAFE");
                    // Move close to the second picture
                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    encoderStrafeRight(4700, 0.63f);
                    //encoderStrafeLeftDualPower(3750, 0.7f, 1000, 0.4f);
                    State = driveState.DRIVETOSECONDBEACON;
                    x = stopCount + 10;
                    break;
                case DRIVETOSECONDBEACON:
                    checkTime();
                    System.out.println("DRIVE TO SECOND BEACON");

                    // Get pose and translation
                    robot.SetButtonPusherPosition(0.45f);
                    //setMode(norm);

                    pose = ((VuforiaTrackableDefaultListener) beacons.get(SECONDPICTURE).getListener()).getRawPose();

                    if(pose != null && x > stopCount) {
                        System.out.println("SEEABLE");
                        translation = pose.getTranslation();
                        y = (int) translation.get(1);
                        x = (int) translation.get(2);
                        System.out.println("X: " + x);
                        telemetry.addData("x: ", x);
                        //telemetry.update();

                        // Drive to beacon using PID controller
                        System.out.println("PID DRIVE");
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
                    checkTime();
                    System.out.println("PUSH SECOND BEACON");
                    // Push the beacon button
                    pushBeaconButton();
                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.encoderMove(-500, -500, 0.4f, 0.4f);
                    State = driveState.END;
                    break;
                case END:
                    checkTime();
                    System.out.println("END");
                    setPower(0.0f);
                    breakLoop = true;
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

        double kP = 0.00045; // 0.0005
        double power = 0.2;
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
    public void pushBeaconButton() throws InterruptedException{
        //if (beaconFlagFirst == 0) {
        telemetry.addData("Blue Right", sensorRGBright.blue());
        telemetry.addData("Red Right", sensorRGBright.red());
        telemetry.addData("Blue Left", sensorRGBleft.blue());
        telemetry.addData("Red Left", sensorRGBleft.red());
        TimeUnit.MILLISECONDS.sleep(280);
//        telemetry.update();
        if(!((sensorRGBright.blue() > sensorRGBright.red() && sensorRGBleft.red() > sensorRGBleft.blue()) || (sensorRGBright.red() > sensorRGBright.blue() && sensorRGBleft.blue() > sensorRGBleft.red()))){
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.encoderMove(50, 50, 0.2f, 0.2f);
        }
        if (sensorRGBright.blue() > sensorRGBright.red() && sensorRGBleft.red() > sensorRGBleft.blue()) {
//            telemetry.update();
            robot.SetButtonPusherPosition(0.00);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TimeUnit.SECONDS.sleep(1);
            this.encoderMove(300, 300, 0.2f, 0.2f);
            beaconFlagFirst = 1;
            TimeUnit.MILLISECONDS.sleep(250);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int count = 100;
            while(sensorRGBright.red() < sensorRGBright.blue() && count <= 160) {
                this.encoderMove(count, count, 0.3f, 0.3f);
                count+=50;
            }
            //requestOpModeStop();
        } else if (sensorRGBright.red() > sensorRGBright.blue() && sensorRGBleft.blue() > sensorRGBleft.red()) {
            // telemetry.update();
            robot.SetButtonPusherPosition(0.90);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TimeUnit.SECONDS.sleep(1);
            this.encoderMove(300, 300, 0.2f, 0.2f);
            beaconFlagFirst = 1;
            TimeUnit.MILLISECONDS.sleep(250);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int count = 100;
            while(sensorRGBleft.red() < sensorRGBleft.blue() && count <= 160){
                this.encoderMove(count, count, 0.3f, 0.3f);
                count+=50;
            }
        } else {
            telemetry.addData("unknown", "unknown");
//            telemetry.update();
        }
        //}
    }
}
