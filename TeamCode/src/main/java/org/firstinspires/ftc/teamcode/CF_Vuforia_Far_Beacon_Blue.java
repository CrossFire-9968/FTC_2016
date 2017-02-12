package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 * Created by Ryley on 2/5/17.
 */
@Autonomous(name="CF_Vuforia_Far_Beacon_Blue", group ="Blue")
//@Disabled
public class CF_Vuforia_Far_Beacon_Blue extends CF_Library {
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
    int angle;
    int errorY;
    double errorAngle;
    int ySquare;
    double turnFront;
    double turnRear;

    float[] data;

    int FIRSTPICTURE = 0;
    int SECONDPICTURE = 2;

    int beaconFlagFirst = 0;

    ColorSensor sensorRGBright;
    ColorSensor sensorRGBleft;

    private enum driveState {
        FIRSTSTRAFE, SQUARETOFIRSTBEACON, DRIVETOFIRSTBEACON, PUSHFIRSTBEACON, SECONDSTRAFE, BALLONE, BALLTWO, DRIVETOSECONDBEACON, PUSHSECONDBEACON, END
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


        final float speed = 0.6f;

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        params.vuforiaLicenseKey = "AU2Vspr/////AAAAGSWZlF6AQEHFh9mbNlt5KlFGl/PX8qeeKea7jh5Xk8Ei573/nsoAjsJu9Cbi2MlRCuEIkZHQJoDGAxXmNgioA+0+DbRC6mG+1QbBu8ACMw0pBk6x3h+wvvqDeyZmjV0Fdji5Bk2bV3AaZ0AanljM2nuosjfFYOeUsoFqjE0+MQfJCOoG2ED2hxhJM88dhMaAH45kQqJ99Pn9c/F8whHUkRLeh71wW3O8qGdHEieX7WQO86VfVadHTrg0Ut8ALwiU/qVqB9pJPn+oVe9rYCixcJztb7XOp4T4Mo0IPUwVtkTUZtZTW1mAOPdbbWx3RX1OohA6q6BBU7ozDdQ1W33/L/mdETevYMf7rKPrb82Zbw8r";

        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        sensorRGBright = hardwareMap.colorSensor.get("AdafruitRGBright");
        sensorRGBleft = hardwareMap.colorSensor.get("AdafruitRGBleft");

        driveState State = driveState.FIRSTSTRAFE;

        beacons.activate();

        initalize();

        runtime.reset();

        while(opModeIsActive() && !breakLoop && runtime.seconds() < endTime) {
            switch (State) {
                case FIRSTSTRAFE:
                    checkTime();
                    System.out.println("FIRST STRAFE");

                    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.encoderStrafeLeft(3000, speed);
                    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.encoderMove(1100, 1100, 0.6f, 0.6f);
                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.encoderStrafeLeft(6750, speed);
                    breakLoop = true;
                    break;
            }
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
    private void checkTime() {
        // Kills the robot if time is over the endTime
        if(runtime.seconds() >= endTime) {
            breakLoop = true;
            requestOpModeStop();
        }
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
    // Pushes the correct beacon button depending on the color
    public void pushBeaconButton() throws InterruptedException{
        //if (beaconFlagFirst == 0) {
        telemetry.addData("Blue Right", sensorRGBright.blue());
        telemetry.addData("Red Right", sensorRGBright.red());
        telemetry.addData("Blue Left", sensorRGBleft.blue());
        telemetry.addData("Red Left", sensorRGBleft.red());
        TimeUnit.MILLISECONDS.sleep(280);
//        telemetry.update();
        if (sensorRGBright.blue() > sensorRGBright.red() && sensorRGBleft.red() > sensorRGBleft.blue()) {
//            telemetry.update();
            robot.SetButtonPusherPosition(0.90);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TimeUnit.SECONDS.sleep(1);
            this.encoderMove(300, 300, 0.2f, 0.2f);
            beaconFlagFirst = 1;
            TimeUnit.MILLISECONDS.sleep(250);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int count = 100;
            while(sensorRGBleft.red() > sensorRGBleft.blue() && count <= 160) {
                this.encoderMove(count, count, 0.3f, 0.3f);
                count+=50;
            }
            //requestOpModeStop();
        } else if (sensorRGBright.red() > sensorRGBright.blue() && sensorRGBleft.blue() > sensorRGBleft.red()) {
            // telemetry.update();
            robot.SetButtonPusherPosition(0.00);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TimeUnit.SECONDS.sleep(1);
            this.encoderMove(300, 300, 0.2f, 0.2f);
            beaconFlagFirst = 1;
            TimeUnit.MILLISECONDS.sleep(250);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int count = 100;
            while(sensorRGBright.red() > sensorRGBright.blue() && count <= 160){
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
