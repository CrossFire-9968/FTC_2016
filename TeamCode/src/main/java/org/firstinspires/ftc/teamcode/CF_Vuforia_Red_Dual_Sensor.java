package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.TrackerManager;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Crossfire_Hardware;

import java.util.concurrent.TimeUnit;

/**
 * Created by Ryley on 10/5/16.
 */
@Autonomous(name="CF_Vuforia_Red_Dual_Sensor", group ="Blue")
//@Disabled
public class CF_Vuforia_Red_Dual_Sensor extends CF_Library implements SensorEventListener {
    private ElapsedTime runtime = new ElapsedTime();

    static double TIMEOUT = 27;

    float xAccel = 0;
    float yAccel = 0;
    float zAccel = 0;
    OpenGLMatrix pose = null;

    final int stopCount = 110;
    boolean seeableFirst;
    boolean seeableSecond;
    int x;
    int y;
    int z;
    int error;


    double kP = 0.0005;
    double power = 0.2;
    double effort;
    double leftPower;
    double rightPower;
    final int FIRSTPICTURE = 3;
    final int SECONDPICTURE = 1;
    int encoderCounts = 0;
    int picFlag = 0;
    int beaconFlagFirst = 0;

    ColorSensor sensorRGBright;
    ColorSensor sensorRGBleft;

    private SensorManager sensorManager;
    private Sensor accelSen;


    float hsvValuesright[] = {0F, 0F, 0F};
    float hsvValuesleft[] = {0F, 0F, 0F};

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.MotorMecanumLeftFront.setDirection(DcMotor.Direction.FORWARD);     // Set to REVERSE if using AndyMark motors
        robot.MotorMecanumLeftRear.setDirection(DcMotor.Direction.FORWARD);      // Set to REVERSE if using AndyMark motors
        robot.MotorMecanumRightFront.setDirection(DcMotor.Direction.REVERSE);    // Set to FORWARD if using AndyMark motors
        robot.MotorMecanumRightRear.setDirection(DcMotor.Direction.REVERSE);


        int firstFlag = 0;

        int turnFlagFirst = 0;
        int turnFlagSecond = 0;



        int countRight = 1;
        int countLeft = 1;

        final int LEFT = 0;
        final int RIGHT = 1;


        DeviceInterfaceModule cdimright;
        DeviceInterfaceModule cdimleft;


        final int LED_CHANNEL = 5;
        
        final float speed = 0.5f;


        // Makes camera output appear on screen
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        // Sets camera direction
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        // License key
        params.vuforiaLicenseKey = "AU2Vspr/////AAAAGSWZlF6AQEHFh9mbNlt5KlFGl/PX8qeeKea7jh5Xk8Ei573/nsoAjsJu9Cbi2MlRCuEIkZHQJoDGAxXmNgioA+0+DbRC6mG+1QbBu8ACMw0pBk6x3h+wvvqDeyZmjV0Fdji5Bk2bV3AaZ0AanljM2nuosjfFYOeUsoFqjE0+MQfJCOoG2ED2hxhJM88dhMaAH45kQqJ99Pn9c/F8whHUkRLeh71wW3O8qGdHEieX7WQO86VfVadHTrg0Ut8ALwiU/qVqB9pJPn+oVe9rYCixcJztb7XOp4T4Mo0IPUwVtkTUZtZTW1mAOPdbbWx3RX1OohA6q6BBU7ozDdQ1W33/L/mdETevYMf7rKPrb82Zbw8r";
        // Sets the thing you see on screen.  Could be AXES, TEAPOT, BUILDINGS, OR NONE
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        // Lets VuForia see more than one thing at a time
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");


        cdimright = hardwareMap.deviceInterfaceModule.get("CF_DimRight");


        cdimright.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        sensorRGBright = hardwareMap.colorSensor.get("AdafruitRGBright");
        sensorRGBleft = hardwareMap.colorSensor.get("AdafruitRGBleft");

        initalize();
        //waitForStart();

        // Activate tracking
        beacons.activate();

        //Attach Accel listner
        sensorManager.registerListener(this, accelSen, SensorManager.SENSOR_DELAY_NORMAL);

        telemetry.addData("xAccel", xAccel);
        telemetry.clearAll();
        telemetry.update();


        //5060

        while (opModeIsActive()) {
            if (firstFlag == 0 && runtime.seconds() < TIMEOUT) {
                this.encoderStrafeRight(3000, speed, runtime);
                this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.encoderMove(1500, 1500, speed, speed, runtime);
                this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.encoderStrafeRight(1750, speed, runtime);
                TimeUnit.SECONDS.sleep((long)0.5f);
                firstFlag = 1;
            }
            seeableFirst = ((VuforiaTrackableDefaultListener) beacons.get(FIRSTPICTURE).getListener()).isVisible();
            while (!seeableFirst && !isStopRequested() && turnFlagFirst == 0 && runtime.seconds() < TIMEOUT) {
                this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.encoderStrafeRight(100, speed, runtime);
                seeableFirst = ((VuforiaTrackableDefaultListener) beacons.get(FIRSTPICTURE).getListener()).isVisible();
//                if(runtime.seconds() > TIMEOUT) {
//                    requestOpModeStop();
//                }
                telemetry.clearAll();
                telemetry.addData("Timer: ", runtime.seconds());
                telemetry.update();
            }

            if (seeableFirst) {
                setPower(0.0f);
                turnFlagFirst = 1;
            }

            driveToBeacon(FIRSTPICTURE, beacons);

            pushBeaconButton();

            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            this.encoderStrafeRight(4150, speed, runtime);

            TimeUnit.SECONDS.sleep((long)0.5);

            seeableSecond = ((VuforiaTrackableDefaultListener) beacons.get(SECONDPICTURE).getListener()).isVisible();
            while (!seeableSecond && !isStopRequested() && turnFlagSecond == 0 && runtime.seconds() < TIMEOUT) {
                this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.encoderStrafeRight(100, speed, runtime);
                seeableSecond = ((VuforiaTrackableDefaultListener) beacons.get(SECONDPICTURE).getListener()).isVisible();
            }

            if (seeableSecond) {
                setPower(0.0f);
                turnFlagSecond = 1;
            }

            picFlag = 0;

            driveToBeacon(SECONDPICTURE, beacons);

            pushBeaconButton();

            requestOpModeStop();


        }


    }

    void initalize() throws java.lang.InterruptedException {
        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.MotorMecanumLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorMecanumRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorMecanumLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorMecanumRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //idle();

        robot.MotorMecanumLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MotorMecanumRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MotorMecanumLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MotorMecanumRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Encoder Reset!", "Encoder Reset");

        sensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        accelSen = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        // Wait for the game to start
        // (driver presses PLAY)
        waitForStart();
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        xAccel = event.values[0];
        yAccel = event.values[1];
        zAccel = event.values[2];
    }

    public void driveToBeacon(int pictureNumber, VuforiaTrackables beaconsArray) {
        if (isStopRequested()) {
            requestOpModeStop();
        }
        robot.SetButtonPusherPosition(0.45f);
        pose = ((VuforiaTrackableDefaultListener) beaconsArray.get(pictureNumber).getListener()).getRawPose();

        if(pose != null) {
            VectorF translation = pose.getTranslation();
            x = (int) translation.get(2);
            telemetry.clearAll();
            telemetry.addData("visible", "visible");
            telemetry.addData("xValue: ", x);
            telemetry.update();

            //x, y, and z position
            VectorF xPose = pose.getRow(2);
            VectorF yPose = pose.getRow(1);
            VectorF zPose = pose.getRow(0);

            double XPose = (double) xPose.get(0);
            double cosXPose = Math.toDegrees(Math.acos(XPose));

            // Get the x, y, and z components, and cast them to ints, because we don't need the full
            // double precision
            z = (int) translation.get(0); //Switch y & z for landscape mode
            y = (int) translation.get(1);
            x = (int) translation.get(2);

            telemetry.addData("x: ", x / 25.4);
            telemetry.addData("y: ", y / 25.4);
            telemetry.addData("z: ", z / 25.4);

            telemetry.addData("xPose: ", xPose);
            telemetry.addData("yPose: ", yPose);
            telemetry.addData("zPose: ", zPose);
            telemetry.addData("cosXPose: ", cosXPose);
            seeableFirst = ((VuforiaTrackableDefaultListener) beaconsArray.get(pictureNumber).getListener()).isVisible();
        }

        while (x >= stopCount && !isStopRequested() && seeableFirst && encoderCounts < 5000 && runtime.seconds() < TIMEOUT) {
            pose = ((VuforiaTrackableDefaultListener) beaconsArray.get(pictureNumber).getListener()).getRawPose();


            // NOTE: If picture not seen is pose = NULL?
            // Is "visible" on screen with light?
            while (pose != null && picFlag == 0 && !isStopRequested() && runtime.seconds() < TIMEOUT) {
                if (isStopRequested()) {
                    requestOpModeStop();
                }
                VectorF translation = pose.getTranslation();
                x = (int) translation.get(2);
                telemetry.clearAll();
                telemetry.addData("visible", "visible");
                telemetry.addData("xValue: ", x);
                telemetry.update();

                // Get the x, y, and z components, and cast them to ints, because we don't need the full
                // double precision
                z = (int) translation.get(0); //Switch y & z for landscape mode
                y = (int) translation.get(1);
                x = (int) translation.get(2);

                telemetry.addData("x: ", x / 25.4);
                telemetry.addData("y: ", y / 25.4);
                telemetry.addData("z: ", z / 25.4);

                seeableFirst = ((VuforiaTrackableDefaultListener) beaconsArray.get(pictureNumber).getListener()).isVisible();

                while (x >= stopCount && !isStopRequested() && seeableFirst && encoderCounts < 5000 && runtime.seconds() < TIMEOUT) {
                    pose = ((VuforiaTrackableDefaultListener) beaconsArray.get(pictureNumber).getListener()).getRawPose();
                    seeableFirst = ((VuforiaTrackableDefaultListener) beaconsArray.get(pictureNumber).getListener()).isVisible();
                    encoderCounts = robot.MotorMecanumLeftFront.getCurrentPosition();
                    if (pose != null) {
                        translation = pose.getTranslation();
                        y = (int) translation.get(1);
                        x = (int) translation.get(2);
                        telemetry.addData("xPose: ", x);
                        telemetry.addData("y: ", y);
                        telemetry.update();
                        error = y + 10;
                        effort = kP * error;
                        rightPower = power + effort;
                        leftPower = power - effort;
                        robot.MotorMecanumLeftFront.setPower(leftPower);
                        robot.MotorMecanumRightFront.setPower(rightPower);
                        robot.MotorMecanumLeftRear.setPower(leftPower);
                        robot.MotorMecanumRightRear.setPower(rightPower);
                    }
                }
                if ((x < stopCount && !isStopRequested()) || !seeableFirst || encoderCounts >= 5000) {
                    robot.MotorMecanumLeftFront.setPower(0.0f);
                    robot.MotorMecanumRightFront.setPower(0.0f);
                    robot.MotorMecanumLeftRear.setPower(0.0f);
                    robot.MotorMecanumRightRear.setPower(0.0f);
                    if (x < stopCount && !isStopRequested()) {
                        picFlag = 1;
                    }
                }
                pose = ((VuforiaTrackableDefaultListener) beaconsArray.get(pictureNumber).getListener()).getRawPose();

            }
        }
    }

    public void pushBeaconButton() throws InterruptedException{
        //if (beaconFlagFirst == 0) {
        Color.RGBToHSV((sensorRGBright.red() * 255) / 800, (sensorRGBright.green() * 255) / 800, (sensorRGBright.blue() * 255) / 800, hsvValuesright);
        Color.RGBToHSV((sensorRGBleft.red() * 255) / 800, (sensorRGBleft.green() * 255) / 800, (sensorRGBleft.blue() * 255) / 800, hsvValuesleft);
        if (sensorRGBright.blue() > sensorRGBright.red() && sensorRGBleft.red() > sensorRGBleft.blue() && runtime.seconds() < TIMEOUT) {
            telemetry.addData("blue", hsvValuesright[0]);
            telemetry.update();
            robot.SetButtonPusherPosition(0.90);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TimeUnit.SECONDS.sleep(1);
            this.encoderMove(300, 300, 0.2f, 0.2f, runtime);
            beaconFlagFirst = 1;
            TimeUnit.SECONDS.sleep(1);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int count = 100;
            while(sensorRGBright.red() < sensorRGBright.blue() && count <= 600){
                this.encoderMove(count, count, 0.3f, 0.3f, runtime);
                count+=50;
            }
            this.encoderMove(-1500, -1500, 0.2f, 0.2f, runtime);

            //requestOpModeStop();
        } else if (sensorRGBright.red() > sensorRGBright.blue() && sensorRGBleft.blue() > sensorRGBleft.red() && runtime.seconds() < TIMEOUT) {
            telemetry.addData("red", hsvValuesright[0]);
            telemetry.update();
            robot.SetButtonPusherPosition(0.00);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TimeUnit.SECONDS.sleep(1);
            this.encoderMove(300, 300, 0.2f, 0.2f, runtime);
            beaconFlagFirst = 1;
            TimeUnit.SECONDS.sleep(1);
            this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int count = 100;
            while(sensorRGBleft.blue() > sensorRGBleft.red() && count <= 600) {
                this.encoderMove(count, count, 0.3f, 0.3f, runtime);
                count += 50;
            }
            this.encoderMove(-1500, -1500, 0.2f, 0.2f, runtime);

            //requestOpModeStop();
        } else {
            telemetry.addData("unknown", hsvValuesright[0]);
            telemetry.update();
        }
        //}
    }
}



