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

/**
 * Created by Ryley on 10/5/16.
 */
@Autonomous(name="CF_Vuforia_Blue_Wheels", group ="Blue")
//@Disabled
public class CF_Vuforia_Blue_Wheels extends CF_Library implements SensorEventListener{

    float xAccel = 0;
    float yAccel = 0;
    float zAccel = 0;

    private SensorManager sensorManager;
    private Sensor accelSen;

    @Override
    public void runOpMode ()throws InterruptedException {
        robot.init(hardwareMap);
       robot.MotorMecanumLeftFront.setDirection(DcMotor.Direction.FORWARD);     // Set to REVERSE if using AndyMark motors
       robot.MotorMecanumLeftRear.setDirection(DcMotor.Direction.FORWARD);      // Set to REVERSE if using AndyMark motors
       robot.MotorMecanumRightFront.setDirection(DcMotor.Direction.REVERSE);    // Set to FORWARD if using AndyMark motors
       robot.MotorMecanumRightRear.setDirection(DcMotor.Direction.REVERSE);
        int x;
        int y;
        int z;
        int error;
        double kP = 0.0005;
        double power = 0.2;
        double effort;
        double leftPower;
        double rightPower;
        boolean seeable;
        int firstFlag = 0;
        int picFlag = 0;
        int turnFlag = 0;
        final int RedUpperLimit_LowRange = 20;
        final int RedLowerLimit_LowRange = 0;
        final int RedUpperLimit_highRange = 360;
        final int RedLowerLimit_highRange = 325;
        final int BlueUpperLimit = 270;
        final int BlueLowerLimit = 220;

        int countRight = 1;
        int countLeft = 1;

        final int LEFT = 0;
        final int RIGHT = 1;

        ColorSensor sensorRGB;
        DeviceInterfaceModule cdim;


        final int LED_CHANNEL = 5;
        final int PICTURE = 0;

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

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;

        cdim = hardwareMap.deviceInterfaceModule.get("CF_Dim");

        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        sensorRGB = hardwareMap.colorSensor.get("AdafruitRGB");

        initalize();
        //waitForStart();

        // Activate tracking
        beacons.activate();

        //Attach Accel listner
        sensorManager.registerListener(this, accelSen, SensorManager.SENSOR_DELAY_NORMAL);
        OpenGLMatrix pose = null;
        telemetry.addData("xAccel", xAccel);
        telemetry.clearAll();
        telemetry.update();

        while (opModeIsActive()) {
            if (firstFlag == 0) {
                this.encoderStrafeLeft(5000, 0.3f);
                firstFlag = 1;
            }
            seeable = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).isVisible();
            int leftCount = robot.MotorMecanumLeftFront.getCurrentPosition();
            int rightCount = robot.MotorMecanumRightFront.getCurrentPosition();
            while (!seeable && !isStopRequested() && turnFlag == 0) {
                this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.encoderStrafeLeft(100, 0.3f);
                seeable = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).isVisible();
            }

            if (seeable) {
                setPower(0.0f);
                turnFlag = 1;
            }
            pose = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).getRawPose();


            while(pose != null && picFlag == 0 && !isStopRequested()) {
                if(isStopRequested()) {
                    requestOpModeStop();
                }
                telemetry.clearAll();
                telemetry.addData("visible", "visible");
                telemetry.update();
                VectorF translation = pose.getTranslation();
                //x, y, and z position
                VectorF xPose = pose.getRow(2);
                VectorF yPose = pose.getRow(1);
                VectorF zPose = pose.getRow(0);

                double XPose = (double)xPose.get(0);
                double cosXPose = Math.toDegrees(Math.acos(XPose));

                // Get the x, y, and z components, and cast them to ints, because we don't need the full
                // double precision
                z = (int)translation.get(0); //Switch y & z for landscape mode
                y = (int)translation.get(1);
                x = (int)translation.get(2);

                telemetry.addData("x: ", x/25.4);
                telemetry.addData("y: ", y/25.4);
                telemetry.addData("z: ", z/25.4);

                telemetry.addData("xPose: ", xPose);
                telemetry.addData("yPose: ", yPose);
                telemetry.addData("zPose: ", zPose);
                telemetry.addData("cosXPose: ", cosXPose);
                seeable = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).isVisible();

                while (x >= 75 && !isStopRequested() && seeable) {
                    pose = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).getRawPose();
                    seeable = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).isVisible();
                    if (pose != null) {
                        translation = pose.getTranslation();
                        y = (int) translation.get(1);
                        x = (int) translation.get(2);
                        telemetry.addData("x: ", x);
                        telemetry.addData("y: ", y);
                        telemetry.update();
                        error = y;
                        effort = kP * error;
                        rightPower = power + effort;
                        leftPower = power - effort;
                        robot.MotorMecanumLeftFront.setPower(leftPower);
                        robot.MotorMecanumRightFront.setPower(rightPower);
                        robot.MotorMecanumLeftRear.setPower(leftPower);
                        robot.MotorMecanumRightRear.setPower(rightPower);
                    }
                }
                if ((x < 75 && !isStopRequested()) || !seeable) {
                    robot.MotorMecanumLeftFront.setPower(0.0f);
                    robot.MotorMecanumRightFront.setPower(0.0f);
                    robot.MotorMecanumLeftRear.setPower(0.0f);
                    robot.MotorMecanumRightRear.setPower(0.0f);
                    if(x < 75 && !isStopRequested()) {
                        picFlag = 1;
                    }
                }
                pose = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).getRawPose();
            }

            Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
            float hue = hsvValues[0];
            if((hue >= BlueLowerLimit) && (hue <= BlueLowerLimit)){
                telemetry.addData("blue", "blue");
                telemetry.update();
                robot.SetButtonPusherPosition(0.28);
            }
            else if ((hue >= RedLowerLimit_highRange) && (hue <= RedUpperLimit_highRange) || (hue >= RedLowerLimit_LowRange) && (hue <= RedUpperLimit_LowRange)) {
                telemetry.addData("red", "red");
                telemetry.update();
                robot.SetButtonPusherPosition(0.70);
            }
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
}

