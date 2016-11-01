package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

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
@TeleOp(name="CF_Vuforia", group ="Test")
//@Disabled
public class CF_Vuforia extends CF_Library {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        int x;
        int y;
        int z;
        int error;
        double kP = 0.0005;
        double power = 0.3;
        double effort;
        double leftPower;
        double rightPower;
        boolean seeable;
        int firstFlag = 0;
        int picFlag = 0;
        int turnFlag = 0;

        int countRight = 1;
        int countLeft = 1;

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

        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;

        cdim = hardwareMap.deviceInterfaceModule.get("CF_Dim");

        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        sensorRGB = hardwareMap.colorSensor.get("AdafruitRGB");

        initalize();
        //waitForStart();

        // Activate tracking
        beacons.activate();
        OpenGLMatrix pose = null;
        telemetry.clearAll();
        telemetry.update();

        while (opModeIsActive()) {

            if(firstFlag == 0) {
                this.encoderMove(4600, 4600, 0.8f, 0.8f);
                this.encoderMove(3600, 5600, -0.3f, 0.3f);
                firstFlag = 1;
            }
             seeable = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).isVisible();
//             while(!seeable) {
//                    telemetry.addData("not visible", "not visible");
//                    telemetry.update();
//                    robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.rightMotor.setPower(0.15f);
//                    //robot.leftMotor.setPower(-0.15f);
//                    seeable = ((VuforiaTrackableDefaultListener) beacons.get(3).getListener()).isVisible();
//                    pose = ((VuforiaTrackableDefaultListener) beacons.get(3).getListener()).getRawPose();
//             }
            int leftCount = robot.leftMotor.getCurrentPosition();
            int rightCount = robot.rightMotor.getCurrentPosition();
            while(!seeable && !isStopRequested() && turnFlag == 0) {
                leftCount -= 60;
                rightCount += 60;
                this.encoderMove(leftCount, rightCount, -0.2f, 0.2f);
                seeable = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).isVisible();
                if(isStopRequested()) {
                    requestOpModeStop();
                }


            }

            if(seeable) {
                robot.rightMotor.setPower(0.0f);
                turnFlag = 1;
                //robot.leftMotor.setPower(0.0f);
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

                    while (x >= 100 && !isStopRequested() && seeable) {
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
                            robot.leftMotor.setPower(leftPower);
                            robot.rightMotor.setPower(rightPower);
                        }
                    }
                    if ((x < 100 && !isStopRequested()) || !seeable) {
                        robot.leftMotor.setPower(0.0f);
                        robot.rightMotor.setPower(0.0f);
                        if(x < 100 && !isStopRequested()) {
                            picFlag = 1;
                        }
                    }
                pose = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).getRawPose();


            }
            telemetry.update();
            Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
            float hue = hsvValues[0];
            if(hue < 310 && picFlag == 1) {
                telemetry.addData("Blue", "Blue");
                telemetry.update();
            }
            if(hue >= 310 && picFlag == 1) {
                telemetry.addData("Red", "Red");
                telemetry.update();
            }



        }


    }
    void initalize() throws java.lang.InterruptedException {
        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Encoder Reset!", "Encoder Reset");
        // Wait for the game to start
        // (driver presses PLAY)
        waitForStart();
    }
}
