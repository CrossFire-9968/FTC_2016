package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        double power = 0.15;
        double effort;
        double leftPower;
        double rightPower;
        boolean seeable;
        int firstFlag = 0;

        int countRight = 1;
        int countLeft = 1;
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

        initalize();

        // Activate tracking
        beacons.activate();
        OpenGLMatrix pose = null;

        while (opModeIsActive()) {

            if(firstFlag == 0) {
                this.encoderMove(4900, 4900, 0.2f, 0.2f);
                firstFlag = 1;
            }
            //for(VuforiaTrackable beac : beacons) {
                // Get position matrix, pose
                seeable = ((VuforiaTrackableDefaultListener) beacons.get(1).getListener()).isVisible();
                if(seeable) {
                    pose = ((VuforiaTrackableDefaultListener) beacons.get(1).getListener()).getRawPose();
                    telemetry.addData("pose: ", pose);
                    telemetry.update();
                }
                if(!seeable) {
                    telemetry.addData()
                }
//                    while(seeable) {
//                        if(isStopRequested()) {
//                        requestOpModeStop();
//                    }
//                        telemetry.addData("seeable", "seeable");
//                        telemetry.update();
//                        seeable = ((VuforiaTrackableDefaultListener) beacons.get(1).getListener()).isVisible();
//
//                    }
//                    while(!seeable) {
//                        telemetry.addData("not Seeable", "not Seeable");
//                        telemetry.update();
//                        seeable = ((VuforiaTrackableDefaultListener) beacons.get(1).getListener()).isVisible();
//
//
//                    }
                while(!seeable) {
                    telemetry.addData("not visible", "not visible");
                    telemetry.addData("pose", pose);
                    telemetry.update();
                    robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.rightMotor.setPower(0.1f);
//                    robot.leftMotor.setPower(-0.1f);
                    seeable = ((VuforiaTrackableDefaultListener) beacons.get(1).getListener()).isVisible();
                }
                while(pose != null) {
                    if(isStopRequested()) {
                        requestOpModeStop();
                    }
                    pose = ((VuforiaTrackableDefaultListener) beacons.get(1).getListener()).getRawPose();
                    telemetry.clearAll();
                    telemetry.addData("visible", "visible");
                    telemetry.update();
                    robot.rightMotor.setPower(0.0f);
                    robot.leftMotor.setPower(0.0f);
                    VectorF translation = pose.getTranslation();

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
                    seeable = ((VuforiaTrackableDefaultListener) beacons.get(1).getListener()).isVisible();

                    while (x >= 250) {
                        pose = ((VuforiaTrackableDefaultListener) beacons.get(1).getListener()).getRawPose();
                        seeable = ((VuforiaTrackableDefaultListener) beacons.get(1).getListener()).isVisible();
                        if (pose != null) {
                            translation = pose.getTranslation();
                            y = (int) translation.get(1);
                            x = (int) translation.get(2);
                            telemetry.addData("x: ", x);
                            telemetry.addData("y: ", y);
                            telemetry.update();
                            error = y;
                            effort = kP * error;
                            rightPower = power - effort;
                            leftPower = power + effort;
                            robot.leftMotor.setPower(leftPower);
                            robot.rightMotor.setPower(rightPower);
                        }
                    }
                    if (x < 250) {
                        robot.leftMotor.setPower(0.0f);
                        robot.rightMotor.setPower(0.0f);
                        seeable = ((VuforiaTrackableDefaultListener) beacons.get(1).getListener()).isVisible();
                    }

                }
                telemetry.update();

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
