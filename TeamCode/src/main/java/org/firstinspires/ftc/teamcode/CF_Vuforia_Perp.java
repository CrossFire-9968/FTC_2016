package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Ryley on 12/11/16.
 */
@TeleOp(name = "CF_Vuforia_Perp", group = "Test")
//@Disabled
public class CF_Vuforia_Perp extends CF_Library {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.MotorMecanumLeftFront.setDirection(DcMotor.Direction.FORWARD);     // Set to REVERSE if using AndyMark motors
        robot.MotorMecanumLeftRear.setDirection(DcMotor.Direction.FORWARD);      // Set to REVERSE if using AndyMark motors
        robot.MotorMecanumRightFront.setDirection(DcMotor.Direction.REVERSE);    // Set to FORWARD if using AndyMark motors
        robot.MotorMecanumRightRear.setDirection(DcMotor.Direction.REVERSE);

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

        beacons.activate();

        OpenGLMatrix pose = null;

        VectorF translation = null;

        float[] data;

        int y;
        int x;

        double kPy = 0.00097;

        double kPangle = 0.0095;
        double kPangleSmall = 0.0009;
        double kIangle = 0.000085;
        double kIangleBig = 0.00023;
        //double kIangle = 0.001;

        //double kIangle = 0.0009;

        float effort;

        int angle;

        int errorY;
        double errorAngle;

        double integral = 0.0;

        double turnFront;
        double turnRear;

        final int PICTURE = 2;
        waitForStart();
        while (opModeIsActive()) {

            pose = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).getRawPose();

            if (pose != null) {
                translation = pose.getTranslation();
                data = pose.getData();
                angle = (int) Math.toDegrees(Math.acos(data[2]));
                y = (int) translation.get(1);
                x = (int) translation.get(2);
                errorY = y;
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
                    effort = (float) ((errorAngle * kPangleSmall) + (integral * kIangleBig));
                }
                else{
                    effort = (float) ((errorAngle * kPangle) + (integral * kIangle));
                }

                if(effort + turnFront < 0.05 && effort + turnRear < 0.05 && effort + turnFront > -0.05 && effort + turnRear > -0.05 && errorAngle < 5 && errorAngle > -5 && errorY < 7 && errorY > -7) {
                    requestOpModeStop();
                }
                strafe(effort + turnFront, effort + turnRear);
                telemetry.addData("error", errorY);
                telemetry.addData("turnRear", turnRear);
                telemetry.update();


            }
            //strafe(0.2f, 0.8f);
        }
    }
}
