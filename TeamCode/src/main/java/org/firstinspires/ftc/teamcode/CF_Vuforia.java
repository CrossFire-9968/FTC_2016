package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.vuforia.HINT;
//import com.vuforia.TrackerManager;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.teamcode.Crossfire_Hardware;
import org.firstinspires.ftc.teamcode.CF_SensorLibrary;

/**
 * Created by Ryley on 10/5/16.
 */
@TeleOp(name = "CF_Vuforia", group = "Test")
//@Disabled
public class CF_Vuforia extends CF_Library implements SensorEventListener
{
   CF_SensorLibrary colorSensor = new CF_SensorLibrary();

   float xAccel = 0;
   float yAccel = 0;
   float zAccel = 0;

   private SensorManager sensorManager;
   private Sensor accelSen;



   @Override
   public void runOpMode() throws InterruptedException
   {
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
      int picFlag = 0;
      int turnFlag = 0;

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

      /**********************************************************************************************
       * REVIEW NOTE: You only run this code once, so you don't need a while loop.
       * An if will work for a one-time check.
       **********************************************************************************************/
      if (opModeIsActive())
      {
         // Start moving - drive straight forward
         encoderMove(4600, 4600, 0.8f, 0.8f);

         // Check to see if the picture is visible
         seeable = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).isVisible();

         int leftCount = robot.MotorMecanumLeftFront.getCurrentPosition();
         int rightCount = robot.MotorMecanumRightFront.getCurrentPosition();

         /**********************************************************************************************
          * REVIEW NOTE: Can we rotate close on the first guess then increment by small steps?  If so,
          * will speed up the rate you acquire picture and be less jerky.
          *
          * Removed !isStopRequested because you wouldn't reach this code if it was.
          * Removed turnFlag == 0 as you don't use it in this part of the code.
          **********************************************************************************************/
         while (!seeable)// && !isStopRequested() && turnFlag == 0)
         {
            leftCount -= 60;
            rightCount += 60;

            /**********************************************************************************************
             * REVIEW NOTE: moving by encoders, I think the negative power is messing you up.  When using
             * RUN_TO_POSITION, direction is determined by the encoder counts.  By commanding a negative
             * power, you may be unintentionally reversing the direction.
             **********************************************************************************************/
            this.encoderMove(leftCount, rightCount, -0.2f, 0.2f);

            // Can robot see the picture?
            seeable = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).isVisible();

            /**********************************************************************************************
             * REVIEW NOTE: Unless you are explicitly exiting this opmode for reasons other than the stop
             * button was pushed, then there is no need to make the request.  It's already done for you.
             **********************************************************************************************/
//            if (isStopRequested())
//            {
//               requestOpModeStop();
//            }

            /**********************************************************************************************
             * REVIEW NOTE: Moved this inside the while loop.
             **********************************************************************************************/
            if (seeable)
            {
               robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);

               /**********************************************************************************************
                * REVIEW NOTE: Removed turnFlag as it is not used if you actually didn't need it as part of the
                * while loop this is code is in.
                **********************************************************************************************/
               turnFlag = 1;
            }
         }

         // Get position information from Vuforia
         pose = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).getRawPose();

         while (pose != null && picFlag == 0 && !isStopRequested())
         {
            /**********************************************************************************************
             * REVIEW NOTE: Unless you are explicitly exiting this opmode for reasons other than the stop
             * button was pushed, then there is no need to make the request.  It's already done for you.
             **********************************************************************************************/
//            if (isStopRequested())
//            {
//               requestOpModeStop();
//            }

            telemetry.clearAll();
            telemetry.addData("visible", "visible");
            telemetry.update();

            VectorF translation = pose.getTranslation();

            //x, y, and z position
            VectorF xPose = pose.getRow(2);
            VectorF yPose = pose.getRow(1);
            VectorF zPose = pose.getRow(0);

            double XPose = (double) xPose.get(0);
            double cosXPose = Math.toDegrees(Math.acos(XPose));

            /**********************************************************************************************
             * REVIEW NOTE: Don't cast to int and then divide by a double (x/25.4).  Do floating point
             * math first, then cast result to an int if you wish.  As programmed, you are losing precision.
             **********************************************************************************************/
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

            // Can robot see the picture?
            seeable = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).isVisible();


            while (x >= 100 && !isStopRequested() && seeable)
            {
               /**********************************************************************************************
                * REVIEW NOTE: Move to the end of while as we already have fresh data.
                **********************************************************************************************/
//               pose = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).getRawPose();
//               seeable = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).isVisible();

               // Tank turn robot until y-error is zero (proportional control only)
               if (pose != null)
               {
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
                  robot.setMecanumPowers(leftPower, rightPower, leftPower, rightPower);
               }

               // Update position data and if robot can see picture
               pose = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).getRawPose();
               seeable = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).isVisible();
            }

            /**********************************************************************************************
             * REVIEW NOTE: Removed if statement.  If you exited the while loop, you are grestop requested check as that is aleady being handled by while loop.
             **********************************************************************************************/
            if ((x < 100 && !isStopRequested()) || !seeable)
            {
               robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);
               if (x < 100 && !isStopRequested())
               {
                  picFlag = 1;
               }
            }

            pose = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).getRawPose();
         }


         CF_SensorLibrary.sensorColor beaconColor = colorSensor.GetAdafruitColor();

         // Get beacon color do something with it.
         // In this code it prints to driver station phone
         if (beaconColor == CF_SensorLibrary.sensorColor.blue && picFlag == 1)
         {
            telemetry.addData("Beacon: ", "Blue");
         }
         else if (beaconColor == CF_SensorLibrary.sensorColor.red && picFlag == 1)
         {
            telemetry.addData("Beacon: ", "Red");
         }
         else
         {
            telemetry.addData("Beacon: ", "Unknown");
         }

         telemetry.update();
      }
   }


   void initalize() throws java.lang.InterruptedException
   {
      // Send telemetry message to signify robot waiting
      telemetry.addData("Status", "Resetting Encoders");
      telemetry.update();

      // Initalize encoder counts to new zero reference and wait
      robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      /********************************************************************
       * REVIEW NOTE: why set encoders to run in init.  Wait until needed
       ********************************************************************/
      //robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

      // Send telemetry message to indicate successful Encoder reset
      telemetry.addData("Encoder Reset!", "Encoder Reset");

      /********************************************************************
       * REVIEW NOTE: Commented out until we get autonomous drive debugged
       ********************************************************************/
//      sensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
//      accelSen = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

      // Wait for the game to start
      // (driver presses PLAY)
      waitForStart();
   }


   @Override
   public void onAccuracyChanged(Sensor sensor, int accuracy)
   {

   }

   @Override
   public void onSensorChanged(SensorEvent event)
   {
      xAccel = event.values[0];
      yAccel = event.values[1];
      zAccel = event.values[2];
   }
}