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
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by Ryley on 10/5/16.
 */
@TeleOp(name = "CF_Vuforia_Jeff", group = "Test")
//@Disabled
public class CF_Vuforia_Jeff extends CF_Library implements SensorEventListener
{
   CF_SensorLibrary colorSensor = new CF_SensorLibrary();

   float xAccel = 0;
   float yAccel = 0;
   float zAccel = 0;

   private SensorManager sensorManager;
   private Sensor accelSen;

   private enum autonomousState
   {
      init, moveToLine, turnTowardBeacon, driveToBeacon, pushBeaconButton, autonomousComplete
   }

   autonomousState aState = autonomousState.init;


   @Override
   public void runOpMode() throws InterruptedException
   {
      robot.init(hardwareMap);
      float x;
      float y;
      float z;
      double error;
      double kP = 0.0005;
      double power = 0.3;
      double effort;
      double leftPower;
      double rightPower;
      boolean seeable;
      int picFlag = 0;
      int leftCount = 0;
      int rightCount = 0;
      boolean autoDone = false;

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

      //Attach Accel listner
      sensorManager.registerListener(this, accelSen, SensorManager.SENSOR_DELAY_NORMAL);
      OpenGLMatrix pose = null;

      telemetry.addData("xAccel", xAccel);
      telemetry.clearAll();
      telemetry.update();


      while (opModeIsActive() || !autoDone)
      {
         seeable = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).isVisible();

         // Update driver station with visible status
         if (seeable)
         {
            telemetry.clearAll();
            telemetry.addData("visible", "visible");
            telemetry.update();
         }

         switch (aState)
         {
            case init:
               initalize();

               // Activate tracking
               beacons.activate();

               aState = autonomousState.moveToLine;
               break;

            case moveToLine:
               // Drive straight forward until we are parallel to beacon and stop
               encoderMove(4600, 4600, 0.8f, 0.8f);

               // Now drive toward beacon
               aState = autonomousState.turnTowardBeacon;
               break;

            case turnTowardBeacon:
               /**********************************************************************************************
                * REVIEW NOTE: Can we rotate close on the first guess then increment by small steps?  If so,
                * will speed up the rate you acquire picture and be less jerky.
                *
                * Removed !isStopRequested because you wouldn't reach this code if it was.
                * Removed turnFlag == 0 as you don't use it in this part of the code.
                **********************************************************************************************/
               if (!seeable)
               {
                  leftCount = robot.MotorMecanumLeftFront.getCurrentPosition() - 60;
                  rightCount = robot.MotorMecanumRightFront.getCurrentPosition() + 60;

                  /**********************************************************************************************
                   * REVIEW NOTE: moving by encoders, I think the negative power is messing you up.  When using
                   * RUN_TO_POSITION, direction is determined by the encoder counts.  By commanding a negative
                   * power, you may be unintentionally reversing the direction.
                   **********************************************************************************************/
                  this.encoderMove(leftCount, rightCount, -0.2f, 0.2f);
               }
               else
               {
                  robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
                  robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);

                  aState = autonomousState.driveToBeacon;
               }

               break;

            case driveToBeacon:
               // Get position information from Vuforia
               pose = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).getRawPose();

               if ((pose != null) && (picFlag == 0))
               {
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
                  z = translation.get(0); //Switch y & z for landscape mode
                  y = translation.get(1);
                  x = translation.get(2);

                  telemetry.addData("x: ", (int) (x / 25.4));
                  telemetry.addData("y: ", (int) (y / 25.4));
                  telemetry.addData("z: ", (int) (z / 25.4));

                  telemetry.addData("xPose: ", xPose);
                  telemetry.addData("yPose: ", yPose);
                  telemetry.addData("zPose: ", zPose);
                  telemetry.addData("cosXPose: ", cosXPose);
                  telemetry.update();

                  if ((x >= 100) && seeable)
                  {
                     error = y;
                     effort = kP * error;
                     rightPower = power + effort;
                     leftPower = power - effort;

                     // Set new motor powers
                     robot.setMecanumPowers(leftPower, rightPower, leftPower, rightPower);

                     // Update position data and if robot can see picture
                     pose = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).getRawPose();
                     seeable = ((VuforiaTrackableDefaultListener) beacons.get(PICTURE).getListener()).isVisible();
                  }
                  else
                  {
                     robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);
                     picFlag = 1;

                     aState = autonomousState.pushBeaconButton;
                  }
               }
               break;

            case pushBeaconButton:
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
               aState = autonomousState.autonomousComplete;
               break;

            case autonomousComplete:
               robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);
               robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               autoDone = true;
               break;
         }
      }
   }


   void initalize() throws InterruptedException
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