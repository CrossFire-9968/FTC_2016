package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
import com.vuforia.ObjectTracker;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Crossfire_Hardware.sensorColor;
import org.firstinspires.ftc.robotcore.internal.VuforiaTrackablesImpl;

/***
 * This file provides basic Telop driving for a robot with mecanum wheels.
 * The code is structured as an Iterative OpMode
 * <p/>
 * This OpMode uses the CrossFire hardware class to define the devices on the robot.
 * All device access is managed through the Crossfire_Hardware class.
 * <p/>
 * This OpMode takes joystick values from three independent axis and computes a
 * desired motor power for each of the mecanum drive motors (one per wheel) to
 * attain desired velocity, direction, and rotation of robot. If the calculated
 * desired power for any motor exceeds the maximum power limit (1.0F), then all
 * motor powers are proportionally reduced.  We do this to retain consistent
 * driving characteristics at the expense of vehicle speed and total power.
 */

@TeleOp(name = "CF_Manual", group = "Drivetrain")
//@Disabled

public class CF_Manual extends OpMode
{
   Crossfire_Hardware robot = new Crossfire_Hardware();
   //CF_SensorLibrary colorSensor = new CF_SensorLibrary();

// Joystick threshold sets a minimum value for the controller joysticks
// to reach before the robot will begin to move.
   private static final float joystickThreshold = 0.003f;

//    When both of the joysticks used for driving are held at the same time, the
//    robot cannot fully do both functions. Priority sets the team's driving
//    priority - whether strafing, turning, or driving straight should have more
//    importance, causing to robot to perform more of one action than of the others.
   private static final float forwardPriority = 1.0f;
   private static final float strafePriority = 1.0f;
   private static final float steerPriority = 1.0f;
   private static final float forwardGain_Scoop = 1.0f;
   private static final float strafeGain_Scoop = 1.0f;
   private static final float steerGain_Scoop = 1.0f;
   private static final float forwardGain_Lifter = 0.5f;
   private static final float strafeGain_Lifter = 0.4f;
   private static final float steerGain_Lifter = 0.3f;

   // Beacon button pusher servo increment rate
   private static final double beaconPusherRate = 0.005;

   private sensorColor beaconColor = sensorColor.unknown;
   int pictureNumber;

   int state = 0;
   float shooterPower = -0.28f;
   boolean firstLastRightButton = false;
   boolean firstButtonRight = false;
   boolean firstLastLeftButton = false;
   boolean firstButtonLeft = false;

   boolean secondLastRightButton = false;
   boolean secondButtonRight = false;

   //these variables set positions for the Loader servo.
   float basePos = 0.0f;
   float Pos = basePos;

   // These variables are for the Vuforia navigation
   OpenGLMatrix pose = null;

   final int stopCount = 200;
   int pic = -1;
   boolean seeable;
   double kP = 0.0025;
   double power = 0.5;
   double effort;
   int error;
   double leftPower;
   double rightPower;
   boolean spinnerFlag = false;
   boolean shooterFlag = false;
   VuforiaTrackables beacons;

   boolean runShooter = false;

   ColorSensor sensorRGBright;
   ColorSensor sensorRGBleft;

   float hsvValuesright[] = {0F, 0F, 0F};
   float hsvValuesleft[] = {0F, 0F, 0F};

   VectorF translation;

   /***
    *This method inits all hardware on the robot as well as the camera on the drivers'
    * phone.
    */
   public void init()
   {
       //This line inits all robot hardware
      robot.init(hardwareMap);

      // This makes the Vuforia picture appear on the screen
       //of the robot phone
      VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
      // Sets camera direction
      params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
      // License key
      params.vuforiaLicenseKey = "AU2Vspr/////AAAAGSWZlF6AQEHFh9mbNlt5KlFGl/PX8qeeKea7jh5Xk8Ei573/nsoAjsJu9Cbi2MlRCuEIkZHQJoDGAxXmNgioA+0+DbRC6mG+1QbBu8ACMw0pBk6x3h+wvvqDeyZmjV0Fdji5Bk2bV3AaZ0AanljM2nuosjfFYOeUsoFqjE0+MQfJCOoG2ED2hxhJM88dhMaAH45kQqJ99Pn9c/F8whHUkRLeh71wW3O8qGdHEieX7WQO86VfVadHTrg0Ut8ALwiU/qVqB9pJPn+oVe9rYCixcJztb7XOp4T4Mo0IPUwVtkTUZtZTW1mAOPdbbWx3RX1OohA6q6BBU7ozDdQ1W33/L/mdETevYMf7rKPrb82Zbw8r";
      // Sets the thing you see on the screen.  Could be AXES, TEAPOT, BUILDINGS, or NONE
      params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

      // Makes and instance of Vuforia
      VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
      // Lets VuForia see more than one object at one time
     // Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

      // Makes and loads the beacons data set
      beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
      beacons.get(0).setName("Wheels");
      beacons.get(1).setName("Tools");
      beacons.get(2).setName("Legos");
      beacons.get(3).setName("Gears");

      beacons.activate();

      telemetry.addData("Vuforia initialized", "");

      sensorRGBright = hardwareMap.colorSensor.get("AdafruitRGBright");
      sensorRGBleft = hardwareMap.colorSensor.get("AdafruitRGBleft");
   }


   /***
    * Method is the main loop for manual opmode.  Things in this method are iteratively
    * run until the stop button is pushed.
    */
   public void loop()
   {
      // Calculate and apply motor power to drive wheels
      RunMecanumWheels();

      // Adjust the beacon button servo and the loader servo
      ServiceServos();

      //beaconColor = colorSensor.GetAdafruitColorRight(robot);

      // Set steering to ball lifter driving mode
      if (gamepad1.a)
      {
         robot.setscooperMode();
      }

      // Set steering to beacon driving mode
      if (gamepad1.y)
      {
         robot.setBeaconMode();
      }

      // Set steering to scooper driving mode
      if (gamepad1.x)
      {
         robot.setBallLifterMode();
      }

      // Set steering to scooper driving mode
      if (gamepad1.dpad_left)
      {
         robot.setleftDriveMode();
      }

      // Set steering to scooper driving mode
      if (gamepad1.dpad_right)
      {
         robot.setrightDriveMode();
      }

      // Runs the drive to beacon method
      if (gamepad1.b) {
         driveToBeacon(beacons);
      }
      //runs the cap ball lifter.
      runLifter();

      try {
         runSpinner();
      } catch(InterruptedException e) {
         telemetry.addData("Exception: ", "Interrupted Exception");
      }
      //runs ball shooter
      try {
         runShooterState();
      } catch(InterruptedException e) {
         telemetry.addData("Exception: ", "Interrupted Exception");
      }
      telemetry.addData("Shooter Power", shooterPower);
      telemetry.update();
   }


   /***
    * This method calculates the individual motor powers required to drive the mecanum
    * wheels based off the driver 1 controller.  This drive strategy uses the following
    * joystick assignments
    * <p/>
    * Left stick:
    * forward (+y)   - Forward drive
    * rearward (-y)  - Reverse drive
    * right (+x)     - Strafe right
    * left (-x)      - Strafe left
    * <p/>
    * Right stick:
    * forward (+y)   - Not used
    * rearward (-y)  - Not used
    * right (+x)     - Tank turn right
    * left (-x)      - Tank turn left
    */
   public void RunMecanumWheels()
   {
      double LFPower = 0.0;
      double RFPower = 0.0;
      double LRPower = 0.0;
      double RRPower = 0.0;
      double leftStickY;
      double leftStickX;
      double rightStickX;
      // Calculate motor powers but only if any of the joystick commands are greater then
      // a minimum threshold.  Adjust this threshold if the motor has motion when the joystick
      // is not being used and in the center position.
      if ((Math.abs(gamepad1.left_stick_y) >= joystickThreshold) ||
         (Math.abs(gamepad1.left_stick_x) >= joystickThreshold) ||
         (Math.abs(gamepad1.right_stick_x) >= joystickThreshold))
      {
         leftStickY = robot.ScaleJoystickCommand(gamepad1.left_stick_y);
         leftStickX = robot.ScaleJoystickCommand(gamepad1.left_stick_x);
         rightStickX = robot.ScaleJoystickCommand(gamepad1.right_stick_x);


         // Calculate power for each mecanum wheel based on joystick inputs.  Each power is
         // based on three drive components: forward/reverse, strafe, and tank turn.
          //There are three drive modes. This mode, Beacon, drives the robot with the beacon
          //pusher servo in the front.
         telemetry.addData("Mode: ", "Beacon");
         if (robot.driveMode == Crossfire_Hardware.driveModeEnum.beaconMode)
         {
            LFPower = (forwardPriority * leftStickY) + (strafePriority * leftStickX) - (steerPriority * rightStickX);
            RFPower = (forwardPriority * leftStickY) - (strafePriority * leftStickX) + (steerPriority * rightStickX);
            LRPower = (forwardPriority * leftStickY) - (strafePriority * leftStickX) - (steerPriority * rightStickX);
            RRPower = (forwardPriority * leftStickY) + (strafePriority * leftStickX) + (steerPriority * rightStickX);
         }

         //The scoop drive mode sets the particle ball gatherer as the front of the robot.
         telemetry.addData("Mode: ", "Scoop");
         if (robot.driveMode == Crossfire_Hardware.driveModeEnum.scooperMode)
         {
            LFPower = (forwardGain_Scoop * -leftStickY) - (strafeGain_Scoop * leftStickX) + (steerGain_Scoop * -rightStickX);
            RFPower = (forwardGain_Scoop * -leftStickY) + (strafeGain_Scoop * leftStickX) - (steerGain_Scoop * -rightStickX);
            LRPower = (forwardGain_Scoop * -leftStickY) + (strafeGain_Scoop * leftStickX) + (steerGain_Scoop * -rightStickX);
            RRPower = (forwardGain_Scoop * -leftStickY) - (strafeGain_Scoop * leftStickX) - (steerGain_Scoop * -rightStickX);
         }

         //The strafe drive mode sets the side of the robot with the cap ball lifter
         //as the front. To drive forward, the robot strafes.
         telemetry.addData("Mode: ", "Strafe");
         if (robot.driveMode == Crossfire_Hardware.driveModeEnum.ballLifterMode)
         {
            LFPower = (forwardGain_Lifter * -leftStickX) - (strafeGain_Lifter * -leftStickY) + (steerGain_Lifter * -rightStickX);
            RFPower = (forwardGain_Lifter * -leftStickX) + (strafeGain_Lifter * -leftStickY) - (steerGain_Lifter * -rightStickX);
            LRPower = (forwardGain_Lifter * -leftStickX) + (strafeGain_Lifter * -leftStickY) + (steerGain_Lifter * -rightStickX);
            RRPower = (forwardGain_Lifter * -leftStickX) - (strafeGain_Lifter * -leftStickY) - (steerGain_Lifter * -rightStickX);
            telemetry.addData("leftStickX", leftStickX);
         }

          //Drives only the motors on the left side of the robot
         telemetry.addData("Mode: ", "Left Drive");
         if (robot.driveMode == Crossfire_Hardware.driveModeEnum.leftDrive)
         {
            LFPower = (forwardGain_Scoop * -leftStickX) - (strafeGain_Scoop * -leftStickY) + (steerGain_Scoop * -rightStickX);
            RFPower = 0;
            //RFPower = (forwardGain_Scoop * -leftStickY) + (strafeGain_Scoop * leftStickX) - (steerGain_Scoop * -rightStickX);
            LRPower = (forwardGain_Scoop * -leftStickX) + (strafeGain_Scoop * -leftStickY) + (steerGain_Scoop * -rightStickX);
            RRPower = 0;
            //RRPower = (forwardGain_Scoop * -leftStickY) - (strafeGain_Scoop * leftStickX) - (steerGain_Scoop * -rightStickX);
         }

         //Drives only the motors on the right side of the robot.
         telemetry.addData("Mode" , "Right Drive");
         if (robot.driveMode == Crossfire_Hardware.driveModeEnum.rightDrive)
         {
            LFPower = 0;
//            LFPower = (forwardGain_Scoop * -leftStickY) - (strafeGain_Scoop * leftStickX) + (steerGain_Scoop * -rightStickX);
            RFPower = (forwardGain_Scoop * -leftStickX) + (strafeGain_Scoop * -leftStickY) - (steerGain_Scoop * -rightStickX);
            LRPower = 0;
            //LRPower = (forwardGain_Scoop * -leftStickY) + (strafeGain_Scoop * leftStickX) + (steerGain_Scoop * -rightStickX);
            RRPower = (forwardGain_Scoop * -leftStickX) - (strafeGain_Scoop * -leftStickY) - (steerGain_Scoop * -rightStickX);
         }

         // Find maximum power commanded to all the mecanum wheels.  Using the above power
         // equations, it is possible to calculate a power command greater than 1.0f (100%).
         // We want to find the max value so we can proportionally reduce motor powers.
         double maxPower = Math.max(LFPower, Math.max(RFPower, Math.max(LRPower, RRPower)));

         // If max power is greater than 1.0f (100% command), then proportionally reduce all motor
         // powers by the maximum power calculated.  This will equally reduce all powers so no
         // motor power is clipped and the robot responds predictably to joystick commands.  If we
         // don't, then one or more motor commands will clip, others will not, and the robot will not
         // behave predictably.  The end result of this reduction is the motor requesting max power
         // will set power to 1.0f (100%) and all other powers will be reduced by the same ratio.
         if (Math.abs(maxPower) > 1.0f)
         {
            LFPower /= maxPower; // Shorthand for LFPower = LFPower / maxPower
            RFPower /= maxPower;
            LRPower /= maxPower;
            RRPower /= maxPower;
         }

         // Update motor powers with new value.
         robot.setMecanumPowers(LFPower, RFPower, LRPower, RRPower);
      }
      else
      {
         // Explicitly set powers to zero.  May not be necessary but is good practice.
         robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);
      }
   }


   public void runLifter(){
      // This basically used the right and left triggers to drive the winch motor
      // for the ball lifter.  The right trigger raises the lifer, and the left
      // trigger lowers the lifter.
      if(gamepad2.right_trigger > 0.05 && gamepad2.left_trigger < 0.05) {
         robot.Lifter.setPower(-1 * gamepad2.right_trigger);
      } else if(gamepad2.left_trigger > 0.05 && gamepad2.right_trigger < 0.05) {
         robot.Lifter.setPower(gamepad2.left_trigger);
      } else {
         robot.Lifter.setPower(0.0f);
      }
   }


   //Runs the two rubber coated wheels so they rotate opposite directions and launch
   //the particle balls into the center vortex
   public void runShooterState() throws InterruptedException {
      // Gamepad 1 can modify the speed of the wheels on the fly if need be
      firstButtonRight = gamepad1.right_bumper;
      firstButtonLeft = gamepad1.left_bumper;
      if (firstButtonRight && !firstLastRightButton) {
         if(shooterPower < -1.0f) {
            shooterPower = 0.0f;
         }
         else {
            shooterPower -= 0.01f;
         }
      }
      if (firstButtonLeft && !firstLastLeftButton) {
         if(shooterPower > 0.0f) {
            shooterPower = -1.0f;
         }
         else {
            shooterPower += 0.01f;
         }
      }
      firstLastRightButton = firstButtonRight;
      firstLastLeftButton = firstButtonLeft;

      // Gamepad 2 turns the wheels on and off
      secondButtonRight = gamepad2.right_bumper;
      if(secondButtonRight && !secondLastRightButton) {
         runShooter = !runShooter;
      }
      if(runShooter) {
         robot.Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         robot.Shooter.setPower(shooterPower);
      }
      if(!runShooter) {
         robot.Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         robot.Shooter.setPower(0.0f);
      }
      secondLastRightButton = secondButtonRight;
   }
   // This is an unused method currently.  It has been phased out by the runShooterState() method
   public void runShooter() throws InterruptedException{
      if(gamepad2.right_bumper) {
         while(gamepad2.right_bumper)
         {
            idle();
         }
         if(shooterFlag)
         {
            robot.Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Shooter.setPower(-0.2f);
         }
         if(!shooterFlag)
         {
            robot.Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Shooter.setPower(0.0f);
         }
         shooterFlag = !shooterFlag;
      }
      telemetry.addData("Speed", shooterPower);
   }

   //Runs the particle ball gatherer on the front of the robot.
   public void runSpinner() throws InterruptedException {

      if(gamepad2.left_bumper)
      {
         while (gamepad2.left_bumper)
         {
            Pos = basePos + 0.1f;
            idle();
         }
         if (spinnerFlag)
         {
            robot.Spinner.setPower(1.0f);
         }
         if (!spinnerFlag)
         {
            robot.Spinner.setPower(0.0f);
         }
         spinnerFlag = !spinnerFlag;
      }
   }

   /***
    * This method operates the servo to push the beacon button.  To push
    * left-hand button, press and hold x to rotate serve CCW. To push
    * right-hand button, press and hold b button to rotate servo CW.
    * This method was created to use Vuforia to drive the robot towards a beacon and
    * press teh team's corresponding color. However, it is not currently in use.
    */

   //Sets controls and positions for both the Button Pusher servo and the Loader servo.
   private void ServiceServos()
   {
      double ButtonPusherPosition = robot.GetButtonPusherPosition();
      // Rotate CCW
      if (gamepad2.x && robot.GetButtonPusherPosition() <= 0.70)
      {
         robot.SetButtonPusherPosition(ButtonPusherPosition + beaconPusherRate);
      }

      // Rotate CW
      else if (gamepad2.b && robot.GetButtonPusherPosition() >= 0.29)
      {
         robot.SetButtonPusherPosition(ButtonPusherPosition - beaconPusherRate);
      }
      //telemetry.addData("Pos: ", robot.GetButtonPusherPosition());

      // The servo we are using on the robot is NOT a true continuous rotation
      // servo.  It is a <i>winch<i> servo, and so it rotates about 6.5 rotations
      // (with our hardware.  It CAN go farther, however.)  It also has feedback,
      // which makes it basically the same as a normal servo, however it's range is *much*
      // more sensitive.  So, after a little experimentation, I found that position 0 through position
      // 0.12 is about 270 deg. of rotation, which is <i>about<i> what we want for our
      // ball stuffer.  So, what this bit will do is start at a base case(whatever our
      // default servo position will be), and then increment that base case slowly, thus moving the
      // servo slowly.  It also will limit the servo to the ~270 degrees of rotation allowable.

      if (gamepad2.dpad_up)
      {
         //robot.Loader.setPosition(0.50);

         // The amount to add is a magic number, and can be adjusted as seen fit :)
         if(Pos < basePos + 0.1)
         {
            Pos += 0.001f;
            //telemetry.addData("Pos: ", Pos);
         }
         else
         {
            Pos = basePos + 0.1f;
            //telemetry.addData("Pos: ", Pos);
         }
      }
      if (gamepad2.dpad_down)
      {
        //robot.Loader.setPosition(0.85);

         // The amount to subtract is a magic number, and can be adjusted as seen fit :)
         if(Pos > basePos)
         {
            Pos -= 0.001f;
         }
         else {
            Pos = basePos;
         }
      }
      if(gamepad2.y) {
         Pos = basePos + 0.1f;
      }
      if(gamepad2.a) {
         Pos = basePos;
      }
      robot.Loader.setPosition(Pos);
   }

    //After the robot can "See" the picture, it will drive to it.
   private void driveToBeacon(VuforiaTrackables picsArray)
   {
      int x = stopCount + 1;
      int y;
      if(checkForStop())
      {
         requestOpModeStop();
      }
      // Determines which picture it is looking at
      else if(((VuforiaTrackableDefaultListener) picsArray.get(0).getListener()).isVisible()) {
         pic = 0;
         seeable = true;
      }
      else if(((VuforiaTrackableDefaultListener) picsArray.get(1).getListener()).isVisible()) {
         pic = 1;
         seeable = true;
      }
      else if(((VuforiaTrackableDefaultListener) picsArray.get(2).getListener()).isVisible()) {
         pic = 2;
         seeable = true;
      }
      else if(((VuforiaTrackableDefaultListener) picsArray.get(3).getListener()).isVisible()) {
         pic = 3;
         seeable = true;
      } else {
         pic = -1;
         seeable = false;
      }
      telemetry.addData("pic", pic);
      telemetry.update();
      // If it sees a picture, it drives to said picture
      if(pic != -1) {
         seeable = ((VuforiaTrackableDefaultListener) picsArray.get(pic).getListener()).isVisible();
         while (gamepad1.b && !checkForStop() && seeable) {
            pose = ((VuforiaTrackableDefaultListener) picsArray.get(pic).getListener()).getRawPose();
            ServiceServos();
            if (checkForStop()) {
               requestOpModeStop();
            }
            telemetry.clearAll();
            telemetry.addData("visible", "visible");
            telemetry.addData("xValue: ", x);

            if (pose != null) {
               // This is the PID controller
               translation = pose.getTranslation();
               y = (int) translation.get(1);
               x = (int) translation.get(2);
               error = y + 10;
               effort = kP * error;
               rightPower = -1 * (power + effort);
               leftPower = -1 * (power - effort);
               robot.MotorMecanumLeftFront.setPower(leftPower);
               robot.MotorMecanumRightFront.setPower(rightPower);
               robot.MotorMecanumLeftRear.setPower(leftPower);
               robot.MotorMecanumRightRear.setPower(rightPower);
            }
            seeable = false;
            seeable = ((VuforiaTrackableDefaultListener) picsArray.get(pic).getListener()).isVisible();
         }

      }

      // Turns drive motors off when done driving to the picture
      robot.MotorMecanumLeftFront.setPower(0.0f);
      robot.MotorMecanumRightFront.setPower(0.0f);
      robot.MotorMecanumLeftRear.setPower(0.0f);
      robot.MotorMecanumRightRear.setPower(0.0f);
      rightPower = 0;
      leftPower = 0;
      pose = null;
      seeable = false;
   }

   // Checks for a requested stop
   private boolean checkForStop()
   {

      if(gamepad1.back)
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   // This is a method to drive to a certain encoder count
   public void encoderMove(int countLeft, int countRight, double leftPower, double rightPower)
   {
      robot.MotorMecanumLeftFront.setPower(leftPower);
      robot.MotorMecanumRightFront.setPower(rightPower);
      robot.MotorMecanumLeftRear.setPower(leftPower);
      robot.MotorMecanumRightRear.setPower(rightPower);

      setMode(DcMotor.RunMode.RUN_TO_POSITION);

      robot.MotorMecanumLeftFront.setTargetPosition(countLeft);
      robot.MotorMecanumRightFront.setTargetPosition(countRight);
      robot.MotorMecanumLeftRear.setTargetPosition(countLeft);
      robot.MotorMecanumRightRear.setTargetPosition(countRight);

      while(robot.MotorMecanumLeftFront.isBusy() && robot.MotorMecanumRightFront.isBusy() && robot.MotorMecanumLeftRear.isBusy() && robot.MotorMecanumRightRear.isBusy())
      {
         double leftPos = robot.MotorMecanumLeftFront.getCurrentPosition();
         double rightPos = robot.MotorMecanumRightFront.getCurrentPosition();
         telemetry.addData("Right",rightPos);
         telemetry.addData("Left",leftPos);
         try
         {
            idle();
         }
         catch(InterruptedException e)
         {
            telemetry.addData("Idle Failed", "Idle Failed");
         }

      }
      setPower(0.0f);
      setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   }


   // Sets the mode of the drive motors
   public void setMode(DcMotor.RunMode mode)
   {
      robot.MotorMecanumLeftFront.setMode(mode);
      robot.MotorMecanumRightFront.setMode(mode);
      robot.MotorMecanumLeftRear.setMode(mode);
      robot.MotorMecanumRightRear.setMode(mode);
   }


   // Sets the power of the drive motors
   public void setPower(float power)
   {
      robot.MotorMecanumLeftFront.setPower(power);
      robot.MotorMecanumRightFront.setPower(power);
      robot.MotorMecanumLeftRear.setPower(power);
      robot.MotorMecanumRightRear.setPower(power);
   }


   // Idles
   public final void idle() throws InterruptedException
   {
      // Abort the OpMode if we've been asked to stop
      if (checkForStop())
         throw new InterruptedException();

      // Otherwise, yield back our thread scheduling quantum and give other threads at
      // our priority level a chance to run
      Thread.yield();
   }
}
