package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 * <p/>
 * This class can be used to define all the specific hardware to the CrossFire robot.
 * <p/>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p/>
 * Motor channel:  Left front drive motor:      "left_front_drive"
 * Motor channel:  Right front drive motor:     "right_front_drive"
 * Motor channel:  Left rear drive motor:       "left_rear_drive"
 * Motor channel:  Right rear drive motor:      "right_rear_drive"
 */
public class Crossfire_Hardware
{
   //Sets all the existing hardware parts on the robot
   //into code.
   public DcMotor MotorMecanumLeftFront;
   public DcMotor MotorMecanumRightFront;
   public DcMotor MotorMecanumLeftRear;
   public DcMotor MotorMecanumRightRear;
   public DcMotor Spinner;
   public DcMotor Lifter;
   public DcMotor Shooter;
   public Servo Loader;
   public Servo ButtonPusher;
   public ColorSensor sensorRGBright;
   public ColorSensor sensorRGBleft;

   /* local OpMode members. */
   HardwareMap hwMap = null;
   private ElapsedTime period = new ElapsedTime();

   public enum driveModeEnum {beaconMode, ballLifterMode, scooperMode};

   public enum sensorColor { blue, red, unknown };

   driveModeEnum driveMode = driveModeEnum.beaconMode;


   /***
    * Initialize standard Hardware interfaces
    *
    * @param ahwMap
    */
   public void init(HardwareMap ahwMap)
   {
      // Save reference to Hardware map
      hwMap = ahwMap;

      // Define and Initialize Motors; sets names for configuration
      MotorMecanumLeftFront = hwMap.dcMotor.get("left_front_drive");
      MotorMecanumRightFront = hwMap.dcMotor.get("right_front_drive");
      MotorMecanumLeftRear = hwMap.dcMotor.get("left_rear_drive");
      MotorMecanumRightRear = hwMap.dcMotor.get("right_rear_drive");
      ButtonPusher = hwMap.servo.get("button_pusher");
      sensorRGBright = hwMap.colorSensor.get("AdafruitRGBright");
      sensorRGBleft = hwMap.colorSensor.get("AdafruitRGBleft");
      Spinner= hwMap.dcMotor.get("spinner");
      Loader = hwMap.servo.get("loader");
      Shooter = hwMap.dcMotor.get("shooter");
      Lifter = hwMap.dcMotor.get("ball_lifter");

      // Sets the polarity of each of the motors.
      MotorMecanumLeftFront.setDirection(DcMotor.Direction.REVERSE);     // Set to REVERSE if using AndyMark motors
      MotorMecanumLeftRear.setDirection(DcMotor.Direction.REVERSE);      // Set to REVERSE if using AndyMark motors
      MotorMecanumRightFront.setDirection(DcMotor.Direction.FORWARD);    // Set to FORWARD if using AndyMark motors
      MotorMecanumRightRear.setDirection(DcMotor.Direction.FORWARD);  // Set to FORWARD if using AndyMark motor
      Shooter.setDirection(DcMotor.Direction.FORWARD);
      Spinner.setDirection(DcMotor.Direction.FORWARD);
      Lifter.setDirection(DcMotorSimple.Direction.FORWARD);

      SetButtonPusherPosition(0.45);
      SetLoaderPosition(0.05f);

       //Set all motors to zero power
      setMecanumPowers(0.0f, 0.0f, 0.0f, 0.0f);
      GetButtonPusherPosition();

      // Initialize driver controls to beacon mode or ball kicker mode.
      // Comment out the mode you don't want to start in.
      setBeaconMode();
      //setBallKickerMode();
   }


   /***
    * This method sets the position of the beacon button pusher servo.  The drive
    * holds down the controller button controller button (digital) which increases or
    * decreases the position value.  This method checks that the driver didn't fall
    * asleep holding the button making the servo rotate to a position it obviously
    * should go.  The limits are magic numbers by Lauren...<sigh>!
    *
    * @param servoPositionDesired Desired position for beacon button pusher servo
    */
   public void SetButtonPusherPosition(double servoPositionDesired)
   {
      double servoPositionActual = Range.clip(servoPositionDesired, 0.28, 0.70);
      ButtonPusher.setPosition(servoPositionActual);
   }


   /***
    * If you want to know where the beacon button servo has gone, this is the method for you.
    *
    * @return Relative position of servo, range is 0 to 1
    */
   public double GetButtonPusherPosition()
   {
      double position = 0.0;

      if (ButtonPusher != null)
      {
         position = ButtonPusher.getPosition();
      }
      return position;
   }

   /***
    * This method sets the position of the ball loader servo.  The driver
    * holds down the controller button (digital) which increases or
    * decreases the position value.  This method checks that the driver didn't fall
    * asleep holding the button making the servo rotate to a position it obviously
    * should go.  The limits are magic numbers by Lauren...<sigh>!
    *
    * @param servoPositionDesired Desired position for beacon button pusher servo
    */
   public void SetLoaderPosition(double servoPositionDesired)
   {
      double servoPositionActual = Range.clip(servoPositionDesired, 0.00, 1.00);
      Loader.setPosition(servoPositionActual);
   }

   /***
    * If you want to know where the beacon button servo has gone, this is the method for you.
    *
    * @return Relative position of servo, range is 0 to 1
    */
//   public double GetLoaderPosition()
//   {
//      double position = 0.0;
//
//      if (Loader != null)
//      {
//         position = Loader.
//      }
//      return position;
//   }

   /***
    * Convenience method to assign motor power for mecanum drive.  Each mecanum
    * wheel is independently driven.
    *
    * @param LFPower Left front mecanum wheel
    * @param RFPower Right front mecanum wheel
    * @param LRPower Left rear mecanum wheel
    * @param RRPower Right Rear mecanum wheel
    */
   public void setMecanumPowers(double LFPower, double RFPower, double LRPower, double RRPower)
   {
      MotorMecanumLeftFront.setPower(LFPower);
      MotorMecanumRightFront.setPower(RFPower);
      MotorMecanumLeftRear.setPower(LRPower);
      MotorMecanumRightRear.setPower(RRPower);
   }


   /***
    * Convenience method for setting encoder runmode for all four mecanum drive motors
    *
    * @param mode Encoder run mode
    */
   public void setMecanumEncoderMode(DcMotor.RunMode mode)
   {
      MotorMecanumLeftFront.setMode(mode);
      MotorMecanumRightFront.setMode(mode);
      MotorMecanumLeftRear.setMode(mode);
      MotorMecanumRightRear.setMode(mode);
   }


   /***
    * Convenience method for setting encoder counts to all four mecanum drive motors
    *
    * @param LFcount Left front encoder counts
    * @param RFcount Right front encoder counts
    * @param LRcount Left rear encoder counts
    * @param RRcount Right rear encoder counts
    */
   public void setMecanumEncoderTargetPosition(int LFcount, int RFcount, int LRcount, int RRcount)
   {
      // Only want to use absolute values.  Take abs of inputs in case user sent negative value.
      MotorMecanumLeftFront.setTargetPosition(Math.abs(LFcount));
      MotorMecanumRightFront.setTargetPosition(Math.abs(RFcount));
      MotorMecanumLeftRear.setTargetPosition(Math.abs(LRcount));
      MotorMecanumRightRear.setTargetPosition(Math.abs(RRcount));
   }


   /***
    * Method determines if the mecanum motors are still busy trying to reach the target position.
    * As each encoder reaches it's target position, it is run mode is changed to RUN_WITHOUT_ENCODERS.
    * This is to disable the PID position control loop and de-energize the motor.  If this is not
    * done and the motors continue to hold position, we have seen cases where one or more motors
    * remain a few counts away from reaching their target due to having to overcome torque
    * applied by motors holding a static position.  This keeps the state machine from advancing
    * to the next state.  This method is a bit jerky looking at the end of a state but works.
    * Alternatively, you could just watch one motor or a pair (e.g. the fronts) and assume the
    * other wheels are close enough.  This method may not work well if the encoder counts and
    * motor power are not synchronized to turn off at roughly the same time.
    *
    * @return boolean flag TRUE - motors busy or FALSE - meaning all motors have reached target position
    *
   public boolean mecanumMotorsBusy()
   {
      boolean motorsBusy = true;

      // Turn off left front motor when target position reached
      if(!MotorMecanumLeftFront.isBusy())
      {
         MotorMecanumLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      }

      // Turn off right front motor when target position reached
      if(!MotorMecanumRightFront.isBusy())
      {
         MotorMecanumRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      }

      // Turn off left rear motor when target position reached
      if(!MotorMecanumLeftRear.isBusy())
      {
         MotorMecanumLeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      }

      // Turn off right rear motor when target position reached
      if(!MotorMecanumRightRear.isBusy())
      {
         MotorMecanumRightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      }

      // If all the motors are set to RUN_WITHOUT_ENCODERS, then we assume that the robot
      // has reached it's final destination and we can transition to the next state.
      if((MotorMecanumLeftFront.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)   ||
         (MotorMecanumRightFront.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)  ||
         (MotorMecanumLeftRear.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)    ||
         (MotorMecanumRightRear.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER))
      {
         motorsBusy = false;
      }

      return (motorsBusy);
   }


   /***
    * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
    * periodic tick.  This is used to compensate for varying processing times for each cycle.
    * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
    *
    * @param periodMs Length of wait cycle in mSec.
    * @throws InterruptedException
    */
   public void waitForTick(long periodMs) throws InterruptedException
   {

      long remaining = periodMs - (long) period.milliseconds();

      // sleep for the remaining portion of the regular cycle period.
      if (remaining > 0)
      {
         Thread.sleep(remaining);
      }

      // Reset the cycle clock for the next pass.
      period.reset();
   }


   /***
    *
    */
   public void setBallLifterMode()
   {
      driveMode = driveModeEnum.ballLifterMode;
   }

   /***
    *
    */
   public void setscooperMode()
   {
      driveMode = driveModeEnum.scooperMode;
   }

   /***
    *
    */
   public void setBeaconMode()
   {
      driveMode = driveModeEnum.beaconMode;
   }


   /***
    *
    * @param input
    * @return
    */
   public double ScaleJoystickCommand(double input)
   {
      double scaledInput;
      final int numPointsInMap = 25;

      // Ensure the values make sense.  Clip the values to max/min values
      double clippedPower = Range.clip(input, -1, 1);

//      // Array used to map joystick input to motor output
//      double[] scalingArray = {0, 0.01, 0.02, 0.04, 0.05, 0.08, 0.11,
//         0.13, 0.17, 0.23, 0.32, 0.4, 0.48, 0.61, 0.73, 0.89, 1};

            // Array used to map joystick input to motor output
      double[] scalingArray =
         {0, 0.004, 0.0045, 0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04, 0.047, 0.05, 0.065, 0.08, 0.11,
         0.13, 0.17, 0.23, 0.32, 0.4, 0.48, 0.61, 0.73, 0.89, 1};

      // Get the corresponding index for the specified argument/parameter.
      int index = (int) (clippedPower * numPointsInMap);

      // Array indexes can only be positive so we need to drop the negative
      if (index < 0)
      {
         index = -index;
      }

      // Limit indexes to actual size of array so we don't overflow
      if (index > numPointsInMap)
      {
         index = numPointsInMap;
      }

      // Handle negative power values as the table only had positive values
      if (clippedPower < 0)
      {
         scaledInput = -scalingArray[index];
      }
      else
      {
         scaledInput = scalingArray[index];
      }

      return scaledInput;
   }
}


