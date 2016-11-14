package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
   /* Public OpMode members. */
   public DcMotor MotorMecanumLeftFront;
   public DcMotor MotorMecanumRightFront;
   public DcMotor MotorMecanumLeftRear;
   public DcMotor MotorMecanumRightRear;
   public Servo ButtonPusher;
   public Servo Flicker;
   public ColorSensor sensorRGB;

   /* local OpMode members. */
   HardwareMap hwMap = null;
   private ElapsedTime period = new ElapsedTime();

   public enum driveModeEnum {beaconMode, ballKickerMode};

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

      // Define and Initialize Motors
      MotorMecanumLeftFront = hwMap.dcMotor.get("left_front_drive");
      MotorMecanumRightFront = hwMap.dcMotor.get("right_front_drive");
      MotorMecanumLeftRear = hwMap.dcMotor.get("left_rear_drive");
      MotorMecanumRightRear = hwMap.dcMotor.get("right_rear_drive");
      ButtonPusher = hwMap.servo.get("button_pusher");
      Flicker = hwMap.servo.get("flicker");
      sensorRGB = hwMap.colorSensor.get("AdafruitRGB");

      // Set motor polarity.  We are using
      // AndyMark motors so directions are opposite.
      MotorMecanumLeftFront.setDirection(DcMotor.Direction.REVERSE);     // Set to REVERSE if using AndyMark motors
      MotorMecanumLeftRear.setDirection(DcMotor.Direction.REVERSE);      // Set to REVERSE if using AndyMark motors
      MotorMecanumRightFront.setDirection(DcMotor.Direction.FORWARD);    // Set to FORWARD if using AndyMark motors
      MotorMecanumRightRear.setDirection(DcMotor.Direction.FORWARD);     // Set to FORWARD if using AndyMark motor
      SetButtonPusherPosition(0.45);
      SetFlickerPosition(0.55);

      // Set all motors to zero power
      setMecanumPowers(0.0f, 0.0f, 0.0f, 0.0f);
      GetButtonPusherPosition();
      GetFlickerPosition();

      setBeaconMode();
   }


   public void SetButtonPusherPosition(double servoPositionDesired)
   {
      double servoPositionActual = Range.clip(servoPositionDesired, 0.28, 0.70);
      ButtonPusher.setPosition(servoPositionActual);
   }


   public double GetButtonPusherPosition()
   {
      double position = 0.0;

      if (ButtonPusher != null)
      {
         position = ButtonPusher.getPosition();
      }
      return position;
   }

   public void SetFlickerPosition(double servoPositionDesired)
   {
      double servoPositionActual = Range.clip(servoPositionDesired, 0.00, 1.00);
      Flicker.setPosition(servoPositionActual);
   }

   public double GetFlickerPosition()
   {
      double position = 0.0;

      if (Flicker != null)
      {
         position = Flicker.getPosition();
      }
      return position;
   }

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


   public void setBallKickerMode()
   {
      driveMode = driveModeEnum.ballKickerMode;
//      MotorMecanumLeftFront.setDirection(DcMotor.Direction.FORWARD);     // Set to REVERSE if using AndyMark motors
//      MotorMecanumLeftRear.setDirection(DcMotor.Direction.FORWARD);      // Set to REVERSE if using AndyMark motors
//      MotorMecanumRightFront.setDirection(DcMotor.Direction.REVERSE);    // Set to FORWARD if using AndyMark motors
//      MotorMecanumRightRear.setDirection(DcMotor.Direction.REVERSE);     // Set to FORWARD if using AndyMark motor
   }


   public void setBeaconMode()
   {
      driveMode = driveModeEnum.beaconMode;
//      MotorMecanumLeftFront.setDirection(DcMotor.Direction.REVERSE);     // Set to REVERSE if using AndyMark motors
//      MotorMecanumLeftRear.setDirection(DcMotor.Direction.REVERSE);      // Set to REVERSE if using AndyMark motors
//      MotorMecanumRightFront.setDirection(DcMotor.Direction.FORWARD);    // Set to FORWARD if using AndyMark motors
//      MotorMecanumRightRear.setDirection(DcMotor.Direction.FORWARD);     // Set to FORWARD if using AndyMark motor
   }


   public double ScaleJoystickCommand(double input)
   {
      double scaledInput;
      final int numPointsInMap = 16;

      // Ensure the values make sense.  Clip the values to max/min values
      double clippedPower = Range.clip(input, -1, 1);

      // Array used to map joystick input to motor output
      double[] scalingArray = {0, 0.01, 0.02, 0.04, 0.05, 0.08, 0.11,
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


