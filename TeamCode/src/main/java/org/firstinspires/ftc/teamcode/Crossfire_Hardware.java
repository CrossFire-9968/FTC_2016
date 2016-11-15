package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
   /* Public OpMode members. */
   public DcMotor MotorMecanumLeftFront;
   public DcMotor MotorMecanumRightFront;
   public DcMotor MotorMecanumLeftRear;
   public DcMotor MotorMecanumRightRear;
   public Servo ButtonPusher;
   public ColorSensor sensorRGB;

   /* local OpMode members. */
   HardwareMap hwMap = null;
   private ElapsedTime period = new ElapsedTime();


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
      sensorRGB = hwMap.colorSensor.get("AdafruitRGB");

      // Set motor polarity.  We are using
      // AndyMark motors so directions are opposite.
      MotorMecanumLeftFront.setDirection(DcMotor.Direction.REVERSE);     // Set to REVERSE if using AndyMark motors
      MotorMecanumLeftRear.setDirection(DcMotor.Direction.REVERSE);      // Set to REVERSE if using AndyMark motors
      MotorMecanumRightFront.setDirection(DcMotor.Direction.FORWARD);    // Set to FORWARD if using AndyMark motors
      MotorMecanumRightRear.setDirection(DcMotor.Direction.FORWARD);     // Set to FORWARD if using AndyMark motor
      SetButtonPusherPosition(0.45);

       //Set all motors to zero power
      setMecanumPowers(0.0f, 0.0f, 0.0f, 0.0f);
      GetButtonPusherPosition();
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
      MotorMecanumLeftFront.setPower(Math.abs(LFPower));
      MotorMecanumRightFront.setPower(Math.abs(RFPower));
      MotorMecanumLeftRear.setPower(Math.abs(LRPower));
      MotorMecanumRightRear.setPower(Math.abs(RRPower));
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
    * @param RRcound Right rear encoder counts
    */
   public void setMecanumEncoderTargetPosition(int LFcount, int RFcount, int LRcount, int RRcound)
   {
      // Only want to use absolute values.  Take abs of inputs in case user sent negative value.
      MotorMecanumLeftFront.setTargetPosition(LFcount);
      MotorMecanumRightFront.setTargetPosition(RFcount);
      MotorMecanumLeftRear.setTargetPosition(LRcount);
      MotorMecanumRightRear.setTargetPosition(RRcound);
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
    */
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
}


