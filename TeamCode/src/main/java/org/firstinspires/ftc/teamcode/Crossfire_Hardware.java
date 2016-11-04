package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

   /* local OpMode members. */
   HardwareMap hwMap = null;
   private ElapsedTime period = new ElapsedTime();

   /* Constructor */
   public Crossfire_Hardware()
   {

   }


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

      // Set motor polarity.  We are using
      // AndyMark motors so directions are opposite.
      MotorMecanumLeftFront.setDirection(DcMotor.Direction.REVERSE);     // Set to REVERSE if using AndyMark motors
      MotorMecanumLeftRear.setDirection(DcMotor.Direction.REVERSE);      // Set to REVERSE if using AndyMark motors
      MotorMecanumRightFront.setDirection(DcMotor.Direction.FORWARD);    // Set to FORWARD if using AndyMark motors
      MotorMecanumRightRear.setDirection(DcMotor.Direction.FORWARD);     // Set to FORWARD if using AndyMark motors

      // Set all motors to zero power
      setMecanumPowers(0.0f, 0.0f, 0.0f, 0.0f);
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
   public void setMecanumPowers(float LFPower, float RFPower, float LRPower, float RRPower)
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
}

