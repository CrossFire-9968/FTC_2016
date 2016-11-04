package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * <p/>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p/>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p/>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Crossfire_Hardware
{
   /* Public OpMode members. */
   public DcMotor LeftFrontMotor;
   public DcMotor RightFrontMotor;
   public DcMotor LeftRearMotor;
   public DcMotor RightRearMotor;

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
      LeftFrontMotor = hwMap.dcMotor.get("left_front_drive");
      RightFrontMotor = hwMap.dcMotor.get("right_front_drive");
      LeftRearMotor = hwMap.dcMotor.get("left_rear_drive");
      RightRearMotor = hwMap.dcMotor.get("right_rear_drive");

      // Set motor polarity.  We are using
      // AndyMark motors so directions are opposite.
      LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);     // Set to REVERSE if using AndyMark motors
      LeftRearMotor.setDirection(DcMotor.Direction.REVERSE);      // Set to REVERSE if using AndyMark motors
      RightFrontMotor.setDirection(DcMotor.Direction.FORWARD);    // Set to FORWARD if using AndyMark motors
      RightRearMotor.setDirection(DcMotor.Direction.FORWARD);     // Set to FORWARD if using AndyMark motors

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
      LeftFrontMotor.setPower(LFPower);
      RightFrontMotor.setPower(RFPower);
      LeftRearMotor.setPower(LRPower);
      RightRearMotor.setPower(RRPower);
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

