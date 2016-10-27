// IF ABOUT 1 INCH OFF THE GROUND

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Anne on 10/14/2016.
 */

@TeleOp(name = "Follow line", group = "Sensor")
//@Disabled

public class CF_FollowLine extends LinearOpMode
{
   private static final int onLine = 2;
   private static final int offLine = 2;
   public DcMotor  leftMotor = null;
   public DcMotor  rightMotor = null;
   OpticalDistanceSensor odsSensor1;
   OpticalDistanceSensor odsSensor2;

   @Override
   public void runOpMode() throws InterruptedException
   {
      odsSensor1 = hardwareMap.opticalDistanceSensor.get("ods1");
      odsSensor2 = hardwareMap.opticalDistanceSensor.get("ods2");
      leftMotor   = hardwareMap.dcMotor.get("left_drive");
      rightMotor  = hardwareMap.dcMotor.get("right_drive");
      leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
      rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

      // Set all motors to zero power
      leftMotor.setPower(0.0f);
      rightMotor.setPower(0.0f);

      waitForStart();
      // while the op mode is active, loop and read the light levels.
      // Note we use opModeIsActive() as our loop condition because it is an interruptable method.

      while (opModeIsActive())
      {
         this.getLineData();
         this.runMotors();
         telemetry.update();
         idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
      }
   }


   public void getLineData()
   {
      if ((odsSensor1.getRawLightDetected() >= onLine) && (odsSensor2.getRawLightDetected() >= onLine))
      {
         telemetry.addData("Robot is" , " on line.");
      }

      else if ((odsSensor1.getRawLightDetected() < offLine) && (odsSensor2.getRawLightDetected() >= onLine))
      {
         telemetry.addData("Off", "line to the left");  // Motor power something
      }

      else if ((odsSensor2.getRawLightDetected() < offLine) && (odsSensor1.getRawLightDetected() >= onLine))
      {
         telemetry.addData("Off" , "line to the right"); // Motor power something
      }

      else
      {
         telemetry.addData("Robot is" , " lost :(");
      }
   }

   public void runMotors()
   {
      if ((odsSensor1.getRawLightDetected() >= onLine) && (odsSensor2.getRawLightDetected() >= onLine))
      {
         leftMotor.setPower(0.1f);
         rightMotor.setPower(0.1f);
      }

      else if ((odsSensor1.getRawLightDetected() < offLine) && (odsSensor2.getRawLightDetected() >= onLine))
      {
         leftMotor.setPower(0.17f);
         rightMotor.setPower(0.05f);
      }

      else if ((odsSensor2.getRawLightDetected() < offLine) && (odsSensor1.getRawLightDetected() >= onLine))
      {
         leftMotor.setPower(0.05f);
         rightMotor.setPower(0.17f);
      }

      else
      {
         leftMotor.setPower(0.0f);
         rightMotor.setPower(0.0f);
      }
   }
}