// IF ABOUT 1 INCH OFF THE GROUND

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Anne on 10/14/2016.
 */
@TeleOp(name = "Sensor: Distance", group = "Sensor")
//@Disabled

public class CF_FollowLine extends LinearOpMode
{
   private static final int onLine = 2;
   private static final int offLine = 2;

   OpticalDistanceSensor odsSensor1;  // Hardware Device Object
   OpticalDistanceSensor odsSensor2;

   @Override
   public void runOpMode() throws InterruptedException
   {

      // get a reference to our Light Sensor object.
      odsSensor1 = hardwareMap.opticalDistanceSensor.get("ods1");
      odsSensor2 = hardwareMap.opticalDistanceSensor.get("ods2");

      // wait for the start button to be pressed.
      waitForStart();

      // while the op mode is active, loop and read the light levels.
      // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
      while (opModeIsActive())
      {
         this.getLineData();

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
         telemetry.addData("Off", "line");  // Motor power something
      }

      else if ((odsSensor2.getRawLightDetected() < offLine) && (odsSensor1.getRawLightDetected() >= onLine))
      {
         telemetry.addData("Off" , "line"); // Motor power something
      }

      else
      {
         telemetry.addData("Robot is" , " lost :(");
      }
   }
}