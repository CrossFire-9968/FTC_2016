package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Anne on 10/10/2016.
 */


/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Optical Distance Sensor
 * It assumes that the ODS sensor is configured with a name of "ods sensor".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensor: MR ODS", group = "Sensor")
//@Disabled

public class CF_SensorMROpticalDistance extends LinearOpMode
{

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

         // send the info back to driver station using telemetry function.
         telemetry.addData("Sensor 1: Raw",    odsSensor1.getRawLightDetected());
         telemetry.addData("Sensor 1: Normal", odsSensor1.getLightDetected());
         telemetry.addData("Sensor 2: Raw",    odsSensor2.getRawLightDetected());
         telemetry.addData("Sensor 2: Normal", odsSensor2.getLightDetected());

         telemetry.update();
         idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
      }
   }
}