package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by jd72958 on 10/30/2016.
 */

@Autonomous(name = "All Sensors", group = "Sensor")
public class Find_And_Tell_Beacon extends LinearOpMode
{
   private static final int RedUpperLimit = 360;
   private static final int RedLowerLimit = 325;
   private static final int BlueUpperLimit = 270;
   private static final int BlueLowerLimit = 220;
   private static final int onLine = 2;
   private static final int offLine = 2;
   public DcMotor leftMotor = null;
   public DcMotor  rightMotor = null;
   OpticalDistanceSensor odsSensor1;
   OpticalDistanceSensor odsSensor2;

//  CF_SensorAdafruitRGB CF_SensorLib = new CF_SensorAdafruitRGB();

   ColorSensor sensorRGB;
   DeviceInterfaceModule cdim;

   // we assume that the LED pin of the RGB sensor is connected to
   // digital port 5 (zero indexed).
   static final int LED_CHANNEL = 5;

   @Override
   public void runOpMode() throws InterruptedException
   {

      // hsvValues is an array that will hold the hue, saturation, and value information.
      float hsvValues[] = {0F,0F,0F};

      // values is a reference to the hsvValues array.
      final float values[] = hsvValues;

      // get a reference to the RelativeLayout so we can change the background
      // color of the Robot Controller app to match the hue detected by the RGB sensor.
      final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

      // bPrevState and bCurrState represent the previous and current state of the button.
      boolean bPrevState = false;
      boolean bCurrState = false;

      // bLedOn represents the state of the LED.
      boolean bLedOn = true;

      // get a reference to our DeviceInterfaceModule object.
      cdim = hardwareMap.deviceInterfaceModule.get("CF_Dim");

      // set the digital channel to output mode.
      // remember, the Adafruit sensor is actually two devices.
      // It's an I2C sensor and it's also an LED that can be turned on or off.
      cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

      // get a reference to our ColorSensor object.
      sensorRGB = hardwareMap.colorSensor.get("AdafruitRGB");

      // turn the LED on in the beginning, just so user will know that the sensor is active.
      cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);

      odsSensor1 = hardwareMap.opticalDistanceSensor.get("ods1");
      odsSensor2 = hardwareMap.opticalDistanceSensor.get("ods2");
      leftMotor   = hardwareMap.dcMotor.get("left_drive");
      rightMotor  = hardwareMap.dcMotor.get("right_drive");
      leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
      rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

      // Set all motors to zero power
      leftMotor.setPower(0.0f);
      rightMotor.setPower(0.0f);

      // wait for the start button to be pressed.
      waitForStart();

      // loop and read the RGB data.
      // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
      while (opModeIsActive())  {

//      // check the status of the x button on gamepad.
//      bCurrState = gamepad1.x;
//
//      // check for button-press state transitions.
//      if ((bCurrState == true) && (bCurrState != bPrevState))  {
//
//        // button is transitioning to a pressed state. Toggle the LED.
//        bLedOn = !bLedOn;
//        cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);
//      }
//
//      // update previous state variable.
//      bPrevState = bCurrState;

         // convert the RGB values to HSV values.
         Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);

//      Get sensor color
         if ((hsvValues[0] >= BlueLowerLimit) && (hsvValues[0] <= BlueUpperLimit))
         {
            telemetry.addData("Beacon is ", "Blue");
         }

         else if ((hsvValues[0] >= RedLowerLimit) && (hsvValues[0] <= RedUpperLimit))
         {
            telemetry.addData("Beacon is ", "Red");
         }

         else
         {
            telemetry.addData("Beacon is ", "Unknown");
         }


         // change the background color to match the color detected by the RGB sensor.
         // pass a reference to the hue, saturation, and value array as an argument
         // to the HSVToColor method.
         relativeLayout.post(new Runnable() {
            public void run() {
               relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
         });

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
