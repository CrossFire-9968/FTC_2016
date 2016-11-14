// IF OPTICAL SENSOR IS ABOUT 1 INCH OFF THE GROUND
package org.firstinspires.ftc.teamcode;

//import android.app.Activity;
import android.graphics.Color;
//import android.view.View;

//import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
//import com.qualcomm.robotcore.hardware.DigitalChannelController;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


public class CF_SensorLibrary
{
//   // Beacon hue thresholds
//   private static final int RedUpperLimit = 360;
//   private static final int RedLowerLimit = 325;
//   private static final int BlueUpperLimit = 270;
//   private static final int BlueLowerLimit = 220;
//
//   public enum sensorColor { unknown, blue, red }
//
////   private static final int onLine = 2;
////   private static final int offLine = 2;
////   OpticalDistanceSensor odsSensor1;
////   OpticalDistanceSensor odsSensor2;
//
//   // we assume that the LED pin of the RGB sensor is connected to
//   // digital port 5 (zero indexed).
//   static final int LED_CHANNEL = 5;


   /***
    * Method determines the color of the beacon by evaluating the hue value returned
    * by the RGBToHSV method.  If the hue falls within a valid range, the method return
    * an enumeration of Blue or Red, otherwise it is Unknown.
    */
//   public sensorColor GetAdafruitColor(Crossfire_Hardware robot)
//   {
//      sensorColor color = sensorColor.unknown;
//
//      // hsvValues is an array that will hold the hue, saturation, and value information.
//      float hsvValues[] = {0F, 0F, 0F};
//
//      // convert the RGB values to HSV values.
//      Color.RGBToHSV((robot.sensorRGB.red() * 255) / 800, (robot.sensorRGB.green() * 255) / 800, (robot.sensorRGB.blue() * 255) / 800, hsvValues);
//
//      // Determine sensor color based on thresholds
//      if ((hsvValues[0] >= BlueLowerLimit) && (hsvValues[0] <= BlueUpperLimit))
//      {
//         color = sensorColor.blue;
//      }
//      else if ((hsvValues[0] >= RedLowerLimit) && (hsvValues[0] <= RedUpperLimit))
//      {
//         color = sensorColor.red;
//      }
//
//      return(color);
//   }


//   public void getLineData()
//   {
//      if ((odsSensor1.getRawLightDetected() >= onLine) && (odsSensor2.getRawLightDetected() >= onLine))
//      {
//         telemetry.addData("Robot is", " on line.");
//      }
//
//      else if ((odsSensor1.getRawLightDetected() < offLine) && (odsSensor2.getRawLightDetected() >= onLine))
//      {
//         telemetry.addData("Off", "line to the left");  // Motor power something
//      }
//
//      else if ((odsSensor2.getRawLightDetected() < offLine) && (odsSensor1.getRawLightDetected() >= onLine))
//      {
//         telemetry.addData("Off", "line to the right"); // Motor power something
//      }
//
//      else
//      {
//         telemetry.addData("Robot is", " lost :(");
//      }
//   }

//   public void runMotors()
//   {
//      if ((odsSensor1.getRawLightDetected() >= onLine) && (odsSensor2.getRawLightDetected() >= onLine))
//      {
//         rightMotor.setPower(0.08f);
//         leftMotor.setPower(0.08f);
//      }
//
//      else if ((odsSensor1.getRawLightDetected() < offLine) && (odsSensor2.getRawLightDetected() >= onLine))
//      {
//         rightMotor.setPower(0.14f);
//         leftMotor.setPower(0.05f);
//      }
//
//      else if ((odsSensor2.getRawLightDetected() < offLine) && (odsSensor1.getRawLightDetected() >= onLine))
//      {
//         rightMotor.setPower(0.05f);
//         leftMotor.setPower(0.14f);
//      }
//
//      else
//      {
//         rightMotor.setPower(0.0f);
//         leftMotor.setPower(0.0f);
//      }
//   }
}


