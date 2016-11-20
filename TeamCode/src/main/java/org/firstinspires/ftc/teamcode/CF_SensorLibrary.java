// IF OPTICAL SENSOR IS ABOUT 1 INCH OFF THE GROUND
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import org.firstinspires.ftc.teamcode.Crossfire_Hardware.sensorColor;


public class CF_SensorLibrary
{
   // Beacon hue thresholds
   private static final int RedUpperLimit_lowRange = 20;
   private static final int RedLowerLimit_lowRange = 0;
   private static final int RedUpperLimit_highRange = 360;
   private static final int RedLowerLimit_highRange = 325;
   private static final int BlueUpperLimit = 270;
   private static final int BlueLowerLimit = 220;

   // hsvValues is an array that will hold the hue, saturation, and value information.
   float hsvValues[] = {0F, 0F, 0F};

   sensorColor color = sensorColor.unknown;

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
    *
    * @return color - enumeration of beacon color
    */
   public sensorColor GetAdafruitColor(Crossfire_Hardware robot)
   {
      // convert the RGB values to HSV values.
      Color.RGBToHSV((robot.sensorRGB.red() * 255) / 800, (robot.sensorRGB.green() * 255) / 800, (robot.sensorRGB.blue() * 255) / 800, hsvValues);

      // Determine sensor color based on thresholds
      if ((hsvValues[0] >= BlueLowerLimit) && (hsvValues[0] <= BlueUpperLimit))
      {
         color = sensorColor.blue;
      }
      else if ((hsvValues[0] >= RedLowerLimit_highRange) && (hsvValues[0] <= RedUpperLimit_highRange) ||
               (hsvValues[0] >= RedLowerLimit_lowRange) && (hsvValues[0] <= RedUpperLimit_lowRange))
      {
         color = sensorColor.red;
      }

      return (color);
   }

   public float GetAdafruitHSV(Crossfire_Hardware robot) {
      Color.RGBToHSV((robot.sensorRGB.red() * 255) / 800, (robot.sensorRGB.green() * 255) / 800, (robot.sensorRGB.blue() * 255) / 800, hsvValues);

      return(hsvValues[0]);
   }


   public float GetAdafruitRED(Crossfire_Hardware robot) {
      Color.RGBToHSV((robot.sensorRGB.red() * 255) / 800, (robot.sensorRGB.green() * 255) / 800, (robot.sensorRGB.blue() * 255) / 800, hsvValues);

      return(robot.sensorRGB.red());
   }


   public float GetAdafruitGREEN(Crossfire_Hardware robot) {
      Color.RGBToHSV((robot.sensorRGB.red() * 255) / 800, (robot.sensorRGB.green() * 255) / 800, (robot.sensorRGB.blue() * 255) / 800, hsvValues);

      return(robot.sensorRGB.green());
   }

   public float GetAdafruitBLUE(Crossfire_Hardware robot) {
      Color.RGBToHSV((robot.sensorRGB.red() * 255) / 800, (robot.sensorRGB.green() * 255) / 800, (robot.sensorRGB.blue() * 255) / 800, hsvValues);

      return(robot.sensorRGB.blue());
   }


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


