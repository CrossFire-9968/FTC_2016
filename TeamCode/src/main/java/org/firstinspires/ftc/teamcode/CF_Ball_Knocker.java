package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by Ryley on 11/4/16.
 */
@Autonomous(name="CF_Ball_Knocker", group ="Knocker")
//@Disabled

public class CF_Ball_Knocker extends CF_Library implements SensorEventListener
{

   float xAccel = 0;
   float yAccel = 0;
   float zAccel = 0;

   private SensorManager sensorManager;
   private Sensor accelSen;

   @Override
   public void runOpMode() throws InterruptedException
   {
      robot.init(hardwareMap);
      setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        sensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
//        accelSen = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
      waitForStart();

      if (opModeIsActive())
      {
         this.encoderStrafeRight(5000, 0.3f);

      }
   }

   @Override
   public void onAccuracyChanged(Sensor sensor, int accuracy)
   {

   }

   @Override
   public void onSensorChanged(SensorEvent event)
   {
      xAccel = event.values[0];
      yAccel = event.values[1];
      zAccel = event.values[2];
   }


}