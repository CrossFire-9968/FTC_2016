package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by Ryley on 11/4/16.
 */
@TeleOp(name="CF_Accelerometer", group ="Accel")
//@Disabled

public class CF_Accelerometer extends CF_Library implements SensorEventListener{

    float xAccel = 0;
    float yAccel = 0;
    float zAccel = 0;

    private SensorManager sensorManager;
    private Sensor accelSen;

    @Override
    public void runOpMode ()throws InterruptedException {
        robot.init(hardwareMap);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        sensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
//        accelSen = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        waitForStart();


        //sensorManager.registerListener(this, accelSen, SensorManager.SENSOR_DELAY_NORMAL);

//        while(opModeIsActive()) {
//            while (!isStopRequested()) {
//                for(double i = 0.0; i <= 1.0; i+= 0.001) {
//                    robot.LeftFrontMotor.setPower(i);
//                    robot.RightFrontMotor.setPower(i);
//                    robot.LeftRearMotor.setPower(i);
//                    robot.RightRearMotor.setPower(i);
//                    System.out.println(i + "," + xAccel);
//                }
//            }
//        }
        if(opModeIsActive()) {
            this.encoderStrafeRight(5714, 0.2f);

        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        xAccel = event.values[0];
        yAccel = event.values[1];
        zAccel = event.values[2];
    }


}
