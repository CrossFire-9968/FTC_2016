package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
public class Crossfire_Hardware_Test
{
    public Servo Loader;
    //public DcMotor BallLifter;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();


    /***
     * Initialize standard Hardware interfaces
     *
     * @param ahwMap
     */
    public void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;

        Loader = hwMap.servo.get("loader");
        SetLoaderPosition(0.0f);
    }


    /***
     * This method sets the position of the ball loader servo.  The driver
     * holds down the controller button (digital) which increases or
     * decreases the position value.  This method checks that the driver didn't fall
     * asleep holding the button making the servo rotate to a position it obviously
     * should go.  The limits are magic numbers by Lauren...<sigh>!
     *
     * @param servoPositionDesired Desired position for beacon button pusher servo
     */
    public void SetLoaderPosition(double servoPositionDesired)
    {
        double servoPositionActual = Range.clip(servoPositionDesired, 0.00, 1.00);
        //Loader.setPosition(servoPositionActual);
    }


}


