package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Ryley on 3/6/17.
 */
public class Crossfire_Hardware_Proggie {

    public DcMotor MotorMecanumLeftFront;
    public DcMotor MotorMecanumRightFront;
    public DcMotor MotorMecanumLeftRear;
    public DcMotor MotorMecanumRightRear;

    public enum driveModeEnum {beaconMode, ballLifterMode, scooperMode};

    driveModeEnum driveMode = driveModeEnum.beaconMode;


    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        MotorMecanumLeftFront = hwMap.dcMotor.get("left_front_drive");
        MotorMecanumRightFront = hwMap.dcMotor.get("right_front_drive");
        MotorMecanumLeftRear = hwMap.dcMotor.get("left_rear_drive");
        MotorMecanumRightRear = hwMap.dcMotor.get("right_rear_drive");

        MotorMecanumLeftFront.setDirection(DcMotor.Direction.REVERSE);     // Set to REVERSE if using AndyMark motors
        MotorMecanumLeftRear.setDirection(DcMotor.Direction.REVERSE);      // Set to REVERSE if using AndyMark motors
        MotorMecanumRightFront.setDirection(DcMotor.Direction.FORWARD);    // Set to FORWARD if using AndyMark motors
        MotorMecanumRightRear.setDirection(DcMotor.Direction.FORWARD);  // Set to FORWARD if using AndyMark motor


    }
    public void setMecanumPowers(double LFPower, double RFPower, double LRPower, double RRPower)
    {
        MotorMecanumLeftFront.setPower(LFPower);
        MotorMecanumRightFront.setPower(RFPower);
        MotorMecanumLeftRear.setPower(LRPower);
        MotorMecanumRightRear.setPower(RRPower);
    }
    public double ScaleJoystickCommand(double input)
    {
        double scaledInput;
        final int numPointsInMap = 34;

        // Ensure the values make sense.  Clip the values to max/min values
        double clippedPower = Range.clip(input, -1, 1);

//      // Array used to map joystick input to motor output
//      double[] scalingArray = {0, 0.01, 0.02, 0.04, 0.05, 0.08, 0.11,
//         0.13, 0.17, 0.23, 0.32, 0.4, 0.48, 0.61, 0.73, 0.89, 1};

        // Array used to map joystick input to motor output
        double[] scalingArray =
                {0, 0.001, 0.0015, 0.002, 0.0026, 0.003, 0.0036,  0.004, 0.0046, 0.005, 0.058, 0.0063, 0.007, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04, 0.047, 0.05, 0.065, 0.08, 0.11,
                        0.13, 0.17, 0.23, 0.32, 0.4, 0.48, 0.61, 0.73, 0.89, 1};

        // Get the corresponding index for the specified argument/parameter.
        int index = (int) (clippedPower * numPointsInMap);

        // Array indexes can only be positive so we need to drop the negative
        if (index < 0)
        {
            index = -index;
        }

        // Limit indexes to actual size of array so we don't overflow
        if (index > numPointsInMap)
        {
            index = numPointsInMap;
        }

        // Handle negative power values as the table only had positive values
        if (clippedPower < 0)
        {
            scaledInput = -scalingArray[index];
        }
        else
        {
            scaledInput = scalingArray[index];
        }

        return scaledInput;
    }

}
