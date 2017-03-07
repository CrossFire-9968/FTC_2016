package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Ryley on 3/6/17.
 */
public class Crossfire_Hardware_Proggie {

    public DcMotor MotorMecanumLeftFront;
    public DcMotor MotorMecanumRightFront;
    public DcMotor MotorMecanumLeftRear;
    public DcMotor MotorMecanumRightRear;

    HardwareMap hwMap = null;

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
}
