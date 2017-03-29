package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Ryley on 3/28/17.
 */

@Autonomous(name = "CF_Encoder_Test", group = "test")
//@Disabled
public class CF_Encoder_Test extends CF_Library_Test{
    @Override public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMove(1000, 1000, 0.2f, 0.2f);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderStrafeLeft(3000, 0.5f);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderStrafeRight(3000, 0.5f);
    }
}
