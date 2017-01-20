package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Ryley on 1/19/17.
 */
@TeleOp(name = "CF_Run", group = "Drivetrain")
//@Disabled
public class CF_Run extends OpMode{
    Crossfire_Hardware robot = new Crossfire_Hardware();

    public void init() {
        robot.init(hardwareMap);
    }

    public void loop() {
        if(gamepad1.a) {
            robot.MotorMecanumLeftFront.setPower(0.5f);
        } else if(!gamepad1.a) {
            robot.MotorMecanumLeftFront.setPower(0.0f);
        }
    }
}
