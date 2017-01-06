package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Ryley on 1/6/17.
 */
@Autonomous(name="CF_Ball_Shooter_Blue", group ="Blue")
//@Disabled
public class CF_Ball_Shooter_Blue extends CF_Library{
    private enum botState {
        FIRSTSTRAFE, BALLONE, BALLTWO, BALLKNOCKER
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.MotorMecanumLeftFront.setDirection(DcMotor.Direction.FORWARD);
        robot.MotorMecanumLeftRear.setDirection(DcMotor.Direction.FORWARD);
        robot.MotorMecanumRightFront.setDirection(DcMotor.Direction.REVERSE);
        robot.MotorMecanumRightRear.setDirection(DcMotor.Direction.REVERSE);

        final float speed = 0.5f;
        boolean breakLoop = false;

        botState State = botState.FIRSTSTRAFE;
        initalize();

        while(opModeIsActive() && !breakLoop){
            switch(State) {
                case FIRSTSTRAFE:
                    System.out.println("FIRST STRAFE");

                    this.encoderStrafeLeft(3000, speed);
                    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.encoderMove(1500, 1500, 0.6f, 0.6f);
                    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.encoderStrafeLeft(2000, speed);

                    State = botState.BALLONE;

                    break;
                case BALLONE:
                    
            }
        }
    }
    private void initalize() throws InterruptedException {
        // Sends telemetry message to signify robot waiting
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // Reset motor encoders
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run using encoders
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Sends telemetry message to signify encoder reset
        if(robot.MotorMecanumLeftFront.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            telemetry.addData("Encoder Reset", "Encoder Reset");
            telemetry.update();
        } else {
            telemetry.addData("Encoder NOT Reset", "Encoder NOT Reset");
        }

        // Wait for start button to be pressed
        waitForStart();
    }
}
