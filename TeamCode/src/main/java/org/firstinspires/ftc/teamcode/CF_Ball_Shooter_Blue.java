package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

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

        final float speed = 0.50f;
        boolean breakLoop = false;
        boolean rfBusy;
        boolean rrBusy;
        boolean lfBusy;
        boolean lrBusy;

        botState State = botState.FIRSTSTRAFE;
        initalize();

        while(opModeIsActive() && !breakLoop){
            rfBusy = false;
            rrBusy = false;
            lfBusy = false;
            lrBusy = false;

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
                    robot.Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.Shooter.setPower(-0.60f);
                    TimeUnit.SECONDS.sleep(2);
                    robot.SetLoaderPosition(0.007);

                    TimeUnit.SECONDS.sleep(6);
                    State = botState.BALLTWO;
                    break;
                case BALLTWO:
                    robot.SetLoaderPosition(0.0f);
                    TimeUnit.SECONDS.sleep(2);
                    robot.Shooter.setPower(0.0f);
                    breakLoop = true;
                    break;
                default:
                    breakLoop = true;
                    break;
                    
            }

            // Telemetry showing what state we are currently in
            telemetry.addData("State: ", State);

            // Telemetry showing if wheels are busy
            if (robot.MotorMecanumLeftFront.isBusy())
            {
                lfBusy = true;
            }

            if (robot.MotorMecanumLeftRear.isBusy())
            {
                lrBusy = true;
            }

            if (robot.MotorMecanumRightFront.isBusy())
            {
                rfBusy = true;
            }

            if (robot.MotorMecanumRightRear.isBusy()) {
                rrBusy = true;
            }

            telemetry.addData("lfBusy: ",lfBusy);
            telemetry.addData("rfBusy: ",rfBusy);
            telemetry.addData("lrBusy: ",lrBusy);
            telemetry.addData("rrBusy: ",rrBusy);
            telemetry.update();
        }
        requestOpModeStop();
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
