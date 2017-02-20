package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

/**
 * Created by dawson on 2/6/2017.
 */

@Autonomous(name = "Backup_Auto", group = "Autonomous")
//@Disabled
public class CF_BackupAuto extends LinearOpMode
{
    Crossfire_Hardware robot = new Crossfire_Hardware();

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private enum states
    {
        AUTOINIT, DRIVETOPOSITION, STARTBALLSHOOTER, STOPBALLSHOOTER, DRIVETOBALL, AUTOCOMPLETE, END
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Wait for the game to start (driver presses PLAY)
        states State = states.AUTOINIT;
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Initialization routines
            switch (State)
            {
                case AUTOINIT:
                    robot.init(hardwareMap);
                    robot.Loader.setPosition(0.08);
                    State = states.DRIVETOPOSITION;
                    break;
                case DRIVETOPOSITION:
                    robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.setMecanumEncoderTargetPosition(2200, 2200, 2200, 2200);
                    robot.setMecanumPowers(0.4, 0.4, 0.4, 0.4);
                    State = states.STARTBALLSHOOTER;
                    break;
                case STARTBALLSHOOTER:
                    robot.setShooterEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.Shooter.setPower(-0.45f);
                    TimeUnit.SECONDS.sleep(2);
                    robot.SetLoaderPosition(0.0);
                    TimeUnit.SECONDS.sleep(4);
                    State = states.STOPBALLSHOOTER;
                    break;
                case STOPBALLSHOOTER:
                    robot.Shooter.setPower(0.0f);
                    State = states.DRIVETOBALL;
                    break;
                case DRIVETOBALL:
                    robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.setMecanumEncoderTargetPosition(800, 800, 800, 800);
                    robot.setMecanumPowers(0.4, 0.4, 0.4, 0.4);
                    State = states.END;
                    break;
                case END:
                    // Set motors to run by encoders and turn off power
                    robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);
                    robot.Shooter.setPower(0.0);

                    telemetry.clear();
                    telemetry.addData("State: ", "Auto Done");
                    telemetry.update();
                    requestOpModeStop();
            }
        }
    }
}
