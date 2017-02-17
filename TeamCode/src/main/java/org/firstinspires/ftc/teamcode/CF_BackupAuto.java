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

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive())
        {
            // Initialization routines
            autoInit();

            driveToPosition();

            startBallShooter();

            stopBallShooter();

            driveToBall();

            Park();

            autoComplete();

            requestOpModeStop();
        }
    }


    /***
     * Initialization routine.  Nothing much happens here except updating
     * the drivers station that we reached this point.  As other actuators
     * are added, the their initializations would happen here
     * (e.g. initial servo position)
     */

    private void autoInit()
    {
        robot.init(hardwareMap);
        robot.Loader.setPosition(0.08);
        telemetry.clear();
        telemetry.clear();
        telemetry.addData("State: ", "init");
        telemetry.update();
    }

    private void driveToPosition()
    {
        robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setMecanumEncoderTargetPosition(2000, 2000, 2000, 2000);
        robot.setMecanumPowers(0.4, 0.4, 0.4, 0.4);
    }

    private void startBallShooter() throws InterruptedException
    {
        robot.setShooterEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Shooter.setPower(-0.45f);
        TimeUnit.SECONDS.sleep(2);
        robot.SetLoaderPosition(0.0);
        TimeUnit.SECONDS.sleep(4);
        telemetry.clear();
        telemetry.addData("State: ", "Ball Shooter");
        telemetry.update();
    }

    private void stopBallShooter()
    {
        robot.Shooter.setPower(0.0f);
        telemetry.clear();
        telemetry.addData("State: ", "Stop Ball Shooter");
        telemetry.update();
    }

    private void driveToBall()
    {
        robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setMecanumEncoderTargetPosition(300, 300, 300, 300);
        robot.setMecanumPowers(0.4, 0.4, 0.4, 0.4);
    }

    private void Park()
    {
        robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.clear();
        telemetry.addData("State: ", "Park");
        robot.setMecanumPowers(0, 0, 0, 0);
    }

    /***
     * This is the final stage of autonomous.  Once entered, the robot will
     * happily wait until the end and do nothing.  This method can be used
     * to stop the robot if something goes awry or it can be the ending point
     * once all other tasks are complete.
     */
    private void autoComplete()
    {
        // Set motors to run by encoders and turn off power
        robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);
        robot.Shooter.setPower(0.0);

        telemetry.clear();
        telemetry.addData("State: ", "Auto Done");
        telemetry.update();
    }
}
