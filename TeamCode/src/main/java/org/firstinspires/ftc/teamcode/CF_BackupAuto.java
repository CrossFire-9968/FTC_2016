package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by dawso on 2/6/2017.
 */

@Autonomous(name = "Backup_Auto", group = "Autonomous")
@Disabled
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

            startBallShooter();

            loadBalls();

            stopBallShooter();

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
        telemetry.clear();
        telemetry.clear();
        telemetry.addData("State: ", "init");
        telemetry.update();
    }

    private void startBallShooter()
    {
        robot.Shooter.setPower(-0.4f);
        telemetry.clear();
        telemetry.addData("State: ", "Start Ball Shooter");
        telemetry.update();
    }

    private void loadBalls()
    {
        robot.Loader.setPosition(0);
        telemetry.clear();
        telemetry.addData("State: ", "Load");
        telemetry.update();
    }

    private void stopBallShooter()
    {
        robot.Shooter.setPower(0.0f);
        telemetry.clear();
        telemetry.addData("State: ", "Stop Ball Shooter");
        telemetry.update();
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
        robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);

        telemetry.clear();
        telemetry.addData("State: ", "Auto Done");
        telemetry.update();
    }
}
