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
    int AutoFlag = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private enum states
    {
        AUTOINIT, DRIVETOPOSITION, STARTBALLSHOOTER, STOPBALLSHOOTER, DRIVETOBALL, AUTOCOMPLETE, END
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);
        TimeUnit.MILLISECONDS.sleep(500);
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
                    AutoFlag = 1;
                    telemetry.addData("AutoFlag = " , "1");
                    telemetry.update();
                    robot.Loader.setPosition(0.08);
                    State = states.DRIVETOPOSITION;
                    TimeUnit.MILLISECONDS.sleep(500);
                    break;
                case DRIVETOPOSITION:
                    AutoFlag = 2;
                    telemetry.addData("AutoFlag = " , "2");
                    telemetry.update();
                    robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.setMecanumEncoderTargetPosition(2200, 2200, 2200, 2200);
                    robot.setMecanumPowers(0.4, 0.4, 0.4, 0.4);
                    robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    State = states.STARTBALLSHOOTER;
                    TimeUnit.MILLISECONDS.sleep(500);
                    break;
//                case STARTBALLSHOOTER:
//                    AutoFlag = 3;
//                    telemetry.addData("AutoFlag = " , "3");
//                    telemetry.update();
//                    robot.Shooter.setPower(-0.45f);
//                    TimeUnit.SECONDS.sleep(2);
//                    robot.SetLoaderPosition(0.0);
//                    TimeUnit.SECONDS.sleep(4);
//                    State = states.STOPBALLSHOOTER;
//                    break;
//                case STOPBALLSHOOTER:
//                    AutoFlag = 4;
//                    telemetry.addData("AutoFlag = " , "4");
//                    telemetry.update();
//                    robot.Shooter.setPower(0.0f);
//                    State = states.DRIVETOBALL;
//                    break;
//                case DRIVETOBALL:
//                    AutoFlag = 5;
//                    telemetry.addData("AutoFlag = " , "5");
//                    telemetry.update();
//                    robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //                    robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    robot.setMecanumEncoderTargetPosition(800, 800, 800, 800);
//                    robot.setMecanumPowers(0.4, 0.4, 0.4, 0.4);
//                    robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    State = states.END;
//                    break;
//                case END:
//                    AutoFlag = 6;
//                    telemetry.addData("AutoFlag = " , "6");
//                    telemetry.update();
//                    // Set motors to run by encoders and turn off power
//                    robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);
//                    robot.Shooter.setPower(0.0);
//
//                    telemetry.clear();
//                    telemetry.addData("State: ", "Auto Done");
//                    telemetry.update();
//                    requestOpModeStop();
//                    break;
            }
        }
    }
}
