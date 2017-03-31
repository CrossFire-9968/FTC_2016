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

   private enum states {
      AUTOINIT, DRIVETOPOSITION, STARTBALLSHOOTER, STOPBALLSHOOTER, DRIVETOBALL, AUTOCOMPLETE, END
   }

   @Override
   public void runOpMode() throws InterruptedException {
      robot.init(hardwareMap);
      waitForStart();
      //        TimeUnit.MILLISECONDS.sleep(500);
      // Wait for the game to start (driver presses PLAY)
      states State = states.AUTOINIT;
      runtime.reset();

      // run until the end of the match (driver presses STOP)
      while (opModeIsActive())
      {
         // Initialization routines
         switch (State)
         {
            case AUTOINIT:
               robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               AutoFlag = 1;
               telemetry.addData("AutoFlag = ", "1");
               //robot.Loader.setPosition(0.08);
               TimeUnit.MILLISECONDS.sleep(500);
               robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
               robot.setMecanumEncoderTargetPosition(2200, 2200, 2200, 2200);
               robot.setMecanumPowers(0.4, 0.4, 0.4, 0.4);
               State = states.DRIVETOPOSITION;
               break;
            case DRIVETOPOSITION:
               AutoFlag = 2;
               telemetry.addData("AutoFlag = ", "2");

               telemetry.addData("LF:", robot.MotorMecanumLeftFront.getCurrentPosition());
               telemetry.addData("RF:", robot.MotorMecanumRightFront.getCurrentPosition());
               telemetry.addData("LR:", robot.MotorMecanumLeftRear.getCurrentPosition());
               telemetry.addData("RR:", robot.MotorMecanumRightRear.getCurrentPosition());

               telemetry.update();

               if (!robot.MotorMecanumLeftFront.isBusy())
               {
                  State = states.STARTBALLSHOOTER;
               }

               break;
            case STARTBALLSHOOTER:
               AutoFlag = 3;
               telemetry.addData("AutoFlag = " , "3");
               telemetry.update();
               robot.Shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               robot.Shooter.setTargetPosition(5000);
               robot.Shooter.setPower(-0.30);
               TimeUnit.SECONDS.sleep(1);
               robot.SetLoaderPosition(0.0);
               TimeUnit.SECONDS.sleep(3);
               State = states.STOPBALLSHOOTER;
               break;
            case STOPBALLSHOOTER:
               AutoFlag = 4;
               telemetry.addData("AutoFlag = " , "4");
               telemetry.update();
               robot.Shooter.setPower(0.0f);
               robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
               robot.setMecanumEncoderTargetPosition(2100, 2100, 2100, 2100);
               robot.setMecanumPowers(0.4, 0.4, 0.4, 0.4);
               State = states.DRIVETOBALL;
               break;
            case DRIVETOBALL:
               AutoFlag = 5;
               telemetry.addData("AutoFlag = " , "5");
               telemetry.update();

               if (!robot.MotorMecanumLeftFront.isBusy())
               {
                  State = states.END;
               }
               break;
            case END:
               AutoFlag = 6;
               telemetry.addData("AutoFlag = " , "6");
               telemetry.update();
//                   Set motors to run by encoders and turn off power
               robot.setMecanumPowers(0.0, 0.0, 0.0, 0.0);
               //robot.Shooter.setPower(0.0);
               telemetry.clear();
               telemetry.addData("LF:", robot.MotorMecanumLeftFront.getCurrentPosition());
               telemetry.addData("RF:", robot.MotorMecanumRightFront.getCurrentPosition());
               telemetry.addData("LR:", robot.MotorMecanumLeftRear.getCurrentPosition());
               telemetry.addData("RR:", robot.MotorMecanumRightRear.getCurrentPosition());
               telemetry.addData("State: ", "Auto Done");
               //requestOpModeStop();
               break;
         }
         telemetry.update();
         idle();
      }
   }
}
