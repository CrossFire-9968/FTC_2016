package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by jd72958 on 11/14/2016.
 */

@Autonomous(name = "Lauren's_Auto", group = "Jeff")  // @Autonomous(...) is the other common choice
@Disabled
public class CF_ParkOnSquare extends LinearOpMode
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

         // Drive straight forward to line
         autoDriveFromWall();

         // Tank turn until facing beacon
         autoTurnToBall();

         //Drive to ball
         autoHitBall();

         // All done, do nothing, wait for autonomous to end
         autoComplete();
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
      telemetry.addData("State: ", "init");
      telemetry.update();
   }


   /***
    * Drive robot straight forward until it gets to the line leading to beacon
    * @throws InterruptedException
    */
   private void autoDriveFromWall() throws InterruptedException
   {
      // Reset encoder counts and set encoders to use RUN_TO_POSITION MODE
      robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

      // Set target position (negative value is forward rotation)
      robot.setMecanumEncoderTargetPosition(-2000, -2000, -2000, -2000);

      // Set motor powers and start motion
      robot.setMecanumPowers(0.3, 0.3, 0.3, 0.3);
   }
      // Check to see if any of the mecanum drive motors are busy.
      // If not, the we can move to the next state.
//      while (opModeIsActive() && robot.mecanumMotorsBusy())
//      {
//         // Add encoder position to telemetry
//         telemetry.clear();
//         telemetry.addData("State: ", "Drive From Wall");
//         telemetry.update();
//
//         idle();
//      }
//
//      // Turn off motors
//      robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
//   }


//   /***
//    * Tank turn until robot is facing beacon
//    * @throws InterruptedException
//    */
   private void autoTurnToBall() throws InterruptedException
   {
      // Reset encoder counts and set encoders to use RUN_TO_POSITION MODE
      robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

      // Set target position
      robot.setMecanumEncoderTargetPosition(-700, 700, -700, 700);

      // Set motor powers and start motion
      robot.setMecanumPowers(0.3, 0.3, 0.3, 0.3);

      // Check to see if any of the mecanum drive motors are busy.
      // If not, the we can move to the next state.
//      while (opModeIsActive() && robot.mecanumMotorsBusy())
//      {
//         // Add encoder position to telemetry
//         telemetry.clear();
//         telemetry.addData("State: ", "Turn To Ball");
//         telemetry.update();
//
//         idle();
//      }

      // Turn off motors
      robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
   }

   private void autoHitBall() throws InterruptedException
   {
      // Reset encoder counts and set encoders to use RUN_TO_POSITION MODE
      robot.setMecanumEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

      // Set target position
      robot.setMecanumEncoderTargetPosition(-2500, -2500, -2500, -2500);

      // Set motor powers and start motion
      robot.setMecanumPowers(0.3, 0.3, 0.3, 0.3);

      // Check to see if any of the mecanum drive motors are busy.
      // If not, the we can move to the next state.
//      while (opModeIsActive() && robot.mecanumMotorsBusy())
//      {
//         // Add encoder position to telemetry
//         telemetry.clear();
//         telemetry.addData("State: ", "Hit Ball");
//         telemetry.update();
//
//         idle();
//      }

      // Turn off motors
      robot.setMecanumEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
