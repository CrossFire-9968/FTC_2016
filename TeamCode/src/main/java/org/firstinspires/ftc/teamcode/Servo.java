package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by jd72958 on 11/6/2016.
 */

@TeleOp(name = "CF_Servo", group = "Drivetrain")
//Disabled
public class Servo extends OpMode
{
   Crossfire_Hardware robot = new Crossfire_Hardware();

   public void init()
   {
      robot.init(hardwareMap);
   }

   public void loop()
   {
      ServiceServo();
   }

   private void ServiceServo()
   {
      double ButtonPusherPosition = robot.GetButtonPusherPosition();

      if (gamepad1.x)
      {
         robot.SetButtonPusherPosition(ButtonPusherPosition + 0.005);
      }

      else if (gamepad1.b)
      {
         robot.SetButtonPusherPosition(ButtonPusherPosition - 0.005);
      }
   }
}
