package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/21/2017.
 */

@Autonomous
public class testAutro extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();


        moveBackwardPID(250,0.001, 0.0000007, 0.5);
        sleep(500);
        pidTurnRight(180);
        sleep(500);
        moveStrafeSpecial(0.6,585);
        //moveStrafeSpecial(0.6, 125);
        //moveStrafeSpecial(0.6, 360);
        sleep(500);
        moveForward(0.3,50);







        //Red Turn Auto
        //moveForwardPID(400,0.001, 0.0000007, 0.5);
        //No strafe
        //moveStrafe(-0.6,320);
        //moveStrafe(-0.6,585);

        //Blue Turn Auto
        //moveBackwardPID(250,0.001, 0.0000007, 0.5);
        //No Strafe
        //moveStrafe(0.6,230);
        //moveStrafe(0.6,420);

        //Blue Straight On Auto
        //moveBackwardPID(250,0.001, 0.0000007, 0.5);
        //moveStrafeSpecial(0.6, 125);
        //moveStrafeSpecial(0.6, 360);
        //moveStrafeSpecial(0.6,580);

        //Red Srraight On Auto
        //moveForwardPID(325,0.001, 0.0000007, 0.5);
        //sleep(500);
        //These are the proper strafe
        //moveStrafe(-0.6, 300);
        //moveStrafe(-0.6, 560);
        //moveStrafe(-0.6, 875);
    }
}
