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
        moveForwardPID(325,0.001, 0.0000007, 0.5);
        sleep(500);
        //These are the proper strafe
        //moveStrafe(-0.6, 300);
        //moveStrafe(-0.6, 560);
        moveStrafe(-0.6, 875);
    }
}
