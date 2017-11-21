package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/20/2017.
 */

@Autonomous
public class AutoCryptoBoxTestBlue extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        moveBackwardPID(300,5.5);
        setZero();
        sleep(2000);
        moveStrafeLeftPID(250,1.5);
        sleep(500);
        brake();
        moveStrafeLeftPID(250,1.5);
        sleep(500);
        brake();
        moveStrafeLeftPID(250,1.5);
        sleep(500);
        brake();
    }
}
