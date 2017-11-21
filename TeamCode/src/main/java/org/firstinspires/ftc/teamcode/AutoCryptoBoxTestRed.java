package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/20/2017.
 */

@Autonomous
public class AutoCryptoBoxTestRed extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        moveForwardPID(275,7);
        brake();
        sleep(500);
        moveStrafeLeftPID(400, 1);
    }
}
