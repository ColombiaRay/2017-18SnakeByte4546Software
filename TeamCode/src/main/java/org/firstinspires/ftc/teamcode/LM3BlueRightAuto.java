package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/29/2017.
 */

@Autonomous (name = "[LM3]BlueRightAuto")
public class LM3BlueRightAuto extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

            setAlliance('b');
            initialize();
            waitForStart();
            scanImage();
            lowerJewel();
            knockJewel();
            sleep(500);
            moveBackward(0.35);
            sleep(1100);
            setZero();
            pidTurnLeft(90 + getGyroYaw());
            sleep(300);
            moveToRightBlueRTurnColumn();
            shootAndStrafe();
    }
}
