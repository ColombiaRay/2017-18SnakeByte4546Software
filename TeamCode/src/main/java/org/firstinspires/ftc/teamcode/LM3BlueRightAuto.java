package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/29/2017.
 */

//@Autonomous (name = "[LM3]BlueRightAuto")
public class LM3BlueRightAuto extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        public void runOpMode() throws InterruptedException {
            setAlliance('b');
            initialize();
            waitForStart();
            scanImage();
            lowerJewel();
            knockJewel();
            //moveBackwardPID(500,0.0013, 0.0000007, 0.5);
            moveBackward(0.00);
            pidTurnLeft(90);
            sleep(650);
            // sleep(735);
            setZero();
            sleep(200);
            scoreGlyphB();
    }
}
