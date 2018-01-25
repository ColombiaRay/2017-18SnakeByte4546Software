package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/29/2017.
 */

@Autonomous (name = "[LM3]RedAutoRight")
public class LM3RedAutoRight extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setAlliance('r');
        initialize();
        waitForStart();
       // scanImage();
       // lowerJewel();
        //knockJewel();
        sleep(500);
        moveForwardPID(690,0.0013, 0.0000007, 0.5);
        scoreGlyph();
    }
}
