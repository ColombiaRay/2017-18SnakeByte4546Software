package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 *
 * Created by raymo on 12/29/2017.
 */

@Autonomous (name = "WRONGJEWEL!BlueLeft")
public class RPBlueLeft extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setAlliance('b');
        initialize();
        setAlliance('r');
        waitForStart();
        scanImage();
        // scanImage();
        lowerJewel();
        knockJewel();
        //moveBackwardPID(500,0.0013, 0.0000007, 0.5);
        moveBackward(0.35);
        sleep(650);
        // sleep(735);
        setZero();
        sleep(200);
        scoreGlyphB();
    }
}
