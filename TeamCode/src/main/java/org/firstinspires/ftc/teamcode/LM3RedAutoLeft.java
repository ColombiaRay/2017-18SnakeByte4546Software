package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 12/29/2017.
 */

@Autonomous
public class LM3RedAutoLeft extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        setAlliance('r');
        initialize();
        waitForStart();
        scanImage();
        lowerJewel();
        knockJewel();
        sleep(500);
        strafeToRedColumnTurn();
        unclampGlyph();
        backUpFromGlyph();
    }
}
