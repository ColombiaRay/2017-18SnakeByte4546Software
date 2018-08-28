package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by ashakarmakar on 1/20/2018.
 */

@Autonomous
public class TunnelTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        shootGlyph(3000);
    }
}
