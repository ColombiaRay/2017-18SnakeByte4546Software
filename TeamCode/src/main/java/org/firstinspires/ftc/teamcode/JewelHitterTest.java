package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/30/2017.
 */

//@Autonomous
public class JewelHitterTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        colorFront.enableLed(false);
        waitForStart();
        sleep(2000);
        jewelKnocker.setPosition(0.3);
        telemetry.addData("pos", 0.3);
        telemetry.update();
        sleep(2000);
        jewelKnocker.setPosition(0.55);
        telemetry.addData("pos", 0.5);
        telemetry.update();
        sleep(2000);
        jewelKnocker.setPosition(0.8);
        telemetry.addData("pos", 0.7);
        telemetry.update();
        sleep(2000);
    }
}
