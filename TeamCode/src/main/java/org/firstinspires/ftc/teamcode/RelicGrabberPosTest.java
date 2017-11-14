package org.firstinspires.ftc.teamcode;

/**
 * Created by raymo on 11/13/2017.
 */

public class RelicGrabberPosTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("initalize","done");
        telemetry.update();
        RelicGrabber.setPosition(0.57);
        sleep(4000);
        RelicGrabber.setPosition(1);
        sleep(4000);
    }
}
