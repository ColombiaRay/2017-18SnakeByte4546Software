package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/7/2017.
 */
@Autonomous
public class ServoPosTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("initalize","done");
        telemetry.update();
        jewelHitter.setPosition(0.57);
        sleep(4000);
        jewelHitter.setPosition(1);
        sleep(4000);
    }
}
