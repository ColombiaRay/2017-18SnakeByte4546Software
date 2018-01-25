package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/19/2017.
 */

@Autonomous

//Quick test for the 11/20 build day to see whether it is viable to knock off the jewel via turning
public class BalanceBoardTurningTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        turn180();
        sleep(300);
        strafeToColumnPWithRange(82);
    }
}
