package org.firstinspires.ftc.teamcode;

/**
 * Created by raymo on 11/19/2017.
 */

//Quick test for the 11/20 build day to see whether it is viable to knock off the jewel via turning
public class BalanceBoardTurningTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        turnLeftPID(20);
        setZero();
        sleep(500);
        turnRightPID(20);
        sleep(2000);
        turnRightPID(20);
        setZero();
        sleep(500);
        turnLeftPID(20);
    }
}
