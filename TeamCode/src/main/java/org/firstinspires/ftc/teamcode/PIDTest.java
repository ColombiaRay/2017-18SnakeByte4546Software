package org.firstinspires.ftc.teamcode;

/**
 * Created by raymo on 11/13/2017.
 */

public class PIDTest extends AutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        moveForwardPID(500);
        moveBackwardPID(500);
    }
}
