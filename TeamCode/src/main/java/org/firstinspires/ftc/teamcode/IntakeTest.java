package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/16/2017.
 */
@Autonomous
public class IntakeTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        //pidTurnLeft(90 + getGyroYaw());

        //moveToRightColumnRTurn(40);
        moveForwardSt(0.3, 7900,-90);
    }
}
