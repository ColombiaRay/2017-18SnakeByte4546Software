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
        moveToRightColumnRTurn(59);
        sleep(3000);
        moveToRightColumnRTurn(42);
        sleep(3000);
        moveToRightColumnRTurn(26);
    }
}
