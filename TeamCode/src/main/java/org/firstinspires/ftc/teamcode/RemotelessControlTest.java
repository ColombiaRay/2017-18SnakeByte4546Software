package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by rubenr on 1/7/18.
 */
@Autonomous (name = "RelicTester")
public class RemotelessControlTest extends AutoOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        for(int i = 0; i < 10; i++){

            relic.setPosition(0);
            Thread.sleep(1000);
            relic.setPosition(1);
            Thread.sleep(1000);
            relicArm.setPosition(1);
            Thread.sleep(1000);
            relicArm.setPosition(0.4);
            Thread.sleep(1000);
            relicArm.setPosition(0.05);
            Thread.sleep(1000);
            tunnel.toggleInTake(-1.0,-1.0);
            Thread.sleep(1000);
            tunnel.toggleInTake(1.0,1.0);
            Thread.sleep(1000);

        }
    }
}


