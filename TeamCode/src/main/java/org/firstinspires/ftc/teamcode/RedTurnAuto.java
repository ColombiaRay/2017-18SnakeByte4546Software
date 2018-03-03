package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 11/30/2017.
 */

//@Autonomous
public class RedTurnAuto extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setInitialAngle();
        waitForStart();
        //scanImage();
        while(opModeIsActive()){
            setPower(0,getStrafeCorrection(0),0);
        }
    }
}
