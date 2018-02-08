package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by raymo on 10/25/17.
 */
@Autonomous
public class ColorTest extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //lowerJewel();
        double redCol = 0;
        double blueCol = 0;
        for (int i = 0; i < 1000; i++){
            redCol += colorFront.red();
            blueCol += colorFront.blue();
            telemetry.addData(redCol + "", blueCol + "");
            telemetry.addData("Read time", i);
            telemetry.update();

        }
    }
}
