package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by raymo on 10/16/2017.
 */
@Autonomous(name = "Servo-urban")
public class ServoTest extends LinearOpMode {
    Servo relic;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Servo relicArm;
        relicArm            = hardwareMap.servo.get("relicArm");
        for (double i = 0; i <= 1; i += 0.1){
            telemetry.addData("Pos", i);
            telemetry.update();
            relicArm.setPosition(i);
            //Relic Arm down 0.15, over the wall .65
            sleep(500);
        }


        /*
        //Open Relic (continuous mode)
        relicArm.setPosition(0.7);
        sleep(3000);

        //Close Relic (continuous mode)
        relicArm.setPosition(0.3);
        sleep(3000);
        */
        /*relicArm.setPosition(1);
        sleep(3000);
        relicArm.setPosition(0);
        sleep(3000);
        */
    }
}
