package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.TimestampedI2cData;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;
/**
 * Created by raymo on 12/11/2017.
 */
import com.qualcomm.robotcore.hardware.Servo;

public class RelicArm{

    private Servo relicArm;
    private double raiseTime = 0;
    private double lowerTime = 0;
    private final double DELAY_TIME_MS = 150;



    public RelicArm(Servo rAS){
        relicArm = rAS;
    }

    public void setRelicArmPos(boolean raiseButton, boolean lowerButton) throws InterruptedException {
        raiseArm(raiseButton);
        lowerArm(lowerButton);
    }

    public void raiseArm(boolean button) throws InterruptedException {
        if ((button) && (System.currentTimeMillis() - raiseTime > DELAY_TIME_MS)){
            relicArm.setPosition(0.25);
            while (relicArm.getPosition() < 0.65){
                relicArm.setPosition(relicArm.getPosition() + 0.05);
                Thread.sleep(100);
            }
            raiseTime = System.currentTimeMillis();
        }
    }

    public void lowerArm(boolean button) throws InterruptedException {
        if ((button) && (System.currentTimeMillis() - raiseTime > DELAY_TIME_MS)){
            relicArm.setPosition(0.65);
            while (relicArm.getPosition() > 0.25){
                relicArm.setPosition(relicArm.getPosition() - 0.05);
                Thread.sleep(100);
            }
            lowerTime = System.currentTimeMillis();
        }
    }
}
