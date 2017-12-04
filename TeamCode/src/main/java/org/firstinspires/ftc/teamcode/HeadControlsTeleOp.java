package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by raymo on 10/4/17.
 */

@TeleOp (name = "HeadShouldersKneesAndToes", group = "TeleOp")
public class HeadControlsTeleOp extends OpMode {

    private double rightMotion;
    private double leftMotion;
    private double leftRelicPosition;
    private double rightRelicPosition;

    private boolean halfSpeed;

    private boolean liftOut;

    private long currentTime;
    private long lastTime;
    private long closeTime;

    // Drive Train
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private boolean braked;

    private Servo gate;
    private DcMotor leftLiftSlide;
    private DcMotor rightLiftSlide;
    private DcMotor liftMani;

    // Matt Tunnel
    private DcMotor liftLeft;
    private DcMotor liftRight;
    private DcMotor inTake;
    private CRServo frontRightTunnel;
    private CRServo backRightTunnel;
    private CRServo frontLeftTunnel;
    private CRServo backLeftTunnel;

    private Servo leftMani;
    private Servo rightMani;

    private Servo jewelHitter;
    private Servo leftArm;
    private Servo rightArm;
    private Servo leftRelic;
    private Servo rightRelic;

    private MattTunnel tunnel;
    private Servo relicGrabber;
    private Servo relicClamp;
    private Servo relicArm;
    private final int DELAYTOGGLE = 500;
    private boolean relicOut;
    private boolean clampClosed;
    private boolean gateClosed;
    private Servo gateServo;
    private double recentPressTime;
    private boolean relicClosed;


    // added 11/30- for lift motor extend/retract
    private DcMotor relicLift;

    //jewelstate 0 is upright, 1 is near upright, 2 is position to hit jewel
    String jewelState = "down";

    @Override
    public void init() {
        /*
        Expansion Hub 1: Motor 0 is BR, Motor 1 is FR, Motor 2 is BL, Motor 3 is FL
        Expansion Hub 1: Servo 0 is RRelicArm, Servo 1 is LRelioArm, Servo 2 is LRelio, Servo 3 is RRelic
        Expansion Hub 1: Servo 4 is LMani, Servo 5 is RMani
        Expansion Hub 2: Motor 0 is LSlide, Motor 1 is liftMani, Motor 2 is RSlide
        Expansion Hub 2: Servo 0 is JewelHitter
        */

        FL                  = hardwareMap.dcMotor.get("FL");
        FR                  = hardwareMap.dcMotor.get("FR");
        BR                  = hardwareMap.dcMotor.get("BR");
        BL                  = hardwareMap.dcMotor.get("BL");
        //leftArm             = hardwareMap.servo.get("LRelicArm");
        //rightArm            = hardwareMap.servo.get("RRelicArm");
        //leftRelic           = hardwareMap.servo.get("LRelic");
        //rightRelic          = hardwareMap.servo.get("RRelic");
        //leftMani            = hardwareMap.servo.get("LMani");
        //rightMani           = hardwareMap.servo.get("RMani");
        //leftLiftSlide       = hardwareMap.dcMotor.get("LSlide");
        //rightLiftSlide      = hardwareMap.dcMotor.get("RSlide");
        //liftMani            = hardwareMap.dcMotor.get("liftMani");
        jewelHitter         = hardwareMap.servo.get("jewelhitter");
        //liftLeft            = hardwareMap.dcMotor.get("leftLift");
        //liftRight           = hardwareMap.dcMotor.get("rightLift");
        inTake              = hardwareMap.dcMotor.get("intake");

        frontRightTunnel    = hardwareMap.crservo.get("FRT");
        backRightTunnel     = hardwareMap.crservo.get("BRT");
        frontLeftTunnel     = hardwareMap.crservo.get("FLT");
        backLeftTunnel      = hardwareMap.crservo.get("BLT");
        recentPressTime = System.currentTimeMillis();
        //gate = hardwareMap.servo.get("gate");

        //relicGrabber        = hardwareMap.servo.get("relicGrabber");
        relicArm            = hardwareMap.servo.get("relicArm");
        halfSpeed           = false;
        braked              = false;
        liftOut             = false;
        relicOut            = false;
        clampClosed         = false;
        gateClosed            = false;
        relicClosed       = false;
        rightMotion         = 0;
        leftMotion          = 0;
        tunnel              = new MattTunnel(liftLeft,liftRight, inTake, frontRightTunnel, backRightTunnel, frontLeftTunnel, backLeftTunnel);
        relicGrabber        = hardwareMap.servo.get("relic");
        currentTime = 0;
        //relicClamp          = hardwareMap.servo.get("relicClamp");

        relicLift           = hardwareMap.dcMotor.get("relicLift");

        //rightMani.setDirection(Servo.Direction.FORWARD);
        //leftMani.setDirection(Servo.Direction.REVERSE);
        jewelHitter.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Initialization", "done");
        telemetry.update();

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        grabRelic();

        lowerRelicArm();
        setPower();
        toggleHalfSpeed();
        //useJewel();
        tunnel.toggleInTake(gamepad2.left_stick_y);
        //tunnel.manipulateLift(gamepad2.right_stick_y); // TODO: add gamepad contols for these methods
        //if(jewelHitter.getPosition() != 1.0) jewelHitter.setPosition(1.0);
        manipLift();
    }


    public double getRightVelocity()
    {
        if (Math.abs(gamepad1.right_stick_y) > 0.05)
            return gamepad1.right_stick_y;
        return 0;
    }

    public double getLeftVelocity()
    {
        if (Math.abs(gamepad1.left_stick_y) > 0.05)
            return gamepad1.left_stick_y;
        return 0;
    }

    public double getLeftShoulder()
    {
        if (gamepad1.left_bumper)
            return 1.0;
        return 0;
    }

    public void useJewel(){
        if (gamepad1.dpad_left){
            jewelHitter.setPosition(0);
        }
        if (gamepad1.dpad_right){
            jewelHitter.setPosition(0.57);
        }
    }


    public void toggleHalfSpeed() {
        currentTime = System.currentTimeMillis();
        if (gamepad1.x && (currentTime - lastTime) < DELAYTOGGLE) {
            if (halfSpeed) {
                halfSpeed = false;
                telemetry.addData("halfspeed", "disabled");
                telemetry.update();
            }
            else if (!halfSpeed) {
                halfSpeed = true;
                telemetry.addData("halfspeed", "enabled");
                telemetry.update();
            }
            lastTime = System.currentTimeMillis();
        }
    }


    public double getRightShoulder()
    {
        if (gamepad1.right_bumper)
            return 1.0;
        return 0;
    }

    public double getHalfSpeed(){
        if (halfSpeed)
            return 0.5;
        return 1;

    }

    public void grabRelic(){
        if (System.currentTimeMillis() - recentPressTime > 300) {
            if (gamepad2.x) {
                if (relicClosed){
                    relicGrabber.setPosition(1);
                    relicClosed = false;
                }
                else if (!relicClosed){
                    relicGrabber.setPosition(0);
                    relicClosed = true;
                }
            }
            recentPressTime = System.currentTimeMillis();
        }
    }


    public void lowerRelicArm(){
        if (gamepad2.right_bumper) {
            relicArm.setPosition(0.3);
        }
        else if (gamepad2.left_bumper) {
            relicArm.setPosition(1);
        }
        else {
            relicArm.setPosition(0.5);
        }
    }

    public void manipLift() {
        if (gamepad2.dpad_up) {
            relicLift.setPower(1);
        } else if (gamepad2.dpad_down) {
            relicLift.setPower(-1);
        }
        else {
            relicLift.setPower(0);
        }
    }

    /*
    public void useGate() {
        if ((gamepad2.b) && ( System.currentTimeMillis() - currentTime < 500)) {
            if (!gateClosed) {
                gate.setPosition(0.35);
                gateClosed = true;
            } else if (gateClosed) {
                gate.setPosition(0.7);
                gateClosed = false;
            }
            currentTime = System.currentTimeMillis();
        }
    }
    */


    //Jewel Hitter (have not tested positions)



    public void setPower() {
        FL.setPower(getHalfSpeed()*(getLeftVelocity() - getLeftShoulder() + getRightShoulder()));
        FR.setPower(getHalfSpeed()*(-getRightVelocity() + getLeftShoulder() - getRightShoulder()));
        BL.setPower(getHalfSpeed()*(getLeftVelocity() + getLeftShoulder() - getRightShoulder()));
        BR.setPower(getHalfSpeed()*(-getRightVelocity() - getLeftShoulder() + getRightShoulder()));
    }


    //Has some potential for balancing
    public void brake(){
        if (gamepad2.left_bumper) {
            if (!braked) {
                FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                braked = true;
            }
            else {
                FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                braked = false;
            }
        }
    }




    /* Methods for original clamp bot (obsolete)
    //relic lift
    //Relic
    public void setArmPower() {
        if (gamepad2.right_stick_y < -0.2) {
            leftArm.setPosition(1);
            rightArm.setPosition(0);
        }
        else if (gamepad2.right_stick_y > 0.2){
            leftArm.setPosition(0);
            rightArm.setPosition(1);
        }
        else{
            leftArm.setPosition(0.5);
            rightArm.setPosition(0.5);
        }
    }

    public void setLiftSlide(){
        if (gamepad2.left_stick_y < -0.1){
            leftLiftSlide.setPower(-1);
            rightLiftSlide.setPower(1);
        }
        else if (gamepad2.left_stick_y > 0.1){
            rightLiftSlide.setPower(-1);
            leftLiftSlide.setPower(1);
        }
        else{
            rightLiftSlide.setPower(0);
            leftLiftSlide.setPower(0);
        }
    }

    public void useRelicGrabber() {
        if (gamepad2.right_stick_y > 0.05){
            relicGrabber.setPosition(1);
            relicOut = true;
        }
        if (gamepad2.right_stick_y < -0.05){
            relicGrabber.setPosition(0);
            relicOut = false;
        }
        if ((gamepad2.right_stick_button) && (relicOut) && (!clampClosed)){
            relicClamp.setPosition(1);
            clampClosed = true;
        }
        if ((gamepad2.right_stick_button) && (relicOut) && (clampClosed)){
            relicClamp.setPosition(1);
            clampClosed = false;
        }

    }

    //clamp for glyphs
    public void setManiPower(){
        if (gamepad2.b){
            leftMani.setPosition(1);
            rightMani.setPosition(1);
            closeTime = System.currentTimeMillis();
        }
        else if (System.currentTimeMillis() - closeTime < 1000){
            leftMani.setPosition(0.3);
            rightMani.setPosition(0.3);
        }
        else{
            leftMani.setPosition(0.5);
            rightMani.setPosition(0.5);
        }
    }

    public void grapRelic() {
        if(gamepad2.x) {
            leftRelicPosition += .005;
            rightRelicPosition -= .005;
            leftRelic.setPosition(leftRelicPosition);
            rightRelic.setPosition(rightRelicPosition);
        }
        if(gamepad2.y) {
            leftRelicPosition += .005;
            leftRelicPosition -= .005;
            leftRelic.setPosition(leftRelicPosition);
            rightRelic.setPosition(rightRelicPosition);
        }
    }

    public void pickRelic(){
        if(gamepad2.x) {
            leftRelic.setPosition(1);
            rightRelic.setPosition(0);
        }
        else if (gamepad2.y){
            leftRelic.setPosition(0);
            rightRelic.setPosition(1);
        }
    }

    public void raiseMani() {
        if (gamepad2.dpad_down){
            liftMani.setPower(1);
        }
        else if (gamepad2.dpad_up){
            liftMani.setPower(-1);
        }
        else{
            liftMani.setPower(0);
        }
    }
    */

}