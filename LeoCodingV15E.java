package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;


import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import java.sql.Time;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Fiducial;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import java.util.Set;

import javax.sql.RowSetEvent;

//this is for the meet 3 bot between meet 3 and 4
//@Disabled
@TeleOp
public class LeoCodingV15E extends LinearOpMode {
    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor left_back;
    private DcMotor right_back;

    public DcMotorEx Launch;
    private DcMotor intake;
    private CRServo side1;
    private CRServo side2;
    private DcMotor Revolver;
    private Servo flick;
//    private DistanceSensor Distance;

    private String color1;
    private String color2;
    private String color3;

    Timer flicktime = new Timer();

//    TimerTask flicktask = new TimerTask();

//    public volatile double colorTime;

//    long colorTime = System.currentTimeMillis();


    private String CanLaunch;
    private String closefar;


//    private DcMotor test;

//    private ColorSensor color;

    private ColorSensor top;

    private ColorSensor top2;

    int revolverpos=1;

    int lastpos=0;
//    private CRServo Stopper;

    private int AprilTagID;
    private Limelight3A limelight;

    GoBildaPrismDriver prism;


    PrismAnimations.Solid purple = new PrismAnimations.Solid(Color.PURPLE);

    PrismAnimations.Solid green = new PrismAnimations.Solid(Color.GREEN);

    PrismAnimations.Solid launchlight = new PrismAnimations.Solid(Color.RED);

//    PrismAnimations.Snakes launchlight = new PrismAnimations.Snakes(Color.RED);

//    PrismAnimations.RainbowSnakes launchlight = new PrismAnimations.RainbowSnakes();


    PrismAnimations.Solid launchlight2 = new PrismAnimations.Solid(Color.RED);

    PrismAnimations.Solid launchlightoff = new PrismAnimations.Solid(new Color(255,255,255));

    PrismAnimations.Solid launchlight2off = new PrismAnimations.Solid(new Color( 255, 255, 255));

    PrismAnimations.Solid linedUp = new PrismAnimations.Solid(Color.ORANGE);

    PrismAnimations.Solid linedUpoff = new PrismAnimations.Solid(Color.WHITE);

    PrismAnimations.Solid clear = new PrismAnimations.Solid(new Color( 255, 255, 255));

//    boolean lastAtSpeed = false;
//
//    boolean atSpeed= false;

    private boolean team;
    //
    private String Team;
    double far=57*28;
    double close=49*28;
    double targetVelocity=close;
    double curTargetVelocity = 0;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(140, 0,0, 13.2);
//    double F = 140/2;
//    double P = 13.2/2;


//
//    private String Color;


//    boolean r=false;
//    boolean l=false;

    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
    static final double     TARGET_RPM    = 600;


    @Override
    public void runOpMode() {


        left_drive = hardwareMap.get(DcMotor.class, "LD");
        right_drive = hardwareMap.get(DcMotor.class, "RD");
        left_back = hardwareMap.get(DcMotor.class, "LB");
        right_back = hardwareMap.get(DcMotor.class, "RB");

//        launch= hardwareMap.get(DcMotor.class, "launch");
//
//        intake= hardwareMap.get(DcMotor.class, "intake");

        Launch = hardwareMap.get(DcMotorEx.class, "Launch");
        Launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Launch.setDirection(DcMotorSimple.Direction.REVERSE);


        intake = hardwareMap.get(DcMotor.class, "intake");

        Revolver = hardwareMap.get(DcMotor.class, "Revolver");


        flick = hardwareMap.get(Servo.class, "flick");


//        Conveyor = hardwareMap.get(DcMotor.class, "Conveyor");

//        test = hardwareMap.get(DcMotor.class, "test");

        side1 = hardwareMap.get(CRServo.class, "side1");

        side2 = hardwareMap.get(CRServo.class, "side2");


//        color = hardwareMap.get(ColorSensor.class, "color");
        top = hardwareMap.get(ColorSensor.class, "top");

        top2 = hardwareMap.get(ColorSensor.class, "top");

//        Distance = hardwareMap.get(DistanceSensor.class, "Distance");

        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        limelight.pipelineSwitch(0);

//        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
//        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
//        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);

        green.setBrightness(50);
        green.setStartIndex(0);
        green.setStopIndex(12);

        purple.setBrightness(50);
        purple.setStartIndex(0);
        purple.setStopIndex(12);

        launchlight.setBrightness(100);
        launchlight.setStartIndex(0);
        launchlight.setStopIndex(0);

//        launchlight.setNumberOfSnakes(1);
//        launchlight.setSnakeLength(2);
//        launchlight.setSpacingBetween(2);
//        launchlight.setSpeed(0.2f);

        launchlight2.setBrightness(100);
        launchlight2.setStartIndex(5);
        launchlight2.setStopIndex(5);

        launchlightoff.setBrightness(10);
        launchlightoff.setStartIndex(0);
        launchlightoff.setStopIndex(0);

        launchlight2off.setBrightness(10);
        launchlight2off.setStartIndex(5);
        launchlight2off.setStopIndex(5);

        linedUp.setBrightness(100);
        linedUp.setStartIndex(8);
        linedUp.setStopIndex(9);

        linedUpoff.setBrightness(10);
        linedUpoff.setStartIndex(8);
        linedUpoff.setStopIndex(9);

        clear.setBrightness(5);
        clear.setStartIndex(0);
        clear.setStopIndex(11);



        telemetry.addData("Device ID: ", prism.getDeviceID());
        telemetry.addData("Firmware Version: ", prism.getFirmwareVersionString());
        telemetry.addData("Hardware Version: ", prism.getHardwareVersionString());
        telemetry.addData("Power Cycle Count: ", prism.getPowerCycleCount());
        telemetry.update();

//            if(limelight.)
//        telemetry.addData("",limelight.updateRobotOrientation(50));

        Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//        Launch.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);



//        color = hardwareMap.get(ColorSensor.class, "color");
//        Intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);
//        Conveyor.setDirection(DcMotor.Direction.REVERSE);


//        telemetry.setMsTransmissionInterval(11);

//        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
//        limelight.start();

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        cap2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        YawPitchRollAngles robotOrientation
        //this number equals how far you want it to go
//        int launchTarget = (int)(96*3.14 * COUNTS_PER_MM);
////        int rightTarget = (int)(610 * COUNTS_PER_MM);
//        double TPS = (60/ 60) * COUNTS_PER_WHEEL_REV;

//        double TargetVelocity = (TARGET_RPM*COUNTS_PER_MOTOR_REV)/60.0;

        boolean pressed = false;

        waitForStart();

        while (opModeIsActive()) {
            //forward/backward, strafe left/right, turn left/right
            float LF_Power=((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            float RF_Power=((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            float LB_Power=((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            float RB_Power=((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));

            // mecanum code
            if (gamepad1.dpad_down) {
                pressed = true;
            } else if (gamepad1.dpad_up) {
                pressed = false;
            }
            if (!pressed) {

                left_drive.setPower((LF_Power)*1);
                right_drive.setPower((RF_Power)*1);
                left_back.setPower((LB_Power)*1);
                right_back.setPower((RB_Power)*1);

            } else {
                //slow mode
                left_drive.setPower((LF_Power) * .4);
                right_drive.setPower((RF_Power) * .4);
                left_back.setPower((LB_Power) * .4);
                right_back.setPower((RB_Power) * .4);
            }
            int Green= (top.green()+top.green())/2;

            int Blue= (top.blue()+top.blue())/2;

            int Red= (top.red()+top.red())/2;

            if(time>.25) {
                if (Blue > Green && Blue > Red && Blue > 100 ) {
//            if(gamepad1.b) {
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, purple);
//                    prism.insertAndUpdateAnimation(LayerHeight.DISABLED, clear);

//                    prism.updateAnimationFromIndex(LayerHeight.LAYER_0);
//            }
                } else if (Green > Blue && Green > Red && Green > 100) {
//            if(gamepad1.a){
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, green);
//                    prism.insertAndUpdateAnimation(LayerHeight.DISABLED, clear);

//                    prism.updateAnimationFromIndex(LayerHeight.LAYER_1);
                } else {
//            if(gamepad1.a) {
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, clear);
//                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, clear);
//                    prism.insertAndUpdateAnimation(LayerHeight.DISABLED, purple);
//                    prism.insertAndUpdateAnimation(LayerHeight.DISABLED, green);


//                    prism.updateAnimationFromIndex(LayerHeight.LAYER_2);
//            }
                }
                resetRuntime();
//                time=0;
            }
            //manual control of revolver
            Revolver.setPower((gamepad2.left_stick_y)*.7);
            //flick control
            if(gamepad2.right_bumper) {

                flick.setPosition(0.77);

            }
            else if(Math.abs(gamepad2.left_stick_y)>.0001){
                flick.setPosition(0.3);
            }
            //flick stays out when button not pressed
            else{

                flick.setPosition(.30);
            }
            //intake control, keep slow
            intake.setPower((gamepad2.left_trigger - gamepad2.right_trigger)*.9);




//number in middle is rotations per minute
//            int close= 28*1/60;

            //fast button * 28 is how many ticks per rev

//            if(gamepad2.a){
//                StopWheels();
//                Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                Revolver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                Revolver.setTargetPosition(160);
//                Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Revolver.setPower(.8);
//                sleep(1000);
//                flick.setPosition(.78);
//                sleep(600);
//                Revolver.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
//            }



//            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

            Launch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


            if(gamepad2.x){
                curTargetVelocity=far;
                targetVelocity=far;

            }

            //slow speed used more often
            if(gamepad2.b){
                curTargetVelocity=close;
                targetVelocity=close;
            }

            //stop launch
            if(gamepad2.y){
                curTargetVelocity=0;
            }

//            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            Launch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            //set Velocity
            Launch.setVelocity(curTargetVelocity);

            double curVelocity = Launch.getVelocity();

            double error = curTargetVelocity - curVelocity;
            //backward launch button
            if(gamepad2.left_bumper){
                curTargetVelocity=-500;
            }


            if(time>.25) {


            if (Launch.getVelocity() > targetVelocity - (5*28)) {
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_2, launchlight);
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_3, launchlight2);

            } else {
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_2, launchlightoff);
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_3, launchlight2off);

            }



            }
//            else{
//                prism.insertAndUpdateAnimation(LayerHeight.LAYER_2, launchlightoff);
//                prism.insertAndUpdateAnimation(LayerHeight.LAYER_2, launchlight2off);
//
//            }






//            if(color.blue()>color.green()&& color.blue()>100){
//                Color="purple";
//            }
//            else if(color.green()>color.blue()&& color.green()>100){
//                Color="green";
//            }
//            else {
//                Color="no artifact";
//            }


            //new attempt
            if(Revolver.getCurrentPosition()>960 || Revolver.getCurrentPosition()<-960) {
                revolverpos=1;
                lastpos=0;
                Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

            if((lastpos - Revolver.getCurrentPosition())>=160&&(lastpos>Revolver.getCurrentPosition())){
                lastpos=Revolver.getCurrentPosition();
//            lastpos=revolverpos*160;
                revolverpos++;
            }
            //      0        161
            else if(((lastpos - Revolver.getCurrentPosition())<=-160)&&(lastpos<Revolver.getCurrentPosition())){
                lastpos=Revolver.getCurrentPosition();
//                lastpos=revolverpos*160;
                revolverpos--;
            }

//            if(revolverpos==1) color1=SetColor();
//            if(revolverpos==3) color2=SetColor();
//            if(revolverpos==5) color3=SetColor();


            if(revolverpos>=7){
                revolverpos=1;
            }

            if(revolverpos<=0){
                revolverpos=6;
            }




//
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<FiducialResult> fiducials = result.getFiducialResults();

                if (!fiducials.isEmpty()) {
//                     Grab the first detected tag
                    AprilTagID = fiducials.get(0).getFiducialId();

//                    telemetry.addData("Detected AprilTag", AprilTagID);
                }
            }

//            if(result.getTx()>5){
//                test.setPower(-.1);
//            }
//           else if(result.getTx()<-5){
//                test.setPower(.1);
//            }
//           else if(result.getTx()>-5 && result.getTx()<5){
//               test.setPower(0);
//            }

            if(gamepad2.left_stick_button){
                team=true;
            }
            if(gamepad2.right_stick_button){
                team=false;
            }
//

            if(team) {
                limelight.pipelineSwitch(1);
                Team="blue";
            }
            if(!team) {
                limelight.pipelineSwitch(2);
                Team="red";
            }

//            if(time>.2) {
                if (Math.abs(result.getTx()) < 5 && result.getTx() != 0 && team) {
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_4, linedUp);
                    CanLaunch = "Yes:blue";
//            Launch.setVelocity(far);
                }

                else if (Math.abs(result.getTx()) < 5 && result.getTx() != 0 && !team) {
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_4, linedUp);
                    CanLaunch = "Yes:red";
//            Launch.setVelocity(far);
                } else {
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_4, linedUpoff);

                    CanLaunch = "No";
                }

//            }


            if(result.getTa()<1){
                closefar="far";
            }
            else if(result.getTa()>.5){
                closefar="close";
            }
            else{
                closefar="can't shoot";
            }

//
//
//            telemetry.update();

//            telemetry.addData("Detected AprilTag", AprilTagID);

//            telemetry.addData("RedValue",  color.red());
//            telemetry.addData("BlueValue", color.blue());
//            telemetry.addData("GreenValue",  color.green());
//
            telemetry.addData("RedValue",  top.red());
            telemetry.addData("BlueValue", top.blue());
            telemetry.addData("GreenValue",  top.green());
//
            telemetry.addData("RedValue",  top2.red());
            telemetry.addData("BlueValue", top2.blue());
            telemetry.addData("GreenValue",  top2.green());

//            telemetry.addData("argb",  color.argb());
//            telemetry.addData("color:",Color);
//
            telemetry.addData("team:",Team);

//            telemetry.addData("time:",time);

//            telemetry.addData("Target Velocity",curTargetVelocity);
//            telemetry.addData("Current Velocity", "%.2f",curVelocity);
//            telemetry.addData("Error", "%.2f", error);
//

////            telemetry.addData("distance",Distance.getDistance(DistanceUnit.MM));
            telemetry.addData("x",result.getTx());
              telemetry.addData("goal distance",result.getTa());
//            telemetry.addData("TPS", TPS);
//            telemetry.addData("launchtarget", launchTarget);

            telemetry.addData("Target",targetVelocity);
//            telemetry.addData("launch",Launch.getCurrentPosition());
//            telemetry.addData("launch",Launch.getPower());
            telemetry.addData("can Launch?:",CanLaunch);
//            telemetry.addData("Distance:",closefar);
//            telemetry.addData("P",  P);
//            telemetry.addData("F", F);
//            telemetry.addData("launch",Launch.getVelocity()/28);
//            telemetry.addData("color1",color1);
//            telemetry.addData("color2",color2);
//            telemetry.addData("color3",color3);
//            telemetry.addData("lastpos",lastpos);
//            telemetry.addData("currentpos",revolverpos);
//            telemetry.addData("Revolver:", Revolver.getCurrentPosition());
            telemetry.update();

//            telemetry.addData("TouchSensor", touch.isPressed());
//            telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
//            telemetry.addData("Yaw", robotOrientation.getYaw(AngleUnit.DEGREES));
//            telemetry.addData("Roll", robotOrientation.getRoll(AngleUnit.DEGREES));
//            telemetry.addData("Pitch", robotOrientation.getPitch(AngleUnit.DEGREES));

//            telemetry.addData("pressed", touch.isPressed());
//

//            telemetry.addData("arm", arm);


        }

    }

//    private String SetColor() {
//        String Colors;
//        if (color.blue() > color.green() &&color.blue()>color.red() && color.blue() > 100) {
//            Colors = "purple";
//        } else if (color.green() > color.blue() && color.green()>color.red() &&color.green() > 100) {
//            Colors = "green";
//        } else if (color.red() > color.blue() && color.red()>color.green()&& color.red() > 100){
//            Colors = "none";
//        }
//        else {
//            Colors="none";
//        }
//        return Colors;
//    }

//    public void setColor(){
//
//    }

public  void StopWheels(){
    left_drive.setPower(0);
    right_drive.setPower(0);
    left_back.setPower(0);
    right_back.setPower(0);

}

}
