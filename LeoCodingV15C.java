package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Fiducial;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

import javax.sql.RowSetEvent;

@Disabled
@TeleOp
public class LeoCodingV15C extends LinearOpMode {
    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor left_back;
    private DcMotor right_back;

    private DcMotor intake;
    private CRServo side1;
    private CRServo side2;
    private DcMotor Revolver;
    private DcMotor Launch;

    private Servo flick;
//    private DcMotor test;

//    private ColorSensor color;
//    private CRServo Stopper;

    private int AprilTagID;
//    private Limelight3A limelight;

    private boolean team;

    private String Team;

    private String Color;

    int revolverpos;

    boolean r=false;
    boolean l=false;




    @Override
    public void runOpMode() {


        left_drive = hardwareMap.get(DcMotor.class, "LD");
        right_drive = hardwareMap.get(DcMotor.class, "RD");
        left_back = hardwareMap.get(DcMotor.class, "LB");
        right_back = hardwareMap.get(DcMotor.class, "RB");

//        launch= hardwareMap.get(DcMotor.class, "launch");
//
//        intake= hardwareMap.get(DcMotor.class, "intake");


        intake = hardwareMap.get(DcMotor.class, "intake");

        Revolver = hardwareMap.get(DcMotor.class, "Revolver");

        Launch = hardwareMap.get(DcMotor.class, "Launch");

        flick = hardwareMap.get(Servo.class, "flick");

//        Launch2 = hardwareMap.get(DcMotor.class, "Launch2");

//        Conveyor = hardwareMap.get(DcMotor.class, "Conveyor");

//        test = hardwareMap.get(DcMotor.class, "test");

        side1 = hardwareMap.get(CRServo.class, "side1");

        side2 = hardwareMap.get(CRServo.class, "side2");


//        color = hardwareMap.get(ColorSensor.class, "color");


//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.start();

//        limelight.pipelineSwitch(1);


//            if(limelight.)
//        telemetry.addData("",limelight.updateRobotOrientation(50));

//        Revolver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        color = hardwareMap.get(ColorSensor.class, "color");
//        Launch1.setDirection(DcMotorSimple.Direction.REVERSE);
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
//        Launch2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        cap2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        YawPitchRollAngles robotOrientation;

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

            //manual control of revolver
            Revolver.setPower((gamepad2.left_stick_y)*.6);
//
             //reset revolver encoder
            if(gamepad2.start){
                Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
//
////          encoder control revolver
//            if(gamepad2.b){
//                Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                r=true;
//            }
//
//            if(r){
//                Revolver.setTargetPosition(320);
//                Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Revolver.setPower(.8);
//            }
//
//            if(r && Revolver.getCurrentPosition() >= revolverpos + 315 &&
//                    Revolver.getCurrentPosition() <= revolverpos + 325 ){
//                r=false;
//            }
//
//            if(gamepad2.a){
//                Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                l=true;
//            }
//
//            if(l){
//                Revolver.setTargetPosition(160);
//                Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Revolver.setPower(.8);
//            }
//
//            if(l && Revolver.getCurrentPosition() >= revolverpos + 155 &&
//                    Revolver.getCurrentPosition() <= revolverpos + 165 ){
//                l=false;
//            }

            //flick control
        if(gamepad2.right_bumper){
            flick.setPosition(0.7);
        }
        else if(gamepad2.a ||gamepad2.b){
            flick.setPosition(0.45);
        }
        //flick stays out when button not pressed
            else{
                flick.setPosition(.45);
            }
            //intake control, keep slow
            intake.setPower((gamepad2.left_trigger - gamepad2.right_trigger)*.7);


//            Launch.setPower(gamepad2.left_stick_y);




            //let launch speed up then turn on side motors
            if(gamepad2.dpad_up){
               Launch.setPower(.9);
           }
            //turns on motors to bring artifact to launch
            if(gamepad2.dpad_right){
                side1.setPower(-.9);
                side2.setPower(.9);
            }
            //slow speed used more often
            if(gamepad2.dpad_down){
                Launch.setPower(.6);
            }
            //stop launch
            if(gamepad2.y){
                side1.setPower(0);
                side2.setPower(0);
                Launch.setPower(0);
            }

            //backward launch button
            if(gamepad2.left_bumper){
                Launch.setPower(-.5);
                side1.setPower(.9);
                side2.setPower(-.9);
            }

//            if(color.blue()>color.green()&& color.blue()>100){
//                Color="purple";
//            }
//            else if(color.green()>color.blue()&& color.green()>100){
//                Color="green";
//            }
//            else if(color.red()>color.blue()&& color.red()>color.green()&& color.red()>100){
//                Color="no artifact";
//            }
//
//            LLResult result = limelight.getLatestResult();
//
//            if (result != null && result.isValid()) {
//                List<FiducialResult> fiducials = result.getFiducialResults();
//
//                if (!fiducials.isEmpty()) {
////                     Grab the first detected tag
//                    AprilTagID = fiducials.get(0).getFiducialId();
//
//                    telemetry.addData("Detected AprilTag", AprilTagID);
//                }
//            }

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

            if(team) {
//                limelight.pipelineSwitch(1);
                Team="blue";
            }
            if(!team) {
//                limelight.pipelineSwitch(2);
                Team="red";
            }


            telemetry.update();

            telemetry.addData("Detected AprilTag", AprilTagID);

//            telemetry.addData("BlueValue", color.blue());
//            telemetry.addData("RedValue",  color.red());
//            telemetry.addData("GreenValue",  color.green());
//            telemetry.addData("argb",  color.argb());
            telemetry.addData("color:",Color);

            telemetry.addData("team:",Team);

            telemetry.addData("Revolver:", Revolver.getCurrentPosition());

//            telemetry.addData("x",result.getTx());
//              telemetry.addData("x",result.get;

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




}
