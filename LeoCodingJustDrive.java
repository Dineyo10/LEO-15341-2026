package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class LeoCodingJustDrive extends LinearOpMode {
    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor left_back;
    private DcMotor right_back;
//    private Servo grab;
//    private Servo leftgrab;
//    private Servo rightgrab;
//    private DcMotor cap;
//    private DcMotor cap2;
//    private DcMotor activeArm1;
//    private DcMotor activeArm2;
//    private Servo arm1;
//    private Servo arm2;
//    private Servo wrist1;
//    private Servo wrist2;
//    private Servo swivel;
//    private Servo drone;
//    boolean arm = true;
//    float height;


    @Override
    public void runOpMode() {


        left_drive = hardwareMap.get(DcMotor.class, "LD");
        right_drive = hardwareMap.get(DcMotor.class, "RD");
        left_back = hardwareMap.get(DcMotor.class, "LB");
        right_back = hardwareMap.get(DcMotor.class, "RB");
//
//        grab = hardwareMap.get(Servo.class, "grab");
//
//        leftgrab = hardwareMap.get(Servo.class, "leftgrab");
//        rightgrab = hardwareMap.get(Servo.class, "rightgrab");
//
//
//        cap = hardwareMap.get(DcMotor.class, "cap");
//        cap2 = hardwareMap.get(DcMotor.class, "cap2");
//
//        activeArm1= hardwareMap.get(DcMotor.class, "activeArm1");
//        activeArm2 = hardwareMap.get(DcMotor.class, "activeArm2");
//
//        arm1 = hardwareMap.get(Servo.class, "arm1");
//        arm2 = hardwareMap.get(Servo.class, "arm2");
//
//        wrist1 = hardwareMap.get(Servo.class, "front1");
//        wrist2 = hardwareMap.get(Servo.class, "front2");
//
//
//        swivel = hardwareMap.get(Servo.class, "swivel");

//        drone = hardwareMap.get(Servo.class, "drone");


        right_back.setDirection(DcMotor.Direction.REVERSE);
        //right_drive is also reversed at line 325 and doesn't need to be reversed
//        right_drive.setDirection(DcMotor.Direction.REVERSE);


//        cap.setDirection(DcMotor.Direction.REVERSE);
////        cap2.setDirection(DcMotor.Direction.REVERSE);
//
//        activeArm1.setDirection(DcMotor.Direction.REVERSE);
////        activeArm2.setDirection(DcMotor.Direction.REVERSE);
//
//        cap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        cap2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        activeArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        activeArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
////        cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        cap2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        cap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        cap2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        activeArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        activeArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        left_back.setTargetPosition(0);
//        right_back.setTargetPosition(0);

//        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        cap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        cap2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        cap2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        wrist1.setPosition(0.36);
//        wrist2.setPosition(.59);
//        swivel.setPosition(.64);
//        arm1.setPosition(.83);
//        arm2.setPosition(0.17);
        boolean pressed = false;
        waitForStart();

        // Put run blocks here.
        while (opModeIsActive()) {

//drone
//fire
//            if (gamepad2.right_stick_button) {
//                drone.setPosition(0.7);
//            }
//
////ready
//            if (gamepad2.left_stick_button) {
//                drone.setPosition(1);
//            }

            //grab
            //grabbing
//            if (gamepad2.x) {
//                grab.setPosition(0);
//                leftgrab.setPosition(0.72);
//                rightgrab.setPosition(0.34);
//            }
//
//            //triangle/y is open
//            //leftgrab up is open
//            //rightgrab down is open
//            if (gamepad2.y ) {
//                leftgrab.setPosition(.33);
//                rightgrab.setPosition(.65);
//                sleep(150);
//                grab.setPosition(1);
//
//            }
//
//            if (activeArm2.getCurrentPosition() > -1000 && gamepad1.a ) {
//                arm1.setPosition(0.10);
//                arm2.setPosition(0.90);
//            }
//            if (activeArm2.getCurrentPosition() < -1000){
//                arm1.setPosition(0.64);
//                arm2.setPosition(0.36);
//            }
//
//
//            if(gamepad1.b){
//                arm1.setPosition(0.25);
//                arm2.setPosition(0.75);
//            }
//            //open leftgrab
//
////
//
//
////            if (gamepad2.dpad_down) {
////                arm1.setPosition(0.5);
////                arm2.setPosition(0.5);
////            }
//            //arm ready for pixel/sample
//// arm down
//            if(gamepad2.dpad_up){
//                swivel.setPosition(.68);
//                wrist1.setPosition(.65);
//                wrist2.setPosition(0.31);
//            }
//            if (gamepad2.a ) {
//                //AXON code
//                wrist1.setPosition(.89);
//                wrist2.setPosition(0.07);
//                swivel.setPosition(.68);
//                arm1.setPosition(0);
//                arm2.setPosition(1);
//
//                //goBilda servo code
////                wrist1.setPosition(.89);
////                wrist2.setPosition(0.07);
////                swivel.setPosition(.68);
////                arm1.setPosition(0.39);
////                arm2.setPosition(0.61);
//            }
//
//            if (gamepad2.b ) {
//                //AXON code
//                wrist1.setPosition(0.36);
//                wrist2.setPosition(.59);
//                swivel.setPosition(.64);
//                arm1.setPosition(1);
//                arm2.setPosition(0);
//                //goBilda code
////                wrist1.setPosition(0.36);
////                wrist2.setPosition(.59);
////                swivel.setPosition(.64);
////                arm1.setPosition(.83);
////                arm2.setPosition(0.17);
//            }
//
//            if(gamepad2.dpad_left){
//                swivel.setPosition(.99);
//            }
//            if(gamepad2.dpad_right){
//                swivel.setPosition(.34);
//            }
//            if(gamepad2.dpad_down){
//                swivel.setPosition(.68);
//            }
//            if(gamepad2.left_stick_button){
//                grab.setPosition(0);
//                sleep(100);
//                swivel.setPosition(0);
//                wrist1.setPosition(.57);
//                wrist2.setPosition(0.39);
//            }

            //arm straight up
//            if (gamepad2.dpad_up) {
//
//                arm1.setPosition(.79);
//                arm2.setPosition(0.21);
//            }


            // mecanum code
            if (gamepad1.dpad_down) {
                pressed = true;
            } else if (gamepad1.dpad_up) {
                pressed = false;
            }
            if (!pressed) {

                left_drive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * 1);
                right_drive.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * 1);
                left_back.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * 1);
                right_back.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 1);

            } else {
                left_drive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * .35);
                right_drive.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * .35);
                left_back.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * .35);
                right_back.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * .35);
            }



//            if(gamepad2.back){
//                cap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                cap2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }

//            if (gamepad2.right_bumper) {
//                cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                cap2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                cap.setPower(1);
//                cap2.setPower(1);
//
////                cap.setTargetPosition(600);
////                cap2.setTargetPosition(600);
//
//                cap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                cap2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                cap.setTargetPosition(600);
//                cap2.setTargetPosition(600);
//
//            }

//          if  (gamepad2.start){
//              cap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//              cap2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//          }

//            if(gamepad2.left_bumper && !arm){
//                height=height+1;
//                arm=true;
//            }
//            if(gamepad2.right_bumper && arm){
//                height=height-1;
//                arm=false;
//            }
//            if (gamepad1.left_bumper && arm==false||gamepad2.left_bumper && arm==false) {
//
//            arm=true;
//
//            }
//
//            if (gamepad1.left_bumper && arm==true||gamepad2.left_bumper && arm== true) {
//
//                arm=false;
//
//            }
//
//
//            if (arm== true) {
//



//            activeArm1.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
//            activeArm2.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

//            if (gamepad1.a ) {
//
//                wrist1.setPosition(0);//90 degrees back | 0.0 //60 degress .34
//                wrist2.setPosition(1);//90 degrees back | 0.8
//
//
//            }
//
//            if (gamepad1.b ) {
//
//                wrist1.setPosition(1);//90 degrees back | 0.0 //60 degress .34
//                wrist2.setPosition(0);//90 degrees back | 0.8
//
//
//            }

//            if (activeArm2.getCurrentPosition() > -1600) {
//                activeArm1.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
//                activeArm2.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
//            }
//            else {
//                activeArm1.setPower(Math.abs(gamepad1.left_trigger - gamepad1.right_trigger));
//                activeArm2.setPower(Math.abs(gamepad1.left_trigger - gamepad1.right_trigger));
//            }
//
//            if (cap2.getCurrentPosition() > -3050) {
//
//
//                cap.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
//                cap2.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
//            }
//            else {
//                cap.setPower(0);
//                cap2.setPower(0);
//            }
//            }
//
//
//
//            if (arm==false) {
//
//
//                if (cap.getCurrentPosition() < 2800) {

            //hayden controls


//                } else {
//                    cap.setPower(0);//-gamepad2.left_trigger - gamepad2.right_trigger);
//                    cap2.setPower(0);//-gamepad2.left_trigger - gamepad2.right_trigger);
//                }
//            }
            telemetry.update();

//            telemetry.addData("RCap", cap.getCurrentPosition());
//            telemetry.addData("Lcap2", cap2.getCurrentPosition());

//            telemetry.addData("leftarm", activeArm1.getCurrentPosition());
//            telemetry.addData("rightarm", activeArm2.getCurrentPosition());

//            telemetry.addData("height", height);
            telemetry.addData("rMotor", right_back.getCurrentPosition());
            telemetry.addData("LMotor", left_back.getCurrentPosition());

            telemetry.addData("FrMotor", right_drive.getCurrentPosition());
            telemetry.addData("FLMotor", left_drive.getCurrentPosition());
//            telemetry.addData("pressed", pressed();
//
//            telemetry.addData("arm", arm);
            telemetry.update();

        }

    }
}