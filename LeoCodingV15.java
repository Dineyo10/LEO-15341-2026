package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@TeleOp
public class LeoCodingV15 extends LinearOpMode {
    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor left_back;
    private DcMotor right_back;

//    private DcMotor launch;
//    private DcMotor intake;
//    private CRServo Intake1;
//    private CRServo Intake2;
    private DcMotor Launch1;
    private DcMotor Launch2;

    private DcMotor Conveyor;



    private CRServo Stopper;

    //    private Servo leftgrab;
//    private Servo rightgrab;

//    private TouchSensor touch;
//    private ColorSensor color;
//    private Servo backGrab;
//    private Servo wrist;
//    private Limelight3A limelight;


    // Target heading for straight movement (0 degrees is "straight")
//    private DistanceSensor distance;
//    private Servo drone;
//    boolean arm = true;
//    float height;


    @Override
    public void runOpMode() {


        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

//        launch= hardwareMap.get(DcMotor.class, "launch");
//
//        intake= hardwareMap.get(DcMotor.class, "intake");


//        Intake1 = hardwareMap.get(CRServo.class, "Intake1");
//        Intake2 = hardwareMap.get(CRServo.class, "Intake2");

        Launch1 = hardwareMap.get(DcMotor.class, "Launch1");
        Launch2 = hardwareMap.get(DcMotor.class, "Launch2");

        Conveyor = hardwareMap.get(DcMotor.class, "Conveyor");



        Stopper = hardwareMap.get(CRServo.class, "Stopper");






//        color = hardwareMap.get(ColorSensor.class, "color");
        Launch1.setDirection(DcMotorSimple.Direction.REVERSE);
//        Intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);
        Conveyor.setDirection(DcMotor.Direction.REVERSE);


        //right_drive is also reversed at line 325 and doesn't need to be reversed
//        right_drive.setDirection(DcMotor.Direction.REVERSE);



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

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

//        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
//        limelight.start();

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Launch1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Launch2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        cap2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        YawPitchRollAngles robotOrientation;

        boolean pressed = false;
        waitForStart();

        while (opModeIsActive()) {

            float LF_Power=((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            float RF_Power=((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            float LB_Power=((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            float RB_Power=((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));


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
                left_drive.setPower((LF_Power) * .4);
                right_drive.setPower((RF_Power) * .4);
                left_back.setPower((LB_Power) * .4);
                right_back.setPower((RB_Power) * .4);
            }

//         Intake1.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
//         Intake2.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

         Conveyor.setPower(gamepad2.left_trigger - gamepad2.right_trigger);


         if(gamepad2.x){
             Stopper.setPower(1);
         }
         else if(gamepad2.y){
             Stopper.setPower(-1);
         }
         else {
             Stopper.setPower(0);
         }


//            if(gamepad2.y){
//                Stopper.setPower(-1);
//            }

            if(gamepad2.dpad_up) {
                Launch1.setPower(.41);
                Launch2.setPower(.41);
            }
            if(gamepad2.dpad_down) {
                Launch1.setPower(.35);
                Launch2.setPower(.35);
            }

            if(gamepad2.b) {
                Launch1.setPower(0);
                Launch2.setPower(0);
            }


//            if(gamepad2.x){
//                Stopper.setPower(1);
////                Launch1.setPower(1);
////                Launch2.setPower(1);
//                sleep(800);
////                Launch1.setPower(0);
////                Launch2.setPower(0);
//                Stopper.setPower(0);
//
//            }
//        if(gamepad2.x){
//            Conveyor.setPower(1);
//        }
//
//        if(gamepad2.y){
//            Conveyor.setPower(-1);
//        }



            //disable for matches!!!
//            telemetry.addData("speed", gamepad1.left_stick_y);


//            telemetry.addData("BlueValue", color.blue());
//            telemetry.addData("RedValue",  color.red());
//            telemetry.addData("GreenValue",  color.green());


//            telemetry.addData("TouchSensor", touch.isPressed());
//            telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
//            telemetry.addData("Yaw", robotOrientation.getYaw(AngleUnit.DEGREES));
//            telemetry.addData("Roll", robotOrientation.getRoll(AngleUnit.DEGREES));
//            telemetry.addData("Pitch", robotOrientation.getPitch(AngleUnit.DEGREES));

//            telemetry.addData("pressed", touch.isPressed());
//

//            telemetry.addData("arm", arm);
            telemetry.update();

        }


    }




}
