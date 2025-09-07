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

//@Disabled
@TeleOp
public class LeoCodingV15 extends LinearOpMode {
    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor left_back;
    private DcMotor right_back;

    private DcMotor launch;
    private DcMotor intake;
    private CRServo Conveyor1;
    private CRServo Conveyor2;

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

        launch= hardwareMap.get(DcMotor.class, "launch");

        intake= hardwareMap.get(DcMotor.class, "intake");


        Conveyor1 = hardwareMap.get(CRServo.class, "Conveyor1");
        Conveyor2 = hardwareMap.get(CRServo.class, "Conveyor2");









//        color = hardwareMap.get(ColorSensor.class, "color");

        right_back.setDirection(DcMotor.Direction.REVERSE);
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
//        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        cap2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        YawPitchRollAngles robotOrientation;

        boolean pressed = false;
        waitForStart();

        while (opModeIsActive()) {

            float LF_Power=((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * 1 );
            float RF_Power=((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * 1);
            float LB_Power=((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * 1);
            float RB_Power=((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 1);


            // mecanum code
            if (gamepad1.dpad_down) {
                pressed = true;
            } else if (gamepad1.dpad_up) {
                pressed = false;
            }
            if (!pressed) {

                left_drive.setPower((LF_Power)*.7);
                right_drive.setPower((RF_Power)*.7);
                left_back.setPower((LB_Power)*.7);
                right_back.setPower((RB_Power)*.7);

            } else {
                left_drive.setPower((LF_Power) * .4);
                right_drive.setPower((RF_Power) * .4);
                left_back.setPower((LB_Power) * .4);
                right_back.setPower((RB_Power) * .4);
            }

            if(gamepad2.x){
                intake.setPower(1);
                Conveyor1.setPower(1);
                Conveyor2.setPower(1);
            }
            if(gamepad2.y){
                intake.setPower(-1);
                Conveyor1.setPower(-1);
                Conveyor2.setPower(-1);
            }






//if(gamepad1.b){
//    boolean w=true;
//}

////            LLResult result = limelight.getLatestResult();
//            if (result != null) {
//                if (result.isValid()) {
//                    Pose3D botpose = result.getBotpose();
//                    telemetry.addData("tx", result.getTx());
//                    telemetry.addData("ty", result.getTy());
//                    telemetry.addData("Botpose", botpose.toString());
////                    telemetry.update();
//                }
//            }

            //disable for matches!!!

//            telemetry.addData("rMotor", right_back.getCurrentPosition());
//            telemetry.addData("LMotor", left_back.getCurrentPosition());
//            telemetry.addData("FrMotor", right_drive.getCurrentPosition());
//            telemetry.addData("FLMotor", left_drive.getCurrentPosition());
//            telemetry.addData("lefttrigger", gamepad2.left_trigger);
//            telemetry.addData("righttrigger", gamepad2.right_trigger);

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

