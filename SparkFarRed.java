package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.hardware.limelightvision.Fiducial;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Fiducial;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

import java.util.Locale;

@Autonomous(name="SparkFarRed", group="red")
//@Disabled

public class SparkFarRed extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private DcMotor intake;
    private DcMotor Revolver;
    private DcMotorEx Launch;

    private Servo flick;

    private CRServo side1;
    private CRServo side2;
//    private ColorSensor color;

    private ColorSensor top;



    private int AprilTagID;
    private Limelight3A limelight;

    private boolean team;

    private String Team;

    private String Color;
    //    private ColorSensor color;
    int revolverpos=1;
    private String color1;
    private String color2;
    private String color3;
    int lastpos=0;

    double far=57*28;

    double close=46.5*28;

    int change =0;

//    int revolverpos;

    //    private AprilTagProcessor aprilTag;
//    private VisionPortal visionPortal;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_2_Left,
        DRIVE_TO_TARGET_3_Left,
        DRIVE_TO_TARGET_2_Center,
        DRIVE_TO_TARGET_3_Center,
        DRIVE_TO_TARGET_2_Right,
        DRIVE_TO_TARGET_3_Right,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5,
        DRIVE_TO_TARGET_6,
        DRIVE_TO_TARGET_7,
        DRIVE_TO_TARGET_8,
        DRIVE_TO_TARGET_9,
        DRIVE_TO_TARGET_10,
        DRIVE_TO_TARGET_11,
        DRIVE_TO_TARGET_12,
        DRIVE_TO_TARGET_13,
        DRIVE_TO_TARGET_14,
        DRIVE_TO_TARGET_15,
        DRIVE_TO_TARGET_16,
        DRIVE_TO_TARGET_17,
        DRIVE_TO_TARGET_18,
        DRIVE_TO_TARGET_19,
        DRIVE_TO_TARGET_20
    }


    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,50,0,AngleUnit.DEGREES,-22);
    static final Pose2D TARGET_2_Left = new Pose2D(DistanceUnit.MM, 610, -300, AngleUnit.DEGREES, -90);
    static final Pose2D TARGET_3_Left = new Pose2D(DistanceUnit.MM, 640, -1100, AngleUnit.DEGREES, -90);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, 50, 0, AngleUnit.DEGREES, -22);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM, 700, 0, AngleUnit.DEGREES, 0);





    static final Pose2D TARGET_6 = new Pose2D(DistanceUnit.MM, -270, 520, AngleUnit.DEGREES, 45);
    static final Pose2D TARGET_7 = new Pose2D(DistanceUnit.MM, -1300, 1200, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_8 = new Pose2D(DistanceUnit.MM, -400, 1200, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_9 = new Pose2D(DistanceUnit.MM, -1300, 1200, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_10 = new Pose2D(DistanceUnit.MM, -1300, 1350, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_11 = new Pose2D(DistanceUnit.MM, -400, 1350, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_12 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_13 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_14 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_15 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_16 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_17 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_18 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_19 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_20 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);


    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.start();
//
//        limelight.pipelineSwitch(0);
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.


        intake = hardwareMap.get(DcMotor.class, "intake");

        Revolver = hardwareMap.get(DcMotor.class, "Revolver");

        Launch = hardwareMap.get(DcMotorEx.class, "Launch");

        flick = hardwareMap.get(Servo.class, "flick");


//        color = hardwareMap.get(ColorSensor.class, "color");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LD");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RD");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "LB");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "RB");


//        Launch = hardwareMap.get(DcMotor.class, "Launch");



        side1 = hardwareMap.get(CRServo.class, "side1");

        side2 = hardwareMap.get(CRServo.class, "side2");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        limelight.pipelineSwitch(0);


//        color = hardwareMap.get(ColorSensor.class, "color");
//        Launch1.setDirection(DcMotorSimple.Direction.REVERSE);
//        Intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        Launch.setDirection(DcMotor.Direction.REVERSE);

        Revolver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Launch2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(177.5, 10); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

//        nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
//        nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();




        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();


        while (opModeIsActive()) {
            odo.update();
            //first attempt at doing this
//            LLResult result = limelight.getLatestResult();
//
//            if (result != null && result.isValid()) {
//                List<FiducialResult> fiducials = result.getFiducialResults();
//
//                for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                    int AprilTagID = fiducial.getFiducialId();
//
//                    telemetry.addData("Detected Apriltag", AprilTagID);
////                }
////
//            if(Revolver.getCurrentPosition()>960 || Revolver.getCurrentPosition()<-960) {
//                revolverpos=1;
////                Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }
//
//
//            if(Math.abs(lastpos - Revolver.getCurrentPosition())>160){
//                lastpos=Revolver.getCurrentPosition();
//                revolverpos++;
//            }
//
//            if(revolverpos==1) color1=SetColor();
//            if(revolverpos==3) color2=SetColor();
//            if(revolverpos==5) color3=SetColor();
//
//            if(revolverpos==7){
//                revolverpos=1;
//            }
//second attempt at doing this





////            telemetry.addData("TagId", );
//            } else {
//                telemetry.addData("Limelight", "No Targets");
//            }
//            telemetry.update();

//            if (result != null && result.isValid()) {
//                List<FiducialResult> fiducials = result.getFiducialResults();
//
//                for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                    int AprilTagID = fiducial.getFiducialId();
//                    if (AprilTagID == 21)stateMachine = StateMachine.DRIVE_TO_TARGET_1;
//                    if (AprilTagID == 22)stateMachine = StateMachine.DRIVE_TO_TARGET_2;
//                    if (AprilTagID == 23)stateMachine = StateMachine.DRIVE_TO_TARGET_3;
//
//                    telemetry.addData("Detected Apriltag", AprilTagID);
//
//                }
//
//            } else {
//                telemetry.addData("Limelight", "No Targets");
//            }
//
//            telemetry.update();
            switch (stateMachine){
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    LLResult result = limelight.getLatestResult();

                    if (result != null && result.isValid()) {
                        List<FiducialResult> fiducials = result.getFiducialResults();

                        if (!fiducials.isEmpty()) {
                            // Grab the first detected tag
                            AprilTagID = fiducials.get(0).getFiducialId();

                            telemetry.addData("Detected AprilTag", AprilTagID);
                        } else {
                            telemetry.addLine("No AprilTags detected");
                        }
                    } else {
                        telemetry.addLine("Limelight: No valid results");
                    }

                    if(AprilTagID== 21){
                        telemetry.addLine("21,left");
                    }
                    if(AprilTagID== 22){
                        telemetry.addLine("22,middle");
                    }
                    if(AprilTagID== 23){
                        telemetry.addLine("23,right");
                    }
                    telemetry.update();
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;
                case DRIVE_TO_TARGET_1:
//                    launch1();
                    Launch.setVelocity(far);

                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), TARGET_1, .4, 1)) {
//                    Stopper.setPower(.3);
//                        Revolver.setTargetPosition(160);
//                        sleep(500);

                        if (AprilTagID==21) {
                            SpinUp();
                            launch21();
                        }
                        else if (AprilTagID==22) {
                            SpinUpshort();
                            launch22();
                        }
                        else if (AprilTagID==23) {
                            SpinUpshort();
                            launch23();
                        }
                        else{
                            SpinUp();
                            launch21();
                        }
//                        sleep(5000);
//                        flick();
//                        sleep(5000);
                        telemetry.addLine("at position #1!");
//                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2_Left;

//                        if (AprilTagID==21) {
//                            stateMachine = StateMachine.DRIVE_TO_TARGET_2_Left;
//                            telemetry.addLine("Pattern 1");
//
//                        }
//                        else if (AprilTagID==22) {
////                            stateMachine = StateMachine.DRIVE_TO_TARGET_2_Center;
//                            telemetry.addLine("Pattern 2");
//
//                        }
//                        else{
////                            stateMachine = StateMachine.DRIVE_TO_TARGET_2_Right;
//                            telemetry.addLine("Pattern 3");
//
//                        }
                    }
//                    break;
//                case DRIVE_TO_TARGET_2:
//                    if (nav.driveTo(odo.getPosition(), TARGET_2, 1, 0.1)){
//
////                        Intake();
//                        intake();
//                        telemetry.addLine("at position #2!");
//                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
//
//                    }
                    break;
                case DRIVE_TO_TARGET_2_Left:
                    if (nav.driveTo(odo.getPosition(), TARGET_2_Left, .7, .2)){

                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3_Left;
                    }
                    break;
                case DRIVE_TO_TARGET_3_Left:
                    intake();
                    if (nav.driveTo(odo.getPosition(), TARGET_3_Left, .4, 2)){
//                        intake();
//                        change=160;
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;

                        Launch.setVelocity(far);

                    }
                    break;
//                case DRIVE_TO_TARGET_2_Center:
//                    if (nav.driveTo(odo.getPosition(), TARGET_2_Center, .7, 2)){
////                        Stopper.setPower(.3);
//                        intake();
//
//                        telemetry.addLine("at position #2!");
//                        stateMachine = StateMachine.DRIVE_TO_TARGET_3_Center;
//                    }
//                    break;
//                case DRIVE_TO_TARGET_3_Center:
//                    if (nav.driveTo(odo.getPosition(), TARGET_3_Center, .7, 2)){
////                        intake();
//                        telemetry.addLine("at position #2!");
//                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
//                        Stopintake();
//                        Launch.setPower(.65);
//                    }
//                    break;
//                case DRIVE_TO_TARGET_2_Right:
//                    if (nav.driveTo(odo.getPosition(), TARGET_2_Right, .7, 2)){
////                        Stopper.setPower(.3);
//                        intake();
//                        telemetry.addLine("at position #2!");
//                        stateMachine = StateMachine.DRIVE_TO_TARGET_3_Right;
//                    }
//                    break;
//                case DRIVE_TO_TARGET_3_Right:
//                    if (nav.driveTo(odo.getPosition(), TARGET_3_Right, .7, 2)){
////                        intake();
//                        telemetry.addLine("at position #2!");
//                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
//                        Stopintake();
//                        Launch.setPower(.65);
//
//                    }
//                    break;
//                case DRIVE_TO_TARGET_3:

//                    if(nav.driveTo(odo.getPosition(), TARGET_3, 1, 0)){
//                        telemetry.addLine("at position #3");
////                        sleep(5000);
////                        Revolver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
//                    }
//                    break;
                case DRIVE_TO_TARGET_4:
                    if(nav.driveTo(odo.getPosition(),TARGET_4,.8,.5)){
                        telemetry.addLine("at position #4");
                        Stopintake();
                        SpinUpshort();
                        if (AprilTagID==21) {
                            launch21();
                        }
                        else if (AprilTagID==22) {
                            launch22();
                        }
                        else if (AprilTagID==23) {
                            launch23();
                        }
                        else{
                            launch21();
                        }


                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if(nav.driveTo(odo.getPosition(),TARGET_5,.7,0.1)){
                        telemetry.addLine("at position #5!");
//                        Stopintake();
//                        launch2();
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    if(nav.driveTo(odo.getPosition(), TARGET_6, 1, 0.1)){

                        telemetry.addLine("at position #6");
//                        transfer();
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;
                case DRIVE_TO_TARGET_7:
                    if(nav.driveTo(odo.getPosition(), TARGET_7, .7, 0)){
                        telemetry.addLine("at position #7");
//                        transfer();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_8;
                    }
                    break;
                case DRIVE_TO_TARGET_8:
                    if(nav.driveTo(odo.getPosition(), TARGET_8, .7, 0)){
                        telemetry.addLine("at position #8");
//                        transfer();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                    }
                    break;
                case DRIVE_TO_TARGET_9:
                    if(nav.driveTo(odo.getPosition(), TARGET_9, .7, 0)){
                        telemetry.addLine("at position #9");
//                        transfer();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_10;
                    }
                    break;
                case DRIVE_TO_TARGET_10:
                    if(nav.driveTo(odo.getPosition(), TARGET_10, 1, 0)){
                        telemetry.addLine("at position #10");
//                        transfer();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_11;
                    }
                    break;
                case DRIVE_TO_TARGET_11:
                    if(nav.driveTo(odo.getPosition(), TARGET_11, .7, 0)){

                        telemetry.addLine("at position #11");
//                        transfer();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_12;
                    }
                    break;
//                case DRIVE_TO_TARGET_12:
//                    if(nav.driveTo(odo.getPosition(), TARGET_12, 1, 1)){
//
//                        telemetry.addLine("at position #11");
////                        transfer();
//                        stateMachine = StateMachine.DRIVE_TO_TARGET_13;
//                    }
//                    break;
//                case DRIVE_TO_TARGET_13:
//                    if(nav.driveTo(odo.getPosition(), TARGET_13, 1, 1)){
//
//                        telemetry.addLine("at position #11");
////                        transfer();
//                        stateMachine = StateMachine.DRIVE_TO_TARGET_14;
//                    }
//                    break;
            }


            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("current state:",stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.addData("Velocity",Launch.getVelocity());
//            telemetry.addData("color1",color1);
//            telemetry.addData("color2",color2);
//            telemetry.addData("color3",color3);
//            telemetry.addData("lastpos",lastpos);
//            telemetry.addData("currentpos",revolverpos);
            telemetry.update();

        }

    }
    private void launch21(){
        Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flick.setPosition(.78);
        sleep(1000);
//        Launch.setPower(.55);
        flick.setPosition(.3);
        Revolver.setTargetPosition(320+change);
        Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Revolver.setPower(.7);
        sleep(1800);
        flick.setPosition(.78);
        sleep(800);
        flick.setPosition(.3);
//        Launch.setPower(.75);
//    Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setTargetPosition(640+change);
        Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Revolver.setPower(.7);
        sleep(1800);
        flick.setPosition(.78);
        sleep(800);
        flick.setPosition(.3);
        sleep(700);
//        Launch.setVelocity(0);
//        Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    private void launch22(){
        flick.setPosition(.3);
        Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setTargetPosition(-320+change);
        Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Revolver.setPower(.8);
        sleep(1500);
        flick.setPosition(.78);
        sleep(600);
//        Launch.setPower(.55);
        flick.setPosition(.3);
        Revolver.setTargetPosition(0+change);
        Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Revolver.setPower(.8);
        sleep(1900);
        flick.setPosition(.78);
        sleep(600);
        flick.setPosition(.3);
//        Launch.setPower(.75);
//    Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setTargetPosition(320+change);
        Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Revolver.setPower(.8);
        sleep(1900);
        flick.setPosition(.78);
        sleep(600);
        flick.setPosition(.3);
//        sleep(700);
//        Launch.setVelocity(0);
//        Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    private void launch23(){Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setTargetPosition(320+change);
        Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Revolver.setPower(.7);
        sleep(1800);
        flick.setPosition(.78);
        sleep(1000);
//        Launch.setPower(.55);
        flick.setPosition(.3);
        Revolver.setTargetPosition(640+change);
        Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Revolver.setPower(.7);
        sleep(1800);
        flick.setPosition(.78);
        sleep(600);
        flick.setPosition(.3);
//        Launch.setPower(.75);
//    Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setTargetPosition(960+change);
        Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Revolver.setPower(.7);
        sleep(1800);
        flick.setPosition(.78);
        sleep(600);
        flick.setPosition(.3);
//        sleep(700);
//        Launch.setVelocity(0);
//        Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    private void intake(){
//        Launch.setVelocity(0);
        intake.setPower(1);
        Revolver.setTargetPosition(-160*8);
        Revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Revolver.setPower(.8);
//        sleep(500);
//        sleep(1000);
    }
    private void Stopintake() {
        intake.setPower(0);
//        Revolver.setPower(0);
//        Revolver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        side1.setPower(0);
//        side2.setPower(0);
//        if (Revolver.getCurrentPosition() < -160 * 7){
//            sleep(500);
//    }
//
//        if (Revolver.getCurrentPosition() > -160 * 7){
//            sleep(1000);
//
//        }
    }
    private  void SpinUp(){
        sleep(3000);

    }
    private  void SpinUpshort(){
        sleep(1500);

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
}
