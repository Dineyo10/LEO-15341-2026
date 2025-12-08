package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Fiducial;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;
import java.util.Locale;

@Autonomous(name="OliverAuto", group="Auto")
//@Disabled

public class OliverAuto extends LinearOpMode {

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    //    private CRServo Intake1;
//    private CRServo Intake2;
    private DcMotor Launch1;
    private DcMotor Launch2;
    private int AprilTagID;

    private DcMotor Conveyor;

    private CRServo Stopper;

    private Limelight3A limelight;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
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
//+x is forward
//+y is left
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,1800,0,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, 1800, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES,0);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_6 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_7 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_8 = new Pose2D(DistanceUnit.MM, -0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_9 = new Pose2D(DistanceUnit.MM, -0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_10 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_11 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
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
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.start();

//        limelight.pipelineSwitch(0);
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back");

//        Intake1 = hardwareMap.get(CRServo.class, "Intake1");
//        Intake2 = hardwareMap.get(CRServo.class, "Intake2");

        Launch1 = hardwareMap.get(DcMotor.class, "Launch1");
        Launch2 = hardwareMap.get(DcMotor.class, "Launch2");

        Conveyor = hardwareMap.get(DcMotor.class, "Conveyor");

        Stopper = hardwareMap.get(CRServo.class, "Stopper");



        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        limelight.pipelineSwitch(0);


//        color = hardwareMap.get(ColorSensor.class, "color");
        Launch1.setDirection(DcMotorSimple.Direction.REVERSE);
//        Intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        Conveyor.setDirection(DcMotor.Direction.REVERSE);

//        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Launch1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Launch2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-190.5, -13.49375); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

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
            telemetry.addLine("21");
        }
        if(AprilTagID== 22){
            telemetry.addLine("22");
        }
        if(AprilTagID== 23){
            telemetry.addLine("23");
        }
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();
////

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
//                }
//

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

                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;
                case DRIVE_TO_TARGET_1:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), TARGET_1, .7, 2)){
//                    Stopper.setPower(.3);
                        telemetry.addLine("at position #1!");
//                        if (AprilTagID==21) {
                            stateMachine = StateMachine.DRIVE_TO_TARGET_2;
//                        }
//
//                        else if (AprilTagID==22) {
//                            stateMachine = StateMachine.DRIVE_TO_TARGET_3;
//                        }
//
//                        else {
//                            stateMachine = StateMachine.DRIVE_TO_TARGET_4;
//                        }
//                        launch();
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    //drive to the second target
                    if (nav.driveTo(odo.getPosition(), TARGET_2, .7, 2)){
//                        Stopper.setPower(.3);
                    Shoot();
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
//                    sleep(30000);
                    break;
                case DRIVE_TO_TARGET_3:
                    if(nav.driveTo(odo.getPosition(), TARGET_3, .7, 2)){
                        telemetry.addLine("at position #3");
//                        Launch1.setPower(.2);


                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if(nav.driveTo(odo.getPosition(),TARGET_4,.7,2)){
                        telemetry.addLine("at position #4");
//                        Conveyor.setPower(.8);
//                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                        stateMachine = StateMachine.AT_TARGET;

                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if(nav.driveTo(odo.getPosition(),TARGET_5,.7,0)){
                        telemetry.addLine("at position #5!");

                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    if(nav.driveTo(odo.getPosition(), TARGET_6, .7, 0)){

                        telemetry.addLine("at position #6");
//                        transfer();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_7;
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

            telemetry.update();

        }
    }
    public void Shoot(){
        Launch1.setPower(.39);
        Launch2.setPower(.39);
        sleep(2000);
        Stopper.setPower(1);
        sleep(2500);
        Stopper.setPower(0);
        sleep(1000);
        Conveyor.setPower(1);
        sleep(2000);
        Stopper.setPower(1);
        sleep(2000);
        Launch1.setPower(0);
        Launch2.setPower(0);
        Stopper.setPower(0);
        Conveyor.setPower(0);
    }
    public void Intake(){
        Conveyor.setPower(1);

    }
    public void launch(){
        Launch1.setPower(.41);
        Launch2.setPower(.415);
        sleep(4000);
        Stopper.setPower(1);
        sleep(3000);
        Stopper.setPower(0);
        sleep(3000);
        Conveyor.setPower(1);
        sleep(500);
        Stopper.setPower(1);
        Conveyor.setPower(1);
        sleep(500);
        Conveyor.setPower(0);
        sleep(2000);
        Stopper.setPower(0);
        sleep(2000);
        Stopper.setPower(1);
        sleep(2000);
        Conveyor.setPower(1);
        sleep(3000);
        Launch1.setPower(0);
        Launch2.setPower(0);
        Stopper.setPower(0);
        Conveyor.setPower(0);

    }

}
