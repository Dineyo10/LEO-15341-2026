package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Fiducial;
import com.qualcomm.hardware.limelightvision.LLResult.*;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LimeLight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//impor com.qualconm.robotcore.eventLoop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.teamcode.mechanisms.TestBench;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public class ApriltagLimelightTest extends LinearOpMode {
    private Limelight3A limelight;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

//    private bench = new TestBench();
    private double distance;
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        limelight.pipelineSwitch(0);

//        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
//
            if (result != null && result.isValid()) {
                List<FiducialResult> fiducials = result.getFiducialResults();

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int AprilTagID = fiducial.getFiducialId();

        telemetry.addData("Detected Apriltag", AprilTagID);
            telemetry.addData("angle",result.getTx());
            telemetry.addData("distance",result.getTa());


            //CODE WORK OR ELSE

//            telemetry.addData("TagId", );
//            } else {
//                telemetry.addData("Limelight", "No Targets");
            }
//            telemetry.update();
        }
        }
    }
}















