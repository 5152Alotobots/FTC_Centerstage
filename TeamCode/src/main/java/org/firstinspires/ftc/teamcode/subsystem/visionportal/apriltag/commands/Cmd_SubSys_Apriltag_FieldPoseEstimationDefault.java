package org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag.commands;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.SubSys_Visionportal;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag.SubSys_Apriltag;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Cmd_SubSys_Apriltag_FieldPoseEstimationDefault extends CommandBase
{
    private SubSys_Visionportal subSysVisionportal;
    private SubSys_Apriltag subSysApriltag;
    private Telemetry telemetry;
    private Supplier<Pose2d> previousEstimatedPose;
    private AprilTagProcessor processor;


    public Cmd_SubSys_Apriltag_FieldPoseEstimationDefault(
            SubSys_Visionportal subSysVisionportal,
            SubSys_Apriltag subSysApriltag,
            Telemetry telemetry,
            Supplier<Pose2d> previousEstimatedPose) {
            this.subSysVisionportal = subSysVisionportal;
            this.subSysApriltag = subSysApriltag;
            this.telemetry = telemetry;
            this.previousEstimatedPose = previousEstimatedPose;

            addRequirements(subSysVisionportal, subSysApriltag);
    }
    @Override
    public void initialize() {
        telemetry.addLine("No vision processes requested, estimating pose with AprilTags...");
        processor = subSysApriltag.getProcessor();
    }

    @Override
    public void execute() {


    }


    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false; // Default command so never finishes
    }

    private Pair<Pose2d, Double> getPoseEstimation() {
        if (!subSysApriltag.getProcessor().getDetections().isEmpty()) {
            Pose2d prevPose = previousEstimatedPose.get();
            double prevX = prevPose.getX();
            double prevY = prevPose.getY();
            double prevRot = prevPose.getHeading();

            // Maybe add multicam support eventually?

            for (int detection = 0; detection < processor.getDetections().size(); detection++) {
                VectorF detectionVector = processor.getDetections().get(0).metadata.fieldPosition;
                double fieldX = detectionVector.get(0);
                double fieldY = detectionVector.get(1);
                double fieldZ = detectionVector.get(2);
                double robotToX = processor.getDetections().get(detection).ftcPose.x;
                double robotToY = processor.getDetections().get(detection).ftcPose.y;
                double robotToZ = processor.getDetections().get(detection).ftcPose.z;
            }
        }
        return new Pair<>(new Pose2d(new Translation2d(0, 0), new Rotation2d()), 0.0);
    }

}

