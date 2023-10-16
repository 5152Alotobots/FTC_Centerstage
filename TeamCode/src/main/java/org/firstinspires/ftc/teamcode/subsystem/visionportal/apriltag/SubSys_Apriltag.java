package org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag.pose.AprilTagFieldLayout;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class SubSys_Apriltag extends SubsystemBase
{
    private AprilTagProcessor processor;
    private AprilTagLibrary cmLib;

    public SubSys_Apriltag() {
        cmLib = AprilTagFieldLayout.getCenterStageCentimetersLibrary();
        // Create a new AprilTag Processor object.
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(cmLib)
                .setDrawTagID(SubSys_Apriltag_Constants.ProcessorOptions.DRAW_TAG_ID)
                .setDrawTagOutline(SubSys_Apriltag_Constants.ProcessorOptions.DRAW_TAG_OUTLINE)
                .setDrawAxes(SubSys_Apriltag_Constants.ProcessorOptions.DRAW_AXES)
                .setDrawCubeProjection(SubSys_Apriltag_Constants.ProcessorOptions.DRAW_CUBE_PROJECTION)
                .build();
    }
    @Override
    public void periodic() {
        // Called once per scheduler run
    }

    public AprilTagProcessor getProcessor() {
        return processor;
    }
}
