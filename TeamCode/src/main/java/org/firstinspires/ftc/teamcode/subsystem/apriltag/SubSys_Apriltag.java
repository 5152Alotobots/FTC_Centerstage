package org.firstinspires.ftc.teamcode.subsystem.apriltag;

import org.firstinspires.ftc.teamcode.subsystem.apriltag.SubSys_Apriltag_Constants.ProcessorOptions;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class SubSys_Apriltag extends SubsystemBase
{
    AprilTagProcessor processor;

    public SubSys_Apriltag() {
        // Create a new AprilTag Processor object.
        processor = new AprilTagProcessor.Builder()
                // .setTagLibrary(CHANGE_ME) USE IF YOU WANT CUSTOM TAG LIB
                .setDrawTagID(ProcessorOptions.DRAW_TAG_ID)
                .setDrawTagOutline(ProcessorOptions.DRAW_TAG_OUTLINE)
                .setDrawAxes(ProcessorOptions.DRAW_AXES)
                .setDrawCubeProjection(ProcessorOptions.DRAW_CUBE_PROJECTION)
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
