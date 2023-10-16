package org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag;

public class SubSys_Apriltag_Constants
{
    public static class ProcessorOptions
    {
        public static final boolean DRAW_TAG_ID = true;
        public static final boolean DRAW_TAG_OUTLINE = true;
        public static final boolean DRAW_AXES = false;
        public static final boolean DRAW_CUBE_PROJECTION = false;
    }

    public static class DataAccuracy
    {
        public static final double OUTLIER_RANGE = 20.0; // Removes any detections that are greater than 20
    }
}
