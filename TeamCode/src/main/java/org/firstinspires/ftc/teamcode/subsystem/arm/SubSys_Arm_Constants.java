package org.firstinspires.ftc.teamcode.subsystem.arm;

public class SubSys_Arm_Constants
{
    public static class MotorIds
    {
        public static final String ROTATE = "lift";
        public static final String EXTEND = "extension";
    }

    public static class Tuning
    {
        public static final double MAX_SPD = 0.2;
    }

    public static class Specs {
        public static final double ROTATION_TICKS_PER_DEGREE = 29.9056;
        public static final double EXTENSION_TICKS_PER_METER = 0.0;
    }

    public static class RotationPIDF
    {
        public static final double kP = 0.06;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
    }

    public static class ExtensionPIDF
    {
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
    }
}
