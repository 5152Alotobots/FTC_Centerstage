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

        /** Outer extension limit */
        public static final double OUTER_EXTEND_LIMIT = 67;
        /** Don't allow rotation if the extension is not at 0 at this limit*/
        public static final double INTAKE_EXTEND_ROTATE_SOFT_LIMIT = -30;
        /** Maximum value to allowed to extend at intake level */
        public static final double MAX_EXT_AT_INTAKE = 5;
    }

    public static class Specs {
        public static final double ROTATION_TICKS_PER_DEGREE = 29.9056;
        public static final double EXTENSION_TICKS_PER_CENTIMETER = 61.6812;
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
        public static final double kP = 0.12;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
    }
}
