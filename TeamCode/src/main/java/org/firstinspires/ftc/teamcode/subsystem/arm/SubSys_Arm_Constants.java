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
        public static final double MAX_ROTATION_SPEED = 1;
        public static final double MAX_EXTENSION_SPEED = 1.0;

        /** Outer extension limit */
        public static final double OUTER_EXTEND_LIMIT = 64; //Soft 48cm
        /** Don't allow rotation down if the extension is above MAX_EXT_AT_INTAKE at this limit*/
        public static final double INTAKE_POS_SOFT_LIMIT = 22;
        /** Maximum value to allowed to extend at intake level */
        public static final double MAX_EXT_AT_INTAKE = 1.5;
    }

    public static class Specs {
        /** Number of rotate motor encoder ticks to equal one degree*/
        public static final double ROTATION_TICKS_PER_DEGREE = 98.9; //29.9056

        /** Number of extension motor encoder ticks to equal one centimeter*/
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
