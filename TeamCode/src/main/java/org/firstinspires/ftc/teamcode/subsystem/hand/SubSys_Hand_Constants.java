package org.firstinspires.ftc.teamcode.subsystem.hand;

public class SubSys_Hand_Constants
{
    public static class MotorIds
    {
        public static final String WRIST = "wrist";
        public static final String LEFT_FRONT_SERVO = "lhgate";
        public static final String LEFT_BACK_SERVO = "lhdrop";
        public static final String RIGHT_FRONT_SERVO = "rhgate";
        public static final String RIGHT_BACK_SERVO = "rhdrop";
    }

    public static class SensorIds
    {
        public static final String LEFT_SENSOR = "leftColor";
        public static final String RIGHT_SENSOR = "rightColor";
    }

    public static class Tuning
    {
        // Rotation
        public static final double ROTATION_UP_LIMIT = 48;
        public static final double ROTATION_DOWN_LIMIT = -35;
        public static final double MAX_ROTATION_SPEED = 0.2;

        // Servos
        public static final double CLOSED_LEFT_FRONT_POS = 1;
        public static final double CLOSED_LEFT_BACK_POS = -0.7;
        public static final double CLOSED_RIGHT_FRONT_POS = 1;
        public static final double CLOSED_RIGHT_BACK_POS = -1;

        public static final double OPEN_LEFT_FRONT_POS = -1;
        public static final double OPEN_LEFT_BACK_POS = 1;
        public static final double OPEN_RIGHT_FRONT_POS = -1;
        public static final double OPEN_RIGHT_BACK_POS = 1;
    }

    public static class Specs {
        /** Number of rotate motor encoder ticks to equal one degree*/
        public static final double ROTATION_TICKS_PER_DEGREE = 0.8;

    }

    public static class RotationPIDF
    {
        public static final double kP = 0.009;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
    }

}
