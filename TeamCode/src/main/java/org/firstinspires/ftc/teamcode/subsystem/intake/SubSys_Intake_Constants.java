package org.firstinspires.ftc.teamcode.subsystem.intake;

public class SubSys_Intake_Constants
{
    public static class MotorIds
    {
        public static final String INTAKE = "intake";
    }

    public static class Tuning
    {
        /** If less than this distance, box is occupied*/
        public static final double OCCUPIED_DISTANCE_CM = 3;

        /** +/- to detect that the arm is close enough to run the intake.*/
        public static final double ARM_DEGREE_TOLERANCE = 2;
    }

    public static class Specs {
        /** Number of rotate motor encoder ticks to rotate 1deg*/
        public static final double INTAKE_TICKS_PER_DEGREE = 29.9056;
    }

    public static class IntakePIDF
    {
        public static final double kP = 0.06;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
    }

}
