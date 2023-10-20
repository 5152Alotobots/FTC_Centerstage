package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class SubSys_Drive_Constants
{
    public static class MotorIds
    {
        public static final String FRONT_LEFT = "frontLeftMotor";
        public static final String FRONT_RIGHT = "frontRightMotor";
        public static final String BACK_LEFT = "backLeftMotor";
        public static final String BACK_RIGHT = "backRightMotor";
    }

    // Locations of the wheels relative to the robot center. X forward/back Y left/right
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.134, 0.152);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.134, -0.152);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-0.134, 0.152);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-0.134, -0.152);

    // Start pose
    public static final Pose2d startPose = new Pose2d(0, 0, new Rotation2d());

    public static class Tuning
    {
        public static final double MAX_SPD = 1.0;
        public static final double MAX_ROT_SPD = 0.0; // NOT IMPLEMENTED
    }

    public static class Specs
    {
        public static final double WHEEL_DIAMETER = 0.075; // Meters
        public static final double TICKS_PER_ROTATION = 515; // Ticks
        public static final double TICKS_PER_CENTIMETER = 21.8684; // Ticks
    }

    public static class PIDF
    {
        public static final double kP = 0.1;
        public static final double kI = 0.1;
        public static final double kD = 0.1;
        public static final double kF = 0.1;
    }

    public static class AngularPIDF
    {
        public static final double kP = 0.022;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
    }
}
