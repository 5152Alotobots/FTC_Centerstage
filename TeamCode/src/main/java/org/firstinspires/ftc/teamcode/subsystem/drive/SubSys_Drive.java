package org.firstinspires.ftc.teamcode.subsystem.drive;

import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.BACK_LEFT_LOCATION;
import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.BACK_RIGHT_LOCATION;
import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.FRONT_LEFT_LOCATION;
import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.FRONT_RIGHT_LOCATION;
import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.Specs.TICKS_PER_CENTIMETER;
import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.Specs.TICKS_PER_INCH;
import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.Tuning;
import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.startPose;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.MotorIds;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;

public class SubSys_Drive extends SubsystemBase
{
    private Motor m_frontLeftMotor;
    private Motor m_frontRightMotor;
    private Motor m_rearLeftMotor;
    private Motor m_rearRightMotor;
    private MecanumDrive m_mecanum;
    private SubSys_Gyro gyroSubSys;
    private Pose2d currentPose = startPose;
    private ElapsedTime time;
    private MecanumDriveKinematics m_kinematics;
    private MecanumDriveOdometry m_odometry;
/**
 * Creates a new SubSys_Drive
 * */
    public SubSys_Drive(SubSys_Gyro gyroSubSys, HardwareMap hwMap) {
        // Create motors
        m_frontLeftMotor = new Motor(hwMap, MotorIds.FRONT_LEFT);
        m_frontRightMotor = new Motor(hwMap, MotorIds.FRONT_RIGHT);
        m_rearLeftMotor = new Motor(hwMap, MotorIds.BACK_LEFT);
        m_rearRightMotor = new Motor(hwMap, MotorIds.BACK_RIGHT);

        // Create MecanumDrive
        m_mecanum = new MecanumDrive(m_frontLeftMotor, m_frontRightMotor, m_rearLeftMotor, m_rearRightMotor);

        // Invert motor
        m_frontRightMotor.setInverted(true);
        //m_rearRightMotor.setInverted(false);
        //m_frontLeftMotor.setInverted(true);
        //m_rearLeftMotor.setInverted(true); //! WORKS BUT AXIS BAD

        // Create gyrosubsys
        this.gyroSubSys = gyroSubSys;

        // Set max speed
        m_mecanum.setMaxSpeed(Tuning.MAX_SPD);

        // Start timer for pose calculations
        time = new ElapsedTime();
        time.reset();

        // Create kinematics
        m_kinematics = new MecanumDriveKinematics
                (
                        FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION,
                        BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION
                );

        m_odometry = new MecanumDriveOdometry
                (
                        m_kinematics, new Rotation2d(gyroSubSys.getYaw()),
                        new Pose2d(0.0, 0.0, new Rotation2d()
                        )
                );
    }

    /**
     * @param ticks The number of ticks measured from the drive motors
     * @return Centimeters
     * @deprecated Use ticksToInches as field layout is in inches
     * */
    public double ticksToCentimeters(double ticks) {
        return (ticks / TICKS_PER_CENTIMETER);
    }

    /**
     * Gets the wheel speeds from the drive
     * @return The wheel speeds in inches per second
     * */
    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                ticksToInches(m_frontLeftMotor.getRate()), ticksToInches(m_frontRightMotor.getRate()),
                ticksToInches(m_rearLeftMotor.getRate()), ticksToInches(m_rearRightMotor.getRate())
        );


    }

    public double ticksToInches(double ticks) {
        return (ticks / TICKS_PER_INCH);
    }

    /** Gets the encoder ticks of all wheels
     * @return Array [Front Left, Front Right, Rear Left, Rear Right]
     * */
    public Integer[] getEncoderTicks() {
        return new Integer[]{
                m_frontLeftMotor.getCurrentPosition(),
                m_frontRightMotor.getCurrentPosition(),
                m_rearLeftMotor.getCurrentPosition(),
                m_rearRightMotor.getCurrentPosition()
        };
    }
    /**
     * Drives the robot based on the following values
     * @param xSpeed The speed to drive the robot on the X-axis (0.0-1.0)
     * @param ySpeed The speed to drive the robot on the Y-axis (0.0-1.0)
     * @param rot The speed to rotate the robot (0.0-1.0)
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldOriented){
        if (!fieldOriented) {
            m_mecanum.driveRobotCentric(xSpeed, ySpeed, rot);
        } else {
            m_mecanum.driveFieldCentric(xSpeed, ySpeed, rot, gyroSubSys.getYaw());
        }
    }

    /**
     * Takes a given FTClib pose and converts it to a roadrunner pose
     * @return Roadrunner pose
     * */
    public com.acmerobotics.roadrunner.geometry.Pose2d convertPose(Pose2d ftclibPose) {
        return new com.acmerobotics.roadrunner.geometry.Pose2d(
                ftclibPose.getX(),
                ftclibPose.getY(),
                ftclibPose.getHeading()
        );
    }

    /**
     * Takes a given Roadrunner pose and converts it to a FTClib pose
     * @return FTClib pose
     * */
    public Pose2d convertPose(com.acmerobotics.roadrunner.geometry.Pose2d roadrunnerPose) {
        return new Pose2d(
                roadrunnerPose.getX(),
                roadrunnerPose.getY(),
                new Rotation2d(roadrunnerPose.getHeading())
        );
    }

    @Override
    public void periodic() {
        // Get gyro angle and update the pose of the robot
        Rotation2d gyroAngle = Rotation2d.fromDegrees(gyroSubSys.getYaw());
        SubSys_Drive_GlobalPoseStorage.currentPose = m_odometry.updateWithTime(time.seconds(), gyroAngle, getWheelSpeeds());
    }

    public Pose2d getPose() {
        return SubSys_Drive_GlobalPoseStorage.currentPose;
    }
}


