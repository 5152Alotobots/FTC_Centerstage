package org.firstinspires.ftc.teamcode.subsystem.drive;

import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.FRONT_LEFT_LOCATION;
import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.FRONT_RIGHT_LOCATION;
import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.REAR_LEFT_LOCATION;
import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.REAR_RIGHT_LOCATION;
import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.Specs.TICKS_PER_METER;
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
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.MotorIds;

import java.util.List;

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
        m_rearLeftMotor = new Motor(hwMap, MotorIds.REAR_LEFT);
        m_rearRightMotor = new Motor(hwMap, MotorIds.REAR_RIGHT);

        // Create MecanumDrive
        m_mecanum = new MecanumDrive(m_frontLeftMotor, m_frontRightMotor, m_rearLeftMotor, m_rearRightMotor);

        // Invert motors
        m_frontLeftMotor.setInverted(true);
        m_frontRightMotor.setInverted(true);
        m_rearLeftMotor.setInverted(true);
        m_frontRightMotor.setInverted(true);
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
                        REAR_LEFT_LOCATION, REAR_RIGHT_LOCATION
                );

        m_odometry = new MecanumDriveOdometry
                (
                        m_kinematics, new Rotation2d(gyroSubSys.getYaw()),
                        new Pose2d(0.0, 0.0, new Rotation2d()
                        )
                );
    }

    public double ticksToMeters(double ticks) {
        return (ticks / TICKS_PER_METER);
    }
    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
        ticksToMeters(m_frontLeftMotor.getRate()), ticksToMeters(m_frontRightMotor.getRate()),
        ticksToMeters(m_rearLeftMotor.getRate()), ticksToMeters(m_rearRightMotor.getRate())
        );

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

    @Override
    public void periodic() {
        // Get gyro angle and update the pose of the robot
        Rotation2d gyroAngle = Rotation2d.fromDegrees(gyroSubSys.getYaw());
        currentPose = m_odometry.updateWithTime(time.seconds(), gyroAngle, getWheelSpeeds());
    }

    public Pose2d getPose() {
        return currentPose;
    }
}


