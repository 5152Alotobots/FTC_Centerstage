package org.firstinspires.ftc.teamcode.subsystem.drive;

import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.Tuning;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.MotorIds;

public class SubSys_Drive extends SubsystemBase
{
    private Motor m_frontLeftMotor;
    private Motor m_frontRightMotor;
    private Motor m_rearLeftMotor;
    private Motor m_rearRightMotor;
    private MecanumDrive m_mecanum;
    private SubSys_Gyro gyroSubSys;
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

        // Create gyrosubsys
        this.gyroSubSys = gyroSubSys;

        // Set max speed
        m_mecanum.setMaxSpeed(Tuning.MAX_SPD);
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
        // Called once per scheduler run
    }
}


