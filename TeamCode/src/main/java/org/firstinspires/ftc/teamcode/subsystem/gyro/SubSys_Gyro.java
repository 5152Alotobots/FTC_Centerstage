package org.firstinspires.ftc.teamcode.subsystem.gyro;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro_Constants.ImuConfig;

public class SubSys_Gyro extends SubsystemBase {

    private IMU imu;
    public SubSys_Gyro(HardwareMap hwMap) {
        // Create Gyro
        imu = hwMap.get(IMU.class, ImuConfig.IMU_NAME);
        imu.initialize(ImuConfig.IMU_PARAMETERS);
        // Reset yaw when initialized
        imu.resetYaw();
    }
    /**
     * Reset the yaw of the internal IMU
     * */
    public void resetYaw() {
        imu.resetYaw();
    }

    public Object getYawPitchRoll() {
        return imu.getRobotYawPitchRollAngles();
    }

    public double getYaw() {
        if (imu.getRobotYawPitchRollAngles() != null) {
            return imu.getRobotYawPitchRollAngles().getYaw(ImuConfig.IMU_ANGLE_UNIT);
        } else return 0;
    }

    public double getPitch() {
        if (imu.getRobotYawPitchRollAngles() != null) {
            return imu.getRobotYawPitchRollAngles().getPitch(ImuConfig.IMU_ANGLE_UNIT);
        } else return 0;
    }

    public double getRoll() {
        if (imu.getRobotYawPitchRollAngles() != null) {
            return imu.getRobotYawPitchRollAngles().getRoll(ImuConfig.IMU_ANGLE_UNIT);
        } else return 0;
    }

    @Override
    public void periodic() {
    }
}
