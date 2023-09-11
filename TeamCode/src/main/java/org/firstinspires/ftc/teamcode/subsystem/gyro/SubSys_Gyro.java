package org.firstinspires.ftc.teamcode.subsystem.gyro;

import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro_Constants.ImuConfig;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

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
        return imu.getRobotYawPitchRollAngles().getYaw(ImuConfig.IMU_ANGLE_UNIT);
    }

    public double getPitch() {
        return imu.getRobotYawPitchRollAngles().getPitch(ImuConfig.IMU_ANGLE_UNIT);
    }

    public double getRoll() {
        return imu.getRobotYawPitchRollAngles().getRoll(ImuConfig.IMU_ANGLE_UNIT);
    }

    @Override
    public void periodic() {
    }
}
