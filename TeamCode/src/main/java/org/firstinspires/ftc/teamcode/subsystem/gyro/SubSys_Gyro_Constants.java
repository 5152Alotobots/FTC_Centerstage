package org.firstinspires.ftc.teamcode.subsystem.gyro;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SubSys_Gyro_Constants
{
    public static class ImuConfig {
        public static final String IMU_NAME = "imu";
        public static final AngleUnit IMU_ANGLE_UNIT = AngleUnit.DEGREES;
        public static final IMU.Parameters IMU_PARAMETERS =
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                );
    }
}
