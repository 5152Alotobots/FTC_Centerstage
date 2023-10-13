package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.arm.commands.Cmd_SubSys_Arm_JoystickDefault;
import org.firstinspires.ftc.teamcode.subsystem.arm.commands.Cmd_SubSys_Arm_RotateAndExtend;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_JoystickDefault;
import org.firstinspires.ftc.teamcode.subsystem.driverstation.SubSys_DriverStation;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.SubSys_Visionportal;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag.SubSys_Apriltag;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow;

@TeleOp(name="Teleop", group = "teleop")
public class Teleop extends CommandOpMode
{

    private SubSys_Tensorflow subSysTensorflow;
    private SubSys_Apriltag subSysApriltag;
    private SubSys_Visionportal subSysVisionportal;

    @Override
    public void initialize() {

        SubSys_DriverStation subSysDriverStation = new SubSys_DriverStation(gamepad1, gamepad2);
        SubSys_Gyro subSysGyro = new SubSys_Gyro(hardwareMap);
        SubSys_Drive subSysDrive = new SubSys_Drive(subSysGyro, hardwareMap);
        SubSys_Arm subSysArm = new SubSys_Arm(hardwareMap);
        // subSysApriltag = new SubSys_Apriltag();
        // subSysTensorflow = new SubSys_Tensorflow();
        // subSysVisionportal = new SubSys_Visionportal(subSysTensorflow, subSysApriltag, hardwareMap);
        register(subSysDriverStation, subSysGyro, subSysDrive, subSysArm);

        /* Default commands */
        subSysDrive.setDefaultCommand(
                new Cmd_SubSys_Drive_JoystickDefault(
                        subSysDrive,
                        telemetry,
                        subSysDriverStation::getDriverLeftX,
                        subSysDriverStation::getDriverLeftY,
                        subSysDriverStation::getDriverRightX,
                        () -> true
                ));
        subSysArm.setDefaultCommand(
                new Cmd_SubSys_Arm_JoystickDefault(
                        subSysArm,
                        telemetry,
                        subSysDriverStation::getCoDriverRightY,
                        subSysDriverStation::getCoDriverLeftY
                )
        );

        /* Arm positions */
        subSysDriverStation.armIntakePositionButton.whenPressed(
            new Cmd_SubSys_Arm_RotateAndExtend(
                    subSysArm,
                    telemetry,
                    () -> 0,
                    () -> 0
            ).withTimeout(2500)
        );
        subSysDriverStation.armMidPositionButton.whenPressed(
                new Cmd_SubSys_Arm_RotateAndExtend(
                        subSysArm,
                        telemetry,
                        () -> -45,
                        () -> 50
                ).withTimeout(3000)
        );
        subSysDriverStation.armHighPositionButton.whenPressed(
                new Cmd_SubSys_Arm_RotateAndExtend(
                        subSysArm,
                        telemetry,
                        () -> -90,
                        () -> 0
                ).withTimeout(3000)
        );

        /* Misc */
        subSysDriverStation.resetGyroButton.whenPressed(
                new InstantCommand(subSysGyro::resetYaw)
        );
    }

}
