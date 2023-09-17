package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_JoystickDefault;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_JoystickRotateAroundRecognition;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_PIDFTuning;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.SubSys_Visionportal;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag.SubSys_Apriltag;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow;

@TeleOp(name="Teleop", group = "teleop")
public class Teleop extends CommandOpMode
{

    private SubSys_Gyro subSysGyro;
    private SubSys_Drive subSysDrive;
    private SubSys_Tensorflow subSysTensorflow;
    private SubSys_Apriltag subSysApriltag;
    private SubSys_Visionportal subSysVisionportal;
    private GamepadEx m_driverController;

    @Override
    public void initialize() {
        subSysGyro = new SubSys_Gyro(hardwareMap);
        subSysDrive = new SubSys_Drive(subSysGyro, hardwareMap);
        subSysApriltag = new SubSys_Apriltag();
        subSysTensorflow = new SubSys_Tensorflow();
        subSysVisionportal = new SubSys_Visionportal(subSysTensorflow, subSysApriltag, hardwareMap);
        m_driverController = new GamepadEx(gamepad1);
        register(subSysGyro, subSysDrive, subSysApriltag, subSysTensorflow, subSysVisionportal);

        subSysDrive.setDefaultCommand(
                new Cmd_SubSys_Drive_JoystickDefault(
                        subSysDrive,
                        telemetry,
                        () -> m_driverController.getLeftX(),
                        () -> m_driverController.getLeftY(),
                        () -> m_driverController.getRightX(),
                        () -> true
                ));

        GamepadButton runPIDF = new GamepadButton(
                m_driverController,
                GamepadKeys.Button.A
        );

        runPIDF.whenHeld(new Cmd_SubSys_Drive_JoystickRotateAroundRecognition(
                subSysDrive,
                subSysVisionportal,
                () -> m_driverController.getLeftX(),
                () -> m_driverController.getLeftY(),
                () -> false
        ));
    }

}
