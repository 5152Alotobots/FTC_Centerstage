package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_JoystickDefault;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_PIDFTuning;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;

@TeleOp(name="PIDFTuning", group = "PIDFTuning")
public class PIDFTuning extends CommandOpMode
{

    private SubSys_Gyro subSysGyro;
    private SubSys_Drive subSysDrive;
    private GamepadEx m_driverController;

    @Override
    public void initialize() {
        subSysGyro = new SubSys_Gyro(hardwareMap);
        subSysDrive = new SubSys_Drive(subSysGyro, hardwareMap);
        m_driverController = new GamepadEx(gamepad1);
        register(subSysDrive, subSysGyro);

        GamepadButton runPIDF = new GamepadButton(
                m_driverController,
                GamepadKeys.Button.A
        );

        runPIDF.whenPressed(new Cmd_SubSys_Drive_PIDFTuning(subSysDrive, telemetry));
    }

}
