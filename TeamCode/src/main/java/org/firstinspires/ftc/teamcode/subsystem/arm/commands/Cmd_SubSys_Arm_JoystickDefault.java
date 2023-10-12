package org.firstinspires.ftc.teamcode.subsystem.arm.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Arm_JoystickDefault extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SubSys_Arm subSysArm;
    private Telemetry telemetry;
    private DoubleSupplier rotCmd;
    private DoubleSupplier extCmd;

    public Cmd_SubSys_Arm_JoystickDefault(
            SubSys_Arm subSysArm,
            Telemetry telemetry,
            DoubleSupplier rotCmd,
            DoubleSupplier extCmd) {

        this.subSysArm = subSysArm;
        this.telemetry = telemetry;
        this.rotCmd = rotCmd;
        this.extCmd = extCmd;
        addRequirements(subSysArm);
    }

    @Override
    public void initialize() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {
        subSysArm.rotate(
                -rotCmd.getAsDouble()
        );
        subSysArm.extend(
                extCmd.getAsDouble()
        );
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
