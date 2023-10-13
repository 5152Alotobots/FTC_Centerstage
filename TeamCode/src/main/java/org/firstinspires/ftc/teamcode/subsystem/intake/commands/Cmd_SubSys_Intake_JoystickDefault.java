package org.firstinspires.ftc.teamcode.subsystem.intake.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Intake_JoystickDefault extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SubSys_Intake subSysIntake;
    private Telemetry telemetry;
    private DoubleSupplier intakeCmd;

    public Cmd_SubSys_Intake_JoystickDefault(
            SubSys_Intake subSysIntake,
            Telemetry telemetry,
            DoubleSupplier intakeCmd) {

        this.subSysIntake = subSysIntake;
        this.telemetry = telemetry;
        this.intakeCmd = intakeCmd;
        addRequirements(subSysIntake);
    }

    @Override
    public void initialize() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {
        subSysIntake.intake(
                intakeCmd.getAsDouble()
        );
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
