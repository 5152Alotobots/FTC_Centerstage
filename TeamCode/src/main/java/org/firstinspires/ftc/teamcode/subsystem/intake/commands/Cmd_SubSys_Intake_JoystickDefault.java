package org.firstinspires.ftc.teamcode.subsystem.intake.commands;

import static org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake_Constants.Tuning.ARM_DEGREE_TOLERANCE;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Intake_JoystickDefault extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SubSys_Intake subSysIntake;
    private final SubSys_Arm subSysArm;
    private Telemetry telemetry;
    private DoubleSupplier intakeCmd;

    public Cmd_SubSys_Intake_JoystickDefault(
            SubSys_Intake subSysIntake,
            SubSys_Arm subSysArm,
            Telemetry telemetry,
            DoubleSupplier intakeCmd) {

        this.subSysIntake = subSysIntake;
        this.subSysArm = subSysArm;
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
        boolean upperlimit = (subSysArm.getRotationDegrees() + ARM_DEGREE_TOLERANCE) > 0;
        boolean lowerlimit = (subSysArm.getRotationDegrees() - ARM_DEGREE_TOLERANCE) < 0;
        boolean inZone = upperlimit && lowerlimit;

        if (intakeCmd.getAsDouble() != 0) {
            subSysIntake.intake(
                    intakeCmd.getAsDouble()
            );
        } else if (inZone) {
            subSysIntake.intake(1); // Intake full speed in
        } else {
            subSysIntake.intake(0);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
