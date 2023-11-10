package org.firstinspires.ftc.teamcode.subsystem.intake.commands;

import static org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake_Constants.Tuning.ARM_DEGREE_TOLERANCE;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Intake_RunWhenArmDegree extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SubSys_Intake subSysIntake;
    private final SubSys_Arm subSysArm;
    private Telemetry telemetry;
    private DoubleSupplier rotCmd;
    private DoubleSupplier armDegreeToRotateAt;
    private BooleanSupplier manualOveride;

    public Cmd_SubSys_Intake_RunWhenArmDegree(
            SubSys_Intake subSysIntake,
            SubSys_Arm subSysArm,
            Telemetry telemetry,
            DoubleSupplier rotCmd,
            DoubleSupplier armDegreeToRotateAt,
            BooleanSupplier manualOveride) {

        this.subSysIntake = subSysIntake;
        this.subSysArm = subSysArm;
        this.telemetry = telemetry;
        this.rotCmd = rotCmd;
        this.armDegreeToRotateAt = armDegreeToRotateAt;
        this.manualOveride = manualOveride;
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
        boolean upperlimit = (subSysArm.getRotationDegrees() + ARM_DEGREE_TOLERANCE) > armDegreeToRotateAt.getAsDouble();
        boolean lowerlimit = (subSysArm.getRotationDegrees() - ARM_DEGREE_TOLERANCE) < armDegreeToRotateAt.getAsDouble();
        boolean inZone = upperlimit && lowerlimit;
        if (!inZone || manualOveride.getAsBoolean()) {
            // Force no output
            subSysIntake.intake(0);
        } else {
            subSysIntake.intake(rotCmd.getAsDouble());
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
