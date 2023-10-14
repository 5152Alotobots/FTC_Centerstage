package org.firstinspires.ftc.teamcode.subsystem.hand.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand;

import java.util.function.BooleanSupplier;

public class Cmd_SubSys_Hand_JoystickDefault extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SubSys_Hand subSysShuttle;
    private Telemetry telemetry;
    private BooleanSupplier rotateUp;
    private BooleanSupplier rotateDown;

    public Cmd_SubSys_Hand_JoystickDefault(
            SubSys_Hand subSysShuttle,
            Telemetry telemetry,
            BooleanSupplier rotateUp,
            BooleanSupplier rotateDown) {

        this.subSysShuttle = subSysShuttle;
        this.telemetry = telemetry;
        this.rotateUp = rotateUp;
        this.rotateDown = rotateDown;
        addRequirements(subSysShuttle);
    }

    @Override
    public void initialize() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {
        // subSysShuttle.rotateWristToDegree(0);
        telemetry.addData("Current wrist pos:", subSysShuttle.getWristDegrees());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
