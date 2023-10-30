package org.firstinspires.ftc.teamcode.subsystem.hand.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Hand_JoystickDefault extends CommandBase
{
    private SubSys_Hand subSysHand;
    private Telemetry telemetry;
    private DoubleSupplier rotCmd;
    public Cmd_SubSys_Hand_JoystickDefault(
            SubSys_Hand subSysHand,
            Telemetry telemetry,
            DoubleSupplier rotCmd) {

        this.subSysHand = subSysHand;
        this.telemetry = telemetry;
        this.rotCmd = rotCmd;

        addRequirements(subSysHand);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        subSysHand.rotate(rotCmd.getAsDouble());
        telemetry.addData("handRotation", subSysHand.getRotationDegrees());
    }

    /**
     * The action to take when the command ends.  Called when either the command finishes normally,
     * or when it interrupted/canceled.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
    }

    /**
     * Whether the command has finished.  Once a command finishes, the scheduler will call its
     * end() method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
