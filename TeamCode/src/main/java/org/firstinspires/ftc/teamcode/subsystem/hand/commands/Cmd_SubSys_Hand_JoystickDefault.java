package org.firstinspires.ftc.teamcode.subsystem.hand.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Hand_JoystickDefault extends CommandBase
{
    private SubSys_Hand subSysHand;
    private SubSys_Arm subSysArm;
    private Telemetry telemetry;
    private DoubleSupplier rotCmd;

    // PIDF
    PIDFController rotPid;

    public Cmd_SubSys_Hand_JoystickDefault(
            SubSys_Hand subSysHand,
            SubSys_Arm subSysArm,
            Telemetry telemetry,
            DoubleSupplier rotCmd) {

        this.subSysHand = subSysHand;
        this.subSysArm = subSysArm;
        this.telemetry = telemetry;
        this.rotCmd = rotCmd;

        addRequirements(subSysHand);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        // Create new PIDF controller with values in SubSys_Hand_Constants
        rotPid = new PIDFController(
                SubSys_Hand_Constants.RotationPIDF.kP,
                SubSys_Hand_Constants.RotationPIDF.kI,
                SubSys_Hand_Constants.RotationPIDF.kD,
                SubSys_Hand_Constants.RotationPIDF.kF);
        rotPid.setTolerance(1); // Degrees
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        double rotCmdDouble = rotCmd.getAsDouble();
        if (rotCmdDouble == 0) {
            subSysHand.rotate(rotCmd.getAsDouble());
        } else {
            double pidCmd = rotPid.calculate(subSysArm.getRotationDegrees() - 30);
            subSysHand.rotate(pidCmd);
        }
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
