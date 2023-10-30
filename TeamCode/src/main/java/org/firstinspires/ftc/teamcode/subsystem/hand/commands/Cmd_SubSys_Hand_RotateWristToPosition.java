package org.firstinspires.ftc.teamcode.subsystem.hand.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.RotationPIDF;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Hand_RotateWristToPosition extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SubSys_Hand subSysHand;
    private Telemetry telemetry;
    private DoubleSupplier degrees;

    // PIDF
    PIDFController rotPid;

    public Cmd_SubSys_Hand_RotateWristToPosition(
            SubSys_Hand subSysHand,
            Telemetry telemetry,
            DoubleSupplier degrees) {

        this.subSysHand = subSysHand;
        this.telemetry = telemetry;
        this.degrees = degrees;
        addRequirements(subSysHand);
    }

    @Override
    public void initialize() {
        // Create new PIDF controller with values in SubSys_Hand_Constants
        rotPid = new PIDFController(
                RotationPIDF.kP,
                RotationPIDF.kI,
                RotationPIDF.kD,
                RotationPIDF.kF);
        rotPid.setTolerance(1); // Degrees

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}


    double rotCmd = 0; // Set rotCmd to zero, so it is not overwritten to zero every loop

    @Override
    public void execute() {
        rotCmd = rotPid.calculate(subSysHand.getRotationDegrees(), degrees.getAsDouble());
        telemetry.addData("wristCmd", rotCmd);
        telemetry.addData("wristPos", subSysHand.getRotationDegrees());
        telemetry.addData("wristSetpoint", rotPid.atSetPoint());

        subSysHand.rotate(rotCmd);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return rotPid.atSetPoint();
    }
}
