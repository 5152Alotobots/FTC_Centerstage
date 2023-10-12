package org.firstinspires.ftc.teamcode.subsystem.arm.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm_Constants.ExtensionPIDF;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Arm_ExtendToMeter extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SubSys_Arm subSysArm;
    private Telemetry telemetry;
    private DoubleSupplier meters;

    // PIDF
    private PIDFController extPid;

    public Cmd_SubSys_Arm_ExtendToMeter(
            SubSys_Arm subSysArm,
            Telemetry telemetry,
            DoubleSupplier meters) {

        this.subSysArm = subSysArm;
        this.telemetry = telemetry;
        this.meters = meters;
        addRequirements(subSysArm);
    }

    @Override
    public void initialize() {
        // Create new PIDF controller with values in SubSys_Arm_Constants
        extPid = new PIDFController(
                ExtensionPIDF.kP,
                ExtensionPIDF.kI,
                ExtensionPIDF.kD,
                ExtensionPIDF.kF);
        extPid.setTolerance(0.01); // Meters

        //TEMP FOR TESTING, REMOVE TO USE ABSOLUTE POSITION
        subSysArm.resetExtensionPosition();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}


    double extCmd = 0; // Set rotCmd to zero, so it is not overwritten to zero every loop

    @Override
    public void execute() {
        extCmd = extPid.calculate(subSysArm.getRotationDegrees(), meters.getAsDouble());
        telemetry.addData("extCmd", extCmd);
        telemetry.addData("armIsAtSetpoint", extPid.atSetPoint());

        subSysArm.rotate(extCmd);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return extPid.atSetPoint();
    }
}
