package org.firstinspires.ftc.teamcode.subsystem.arm.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm_Constants.RotationPIDF;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm_Constants.ExtensionPIDF;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Arm_RotateAndExtend extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SubSys_Arm subSysArm;
    private Telemetry telemetry;
    private DoubleSupplier degrees;
    private DoubleSupplier centimeters;

    // PIDF
    private PIDFController extPid;
    private PIDFController rotPid;

    public Cmd_SubSys_Arm_RotateAndExtend(
            SubSys_Arm subSysArm,
            Telemetry telemetry,
            DoubleSupplier degrees,
            DoubleSupplier centimeters) {

        this.subSysArm = subSysArm;
        this.telemetry = telemetry;
        this.degrees = degrees;
        this.centimeters = centimeters;
        addRequirements(subSysArm);
    }

    @Override
    public void initialize() {
        // Create new PIDF controller with values in SubSys_Arm_Constants
        rotPid = new PIDFController(RotationPIDF.kP, RotationPIDF.kI, RotationPIDF.kD, RotationPIDF.kF);
        rotPid.setTolerance(2); // Degrees

        // Create new PIDF controller with values in SubSys_Arm_Constants
        extPid = new PIDFController(ExtensionPIDF.kP, ExtensionPIDF.kI, ExtensionPIDF.kD, ExtensionPIDF.kF);
        extPid.setTolerance(0.2); // Centimeters
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}


    double extCmd = 0; // Set rotCmd to zero, so it is not overwritten to zero every loop
    double rotCmd = 0;

    @Override
    public void execute() {
        extCmd = extPid.calculate(subSysArm.getExtensionCentimeters(), centimeters.getAsDouble());
        telemetry.addData("extCmd", extCmd);
        telemetry.addData("extSetpoint", extPid.atSetPoint());

        subSysArm.extend(extCmd);

        rotCmd = rotPid.calculate(subSysArm.getRotationDegrees(), degrees.getAsDouble());
        telemetry.addData("rotCmd", rotCmd);
        telemetry.addData("rotSetpoint", rotPid.atSetPoint());

        subSysArm.rotate(rotCmd);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return extPid.atSetPoint() && rotPid.atSetPoint();
    }
}
