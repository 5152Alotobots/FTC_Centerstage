package org.firstinspires.ftc.teamcode.subsystem.arm.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.github.freva.asciitable.AsciiTable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm_Constants.ExtensionPIDF;

import java.text.DecimalFormat;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Arm_ExtendToCentimeter extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SubSys_Arm subSysArm;
    private Telemetry telemetry;
    private DoubleSupplier centimeters;
    private DecimalFormat round;

    // PIDF
    private PIDFController extPid;

    public Cmd_SubSys_Arm_ExtendToCentimeter(
            SubSys_Arm subSysArm,
            Telemetry telemetry,
            DoubleSupplier centimeters) {

        this.subSysArm = subSysArm;
        this.telemetry = telemetry;
        this.centimeters = centimeters;
        addRequirements(subSysArm);

        round = new DecimalFormat("0.00");
    }

    @Override
    public void initialize() {
        // Create new PIDF controller with values in SubSys_Arm_Constants
        extPid = new PIDFController(
                ExtensionPIDF.kP,
                ExtensionPIDF.kI,
                ExtensionPIDF.kD,
                ExtensionPIDF.kF);
        extPid.setTolerance(0.5); // centimeter

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}


    double extCmd = 0; // Set rotCmd to zero, so it is not overwritten to zero every loop

    @Override
    public void execute() {
        extCmd = extPid.calculate(subSysArm.getExtensionCentimeters(), centimeters.getAsDouble());
        telemetry.addData("extCmd", extCmd);
        telemetry.addData("extCentimeter", subSysArm.getExtensionCentimeters());
        telemetry.addData("extTicks", subSysArm.getExtensionTicks());
        telemetry.addData("extSetpoint", extPid.atSetPoint());

        subSysArm.extend(extCmd);

        String cmdExtString = round.format(extCmd);
        String setpointExt = round.format(subSysArm.getExtensionCentimeters());

        String[] headers = {"", "Setpoint", "Command"};
        String[][] data = {
                {"Ext", setpointExt, cmdExtString},
        };
        String table = AsciiTable.getTable(headers, data);
        telemetry.addLine(table);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return extPid.atSetPoint();
    }
}
