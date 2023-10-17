package org.firstinspires.ftc.teamcode.subsystem.arm.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.github.freva.asciitable.AsciiTable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm_Constants.RotationPIDF;

import java.text.DecimalFormat;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Arm_RotateToDegree extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SubSys_Arm subSysArm;
    private Telemetry telemetry;
    private DoubleSupplier degrees;
    private final DecimalFormat round;

    // PIDF
    PIDFController rotPid;

    public Cmd_SubSys_Arm_RotateToDegree(
            SubSys_Arm subSysArm,
            Telemetry telemetry,
            DoubleSupplier degrees) {

        this.subSysArm = subSysArm;
        this.telemetry = telemetry;
        this.degrees = degrees;
        addRequirements(subSysArm);

        round = new DecimalFormat("0.00");
    }

    @Override
    public void initialize() {
        // Create new PIDF controller with values in SubSys_Arm_Constants
        rotPid = new PIDFController(
                RotationPIDF.kP,
                RotationPIDF.kI,
                RotationPIDF.kD,
                RotationPIDF.kF);
        rotPid.setTolerance(2); // Degrees

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}


    double rotCmd = 0; // Set rotCmd to zero, so it is not overwritten to zero every loop

    @Override
    public void execute() {
        rotCmd = rotPid.calculate(subSysArm.getRotationDegrees(), degrees.getAsDouble());

        subSysArm.rotate(rotCmd); // Rotate

        String cmdRotString = round.format(rotCmd);
        String setpointRot = round.format(subSysArm.getRotationDegrees());

        String[] headers = {"", "Setpoint", "Command"};
        String[][] data = {
                {"Rot", setpointRot, cmdRotString}
        };
        String table = AsciiTable.getTable(headers, data);
        telemetry.addLine(table);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return rotPid.atSetPoint();
    }
}
