package org.firstinspires.ftc.teamcode.subsystem.arm.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.github.freva.asciitable.AsciiTable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;

import java.text.DecimalFormat;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Arm_JoystickDefault extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SubSys_Arm subSysArm;
    private Telemetry telemetry;
    private DoubleSupplier rotCmd;
    private DoubleSupplier extCmd;
    private final DecimalFormat round;

    public Cmd_SubSys_Arm_JoystickDefault(
            SubSys_Arm subSysArm,
            Telemetry telemetry,
            DoubleSupplier rotCmd,
            DoubleSupplier extCmd) {

        this.subSysArm = subSysArm;
        this.telemetry = telemetry;
        this.rotCmd = rotCmd;
        this.extCmd = extCmd;
        addRequirements(subSysArm);

        // round to TWO decimal places
        round = new DecimalFormat("0.00");
    }

    @Override
    public void initialize() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {
        subSysArm.rotate(
                -rotCmd.getAsDouble() // Inverse so controls are push up to go down, back to go up
        );
        subSysArm.extend(
                extCmd.getAsDouble()
        );

        String poseExtString = round.format(subSysArm.getExtensionCentimeters());
        String poseRotString = round.format(subSysArm.getRotationDegrees());
        String cmdExtString = round.format(extCmd);
        String cmdRotString = round.format(rotCmd);

        String[] headers = {"", "CM/Deg", "Command"};
        String[][] data = {
                {"Ext", poseExtString, cmdExtString},
                {"Rot", poseRotString, cmdRotString}
        };
        String table = AsciiTable.getTable(headers, data);
        telemetry.addLine(table);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
