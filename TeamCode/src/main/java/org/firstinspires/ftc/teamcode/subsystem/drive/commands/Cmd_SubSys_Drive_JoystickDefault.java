package org.firstinspires.ftc.teamcode.subsystem.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.github.freva.asciitable.AsciiTable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;

import java.text.DecimalFormat;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Drive_JoystickDefault extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SubSys_Drive subSysDrive;
    private final Telemetry telemetry;

    private final DoubleSupplier xCmd;
    private final DoubleSupplier yCmd;
    private final DoubleSupplier rotCmd;
    private final BooleanSupplier fieldOriented;
    private final DecimalFormat round;

    public Cmd_SubSys_Drive_JoystickDefault(
            SubSys_Drive subSysDrive,
            Telemetry telemetry,
            DoubleSupplier xCmd,
            DoubleSupplier yCmd,
            DoubleSupplier rotCmd,
            BooleanSupplier fieldOriented) {

        this.subSysDrive = subSysDrive;
        this.telemetry = telemetry;
        this.xCmd = xCmd;
        this.yCmd = yCmd;
        this.rotCmd = rotCmd;
        this.fieldOriented = fieldOriented;
        addRequirements(subSysDrive);

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
        subSysDrive.drive(
                xCmd.getAsDouble(),
                yCmd.getAsDouble(),
                rotCmd.getAsDouble(),
                fieldOriented.getAsBoolean()
        );

        String poseXString = round.format(subSysDrive.getPose().getX());
        String poseYString = round.format(subSysDrive.getPose().getY());
        String poseRotString = round.format(subSysDrive.getPose().getHeading());
        String cmdXString = round.format(xCmd.getAsDouble());
        String cmdYString = round.format(yCmd.getAsDouble());
        String cmdRotString = round.format(rotCmd.getAsDouble());

        String[] headers = {"", "Pose", "Command"};
        String[][] data = {
                {"X", poseXString, cmdXString},
                {"Y", poseYString, cmdYString},
                {"Rot", poseRotString, cmdRotString},
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

