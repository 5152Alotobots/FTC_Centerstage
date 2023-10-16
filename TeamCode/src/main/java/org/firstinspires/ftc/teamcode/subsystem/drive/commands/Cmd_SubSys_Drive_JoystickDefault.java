package org.firstinspires.ftc.teamcode.subsystem.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;

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
    }
    @Override
    public void initialize() {}

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

        telemetry.addData("POSE XY: ", subSysDrive.getPose().getX() + "," + subSysDrive.getPose().getY());
        telemetry.addData("COMMAND X: ", xCmd.getAsDouble());
        telemetry.addData("COMMAND Y: ", yCmd.getAsDouble());
        telemetry.addData("COMMAND ROT: ", rotCmd.getAsDouble());
        telemetry.addLine("---");
        MecanumDriveWheelSpeeds wheelSpeeds = subSysDrive.getWheelSpeeds();
        telemetry.addData("WHEEL SPEEDS: FL,FR,RL,RR", wheelSpeeds.frontLeftMetersPerSecond +","+wheelSpeeds.frontRightMetersPerSecond+","+wheelSpeeds.rearLeftMetersPerSecond+","+wheelSpeeds.rearRightMetersPerSecond);
        telemetry.addData("ENCODER TICKS RR:", subSysDrive.getEncoderTicks()[3]);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

