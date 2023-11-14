package org.firstinspires.ftc.teamcode.subsystem.drive.commands;

import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.Tuning.DEFAULT_SPEED;
import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.Tuning.SLOW_SPEED;
import static org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.Tuning.TURBO_SPEED;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.util.MathUtils;

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
    private final BooleanSupplier slowMode;
    private final BooleanSupplier turboMode;

    public Cmd_SubSys_Drive_JoystickDefault(
            SubSys_Drive subSysDrive,
            Telemetry telemetry,
            DoubleSupplier xCmd,
            DoubleSupplier yCmd,
            DoubleSupplier rotCmd,
            BooleanSupplier fieldOriented,
            BooleanSupplier slowMode,
            BooleanSupplier turboMode) {

        this.subSysDrive = subSysDrive;
        this.telemetry = telemetry;
        this.xCmd = xCmd;
        this.yCmd = yCmd;
        this.rotCmd = rotCmd;
        this.fieldOriented = fieldOriented;
        this.slowMode = slowMode;
        this.turboMode = turboMode;

        addRequirements(subSysDrive);
    }
    @Override
    public void initialize() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {
        double xCmdSlow = MathUtils.clamp(xCmd.getAsDouble(), -SLOW_SPEED, SLOW_SPEED);
        double yCmdSlow = MathUtils.clamp(yCmd.getAsDouble(), -SLOW_SPEED, SLOW_SPEED);
        double rotCmdSlow = MathUtils.clamp(rotCmd.getAsDouble(), -SLOW_SPEED, SLOW_SPEED);

        double xCmdDefault = MathUtils.clamp(xCmd.getAsDouble(), -DEFAULT_SPEED, DEFAULT_SPEED);
        double yCmdDefault = MathUtils.clamp(yCmd.getAsDouble(), -DEFAULT_SPEED, DEFAULT_SPEED);
        double rotCmdDefault = MathUtils.clamp(rotCmd.getAsDouble(), -DEFAULT_SPEED, DEFAULT_SPEED);

        double xCmdTurbo = MathUtils.clamp(xCmd.getAsDouble(), -TURBO_SPEED, TURBO_SPEED);
        double yCmdTurbo = MathUtils.clamp(yCmd.getAsDouble(), -TURBO_SPEED, TURBO_SPEED);
        double rotCmdTurbo = MathUtils.clamp(rotCmd.getAsDouble(), -TURBO_SPEED, TURBO_SPEED);

        if (slowMode.getAsBoolean()) {
            subSysDrive.drive(
                    xCmdSlow,
                    yCmdSlow,
                    rotCmdSlow,
                    fieldOriented.getAsBoolean()
            );
        } else if (turboMode.getAsBoolean()) {
            subSysDrive.drive(
                    xCmdTurbo,
                    yCmdTurbo,
                    rotCmdTurbo,
                    fieldOriented.getAsBoolean()
            );
        } else {
            subSysDrive.drive(
                    xCmdDefault,
                    yCmdDefault,
                    rotCmdDefault,
                    fieldOriented.getAsBoolean()
            );
        }
        telemetry.addData("POSE XY: ", subSysDrive.getPose().getX() + "," + subSysDrive.getPose().getY());
        telemetry.addData("COMMAND X: ", xCmd.getAsDouble());
        telemetry.addData("COMMAND Y: ", yCmd.getAsDouble());
        telemetry.addData("COMMAND ROT: ", rotCmd.getAsDouble());
        telemetry.addLine("---");
        MecanumDriveWheelSpeeds wheelSpeeds = subSysDrive.getWheelSpeeds();
        telemetry.addData("WHEEL SPEEDS: FL,FR,RL,RR", wheelSpeeds.frontLeftMetersPerSecond +","+wheelSpeeds.frontRightMetersPerSecond+","+wheelSpeeds.rearLeftMetersPerSecond+","+wheelSpeeds.rearRightMetersPerSecond);
        telemetry.addData("ENCODER TICKS:", subSysDrive.getEncoderTicks()[0]+","+subSysDrive.getEncoderTicks()[1]+","+subSysDrive.getEncoderTicks()[2]+","+subSysDrive.getEncoderTicks()[3]);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

