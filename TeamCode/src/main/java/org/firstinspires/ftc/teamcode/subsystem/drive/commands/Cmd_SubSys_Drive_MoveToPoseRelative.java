package org.firstinspires.ftc.teamcode.subsystem.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.AngularPIDF;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.PIDF;

public class Cmd_SubSys_Drive_MoveToPoseRelative extends CommandBase
{
    private SubSys_Drive subSysDrive;
    private Telemetry telemetry;
    private Pose2d pose;
    private Pose2d startPose;

    // Define PIDF's
    private PIDFController xPid;
    private PIDFController yPid;
    private PIDFController rotPid;

    /**
     * Moves the robot relative to where it is currently. (X = Forward/Back, Y=Left/Right)
     * @param pose Pose2d of where to move relative (inches)
     * */
    public Cmd_SubSys_Drive_MoveToPoseRelative(
            SubSys_Drive subSysDrive,
            Telemetry telemetry,
            Pose2d pose) {
        this.subSysDrive = subSysDrive;
        this.telemetry = telemetry;
        this.pose = pose;

        addRequirements(subSysDrive);
    }
    @Override
    public void initialize() {
        startPose = new Pose2d();

        xPid = new PIDFController(PIDF.kP, PIDF.kI, PIDF.kD, PIDF.kF);
        yPid = new PIDFController(PIDF.kP, PIDF.kI, PIDF.kD, PIDF.kF);
        rotPid = new PIDFController(AngularPIDF.kP, AngularPIDF.kI, AngularPIDF.kD, AngularPIDF.kF);

        // Set tolerances
        xPid.setTolerance(0.05); // in
        yPid.setTolerance(0.1); // in
        rotPid.setTolerance(1); // Deg Yaw
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {

        // Attempt to move the robot 0.5m forwards and left
        double xCmd = xPid.calculate(startPose.getX(), pose.getX());
        double yCmd = yPid.calculate(startPose.getY(), pose.getY());
        double rotCmd = rotPid.calculate(-startPose.getRotation().getDegrees(), pose.getRotation().getDegrees()); // Make negative as CLOCKWISE IS NEGATIVE

        subSysDrive.drive(
                xCmd,
                yCmd,
                rotCmd,
                false
        );

        telemetry.addData("POSE XY: ", subSysDrive.getPose().getX() + "," + subSysDrive.getPose().getY());
        telemetry.addData("SETPOINT X", xPid.getSetPoint());
        telemetry.addData("COMMAND X", xCmd);
        telemetry.addData("AT SETPOINT X", xPid.atSetPoint());
        telemetry.addLine("---");
        telemetry.addData("SETPOINT Y", yPid.getSetPoint());
        telemetry.addData("COMMAND Y", yCmd);
        telemetry.addData("AT SETPOINT Y", yPid.atSetPoint());
        telemetry.addLine("---");
        telemetry.addData("SETPOINT ROT", rotPid.getSetPoint());
        telemetry.addData("COMMAND ROT", rotCmd);
        telemetry.addData("AT SETPOINT ROT", rotPid.atSetPoint());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (xPid.atSetPoint() && yPid.atSetPoint() && rotPid.atSetPoint());
    }
}
