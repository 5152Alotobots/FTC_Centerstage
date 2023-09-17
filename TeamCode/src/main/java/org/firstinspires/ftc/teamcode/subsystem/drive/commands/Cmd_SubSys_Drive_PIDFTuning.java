package org.firstinspires.ftc.teamcode.subsystem.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.AngularPIDF;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.PIDF;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.SubSys_Visionportal;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class Cmd_SubSys_Drive_PIDFTuning extends CommandBase
{
    private SubSys_Drive subSysDrive;
    private Telemetry telemetry;

    // Define PIDF's
    private PIDFController xPid;
    private PIDFController yPid;
    private PIDFController rotPid;

    public Cmd_SubSys_Drive_PIDFTuning(SubSys_Drive subSysDrive, Telemetry telemetry) {
        this.subSysDrive = subSysDrive;
        this.telemetry = telemetry;
        addRequirements(subSysDrive);
    }
    @Override
    public void initialize() {
        xPid = new PIDFController(PIDF.kP, PIDF.kI, PIDF.kD, PIDF.kF);
        yPid = new PIDFController(PIDF.kP, PIDF.kI, PIDF.kD, PIDF.kF);
        rotPid = new PIDFController(AngularPIDF.kP, AngularPIDF.kI, AngularPIDF.kD, AngularPIDF.kF);

        // Set tolerances
        xPid.setTolerance(0.05); // Meters
        yPid.setTolerance(0.1); // Meters
        rotPid.setTolerance(1); // Deg Yaw
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {
        // Get the current pose of the robot
        Pose2d currentPose = subSysDrive.getPose();

        // Attempt to move the robot 0.5m forwards and left
        double xCmd = xPid.calculate(currentPose.getX(), 0.5);
        double yCmd = yPid.calculate(currentPose.getY(), 0.5);
        double rotCmd = rotPid.calculate(-currentPose.getRotation().getDegrees(), 45); // Make negative as CLOCKWISE IS NEGATIVE

        subSysDrive.drive(
                0,
                0,
                rotCmd,
                true
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
        telemetry.addData("ROT POSE: ", currentPose.getRotation().getDegrees());
        telemetry.update();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
