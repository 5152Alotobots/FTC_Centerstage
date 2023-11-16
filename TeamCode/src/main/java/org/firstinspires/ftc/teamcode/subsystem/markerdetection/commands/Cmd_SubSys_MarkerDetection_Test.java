package org.firstinspires.ftc.teamcode.subsystem.markerdetection.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.markerdetection.SubSys_MarkerDetection;

public class Cmd_SubSys_MarkerDetection_Test extends CommandBase
{
    private SubSys_MarkerDetection subSysMarkerDetection;
    private Telemetry telemetry;
    public Cmd_SubSys_MarkerDetection_Test(
            SubSys_MarkerDetection subSysMarkerDetection,
            Telemetry telemetry) {
        this.subSysMarkerDetection = subSysMarkerDetection;
        this.telemetry = telemetry;
        addRequirements(subSysMarkerDetection);
    }
    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        subSysMarkerDetection.initializeSensors();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        telemetry.addData("Position: ", subSysMarkerDetection.getMarkerPosition().toString());
        telemetry.addData("Red L,R", subSysMarkerDetection.getRed()[0]+","+subSysMarkerDetection.getRed()[1]);
    }

    /**
     * The action to take when the command ends.  Called when either the command finishes normally,
     * or when it interrupted/canceled.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
    }

    /**
     * Whether the command has finished.  Once a command finishes, the scheduler will call its
     * end() method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
