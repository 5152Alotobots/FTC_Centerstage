package org.firstinspires.ftc.teamcode.subsystem.tensorflow.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.visionportal.SubSys_Visionportal;

public class Cmd_SubSys_Tensorflow_NavigateToBest extends CommandBase
{
    private SubSys_Visionportal visionportal;

    public Cmd_SubSys_Tensorflow_NavigateToBest(SubSys_Visionportal visionportal) {
        this.visionportal = visionportal;
        addRequirements(visionportal);
    }
    @Override
    public void initialize() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
