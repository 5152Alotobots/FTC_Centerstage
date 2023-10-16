package org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.AngularPIDF;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.PIDF;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.SubSys_Visionportal;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class Cmd_SubSys_Tensorflow_NavigateToBest extends CommandBase
{
    private SubSys_Visionportal subSysVisionportal;
    private SubSys_Tensorflow subSysTensorflow;
    private TfodProcessor processor;
    private int orderMode;

    // Define PIDF's
    private PIDFController xPid;
    private PIDFController yPid;
    private PIDFController rotPid;

    public Cmd_SubSys_Tensorflow_NavigateToBest(SubSys_Visionportal subSysVisionportal, SubSys_Tensorflow subSysTensorflow, int orderMode) {
        this.subSysVisionportal = subSysVisionportal;
        this.subSysTensorflow = subSysTensorflow;
        this.orderMode = orderMode;
        addRequirements(subSysVisionportal, subSysTensorflow);
    }
    @Override
    public void initialize() {
        processor = subSysTensorflow.getProcessor();
        xPid = new PIDFController(PIDF.kP, PIDF.kI, PIDF.kD, PIDF.kF);
        yPid = new PIDFController(PIDF.kP, PIDF.kI, PIDF.kD, PIDF.kF);
        rotPid = new PIDFController(AngularPIDF.kP, AngularPIDF.kI, AngularPIDF.kD, AngularPIDF.kF);

        // Set tolerances
        xPid.setTolerance(0.1); // Meters
        yPid.setTolerance(0.1); // Meters
        rotPid.setTolerance(0.1); // Deg Yaw
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {
        Recognition bestRecognition = subSysTensorflow.getBestRecognition(orderMode);
        xPid.setSetPoint(subSysTensorflow.getCenter(bestRecognition)[0]); // Set set point of xPid



    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
