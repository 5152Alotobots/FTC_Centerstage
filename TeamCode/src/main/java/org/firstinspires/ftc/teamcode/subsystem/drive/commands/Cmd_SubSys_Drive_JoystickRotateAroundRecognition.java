package org.firstinspires.ftc.teamcode.subsystem.drive.commands;

import static org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow_Constants.OrderMode.LARGEST;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_Constants.AngularPIDF;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.SubSys_Visionportal;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Drive_JoystickRotateAroundRecognition extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SubSys_Drive subSysDrive;
    private final SubSys_Visionportal subSysVisionportal;
    private final DoubleSupplier xCmd;
    private final DoubleSupplier yCmd;
    private final BooleanSupplier fieldOriented;

    // PIDF
    PIDFController rotPid;

    public Cmd_SubSys_Drive_JoystickRotateAroundRecognition(
            SubSys_Drive subSysDrive,
            SubSys_Visionportal subSysVisionportal,
            DoubleSupplier xCmd,
            DoubleSupplier yCmd,
            BooleanSupplier fieldOriented) {

        this.subSysDrive = subSysDrive;
        this.subSysVisionportal =subSysVisionportal;
        this.xCmd = xCmd;
        this.yCmd = yCmd;
        this.fieldOriented = fieldOriented;
        addRequirements(subSysDrive, subSysVisionportal);
    }
    @Override
    public void initialize() {
        // Create new PIDF controller with values in SubSys_Drive_Constants
        rotPid = new PIDFController(AngularPIDF.kP, AngularPIDF.kI, AngularPIDF.kD, AngularPIDF.kF);
        rotPid.setTolerance(1); // Degrees
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {
        // Get the best recognition from the pipeline
        Recognition bestRecognition = subSysVisionportal.subSysTensorflow.getBestRecognition(LARGEST);
        double rotCmd = 0;
        double lastNonZeroRotCmd = 0;
        // Calculate the rotation value
        if (bestRecognition != null) {
            rotCmd = rotPid.calculate(0, bestRecognition.estimateAngleToObject(AngleUnit.DEGREES));
            lastNonZeroRotCmd = rotCmd;
        } else {
            rotCmd = lastNonZeroRotCmd;
        }
        subSysDrive.drive(
                xCmd.getAsDouble(),
                yCmd.getAsDouble(),
                rotCmd*0.4,
                fieldOriented.getAsBoolean()
        );

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

