package org.firstinspires.ftc.teamcode.subsystem.visionportal.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.github.freva.asciitable.AsciiTable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.SubSys_Visionportal;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.function.BooleanSupplier;

public class Cmd_SubSys_Visionportal_Default extends CommandBase
{
    private SubSys_Visionportal subSysVisionportal;
    private Telemetry telemetry;
    private VisionPortal portal;
    private BooleanSupplier tensorflowEnabled, apriltagEnabled;

    public Cmd_SubSys_Visionportal_Default(
            SubSys_Visionportal subSysVisionportal,
            Telemetry telemetry,
            BooleanSupplier tensorflowEnabled,
            BooleanSupplier apriltagEnabled) {

        this.subSysVisionportal = subSysVisionportal;
        this.telemetry = telemetry;
        this.tensorflowEnabled = tensorflowEnabled;
        this.apriltagEnabled = apriltagEnabled;
        addRequirements(subSysVisionportal);
    }
    @Override
    public void initialize() {
        subSysVisionportal.enableTensorflow(true);
        subSysVisionportal.enableApriltag(true);
        portal = subSysVisionportal.getPortal();
    }

    @Override
    public void execute() {
        // If camera cant be opened, attempt to not kill the program
        if (portal.getCameraState() == VisionPortal.CameraState.ERROR) {
            telemetry.addLine("Failed to open camera stream, vision not available!");
            portal.close();
        } else {
            String state = portal.getCameraState().toString();
            String fps = String.valueOf(portal.getFps());
            String tfEnabled = String.valueOf(tensorflowEnabled.getAsBoolean());
            String atEnabled = String.valueOf(tensorflowEnabled.getAsBoolean());

            String[] headers = {"State", "Fps", "TF Enabled", "AT Enabled"};
            String[][] data = {
                    {state, fps, tfEnabled, atEnabled}
            };
            String table = AsciiTable.getTable(headers, data);
            telemetry.addLine(table);

        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false; // default command so never ends
    }
}
