package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystem.visionportal.SubSys_Visionportal;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag.SubSys_Apriltag;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow;

public class TfTest extends CommandOpMode
{
    private SubSys_Tensorflow subSysTensorflow;
    private SubSys_Apriltag subSysApriltag;
    private SubSys_Visionportal subSysVisionportal;
    @Override
    public void initialize() {
        subSysApriltag = new SubSys_Apriltag();
        subSysTensorflow = new SubSys_Tensorflow();
        subSysVisionportal = new SubSys_Visionportal(subSysTensorflow, subSysApriltag, hardwareMap);

        register(subSysApriltag, subSysTensorflow, subSysVisionportal);
    }
}
