package org.firstinspires.ftc.teamcode.subsystem.visionportal;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag.SubSys_Apriltag;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow;
import org.firstinspires.ftc.vision.VisionPortal;

public class SubSys_Visionportal extends SubsystemBase
{
    VisionPortal portal;
    public SubSys_Apriltag subSysApriltag;
    public SubSys_Tensorflow subSysTensorflow;

    public SubSys_Visionportal(SubSys_Tensorflow subSysTensorflow, SubSys_Apriltag subSysApriltag, HardwareMap hardwareMap) {
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "front"))
                // .addProcessor(subSysApriltag.getProcessor())
                .addProcessor(subSysTensorflow.getProcessor())
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .build();

        // Assign global vars
        this.subSysApriltag = subSysApriltag;
        this.subSysTensorflow = subSysTensorflow;
    }

    @Override
    public void periodic() {
        // Runs once every scheduler run
    }


}
