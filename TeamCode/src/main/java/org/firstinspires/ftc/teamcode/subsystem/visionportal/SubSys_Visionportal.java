package org.firstinspires.ftc.teamcode.subsystem.visionportal;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag.SubSys_Apriltag;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class SubSys_Visionportal extends SubsystemBase
{
    VisionPortal portal;
    private SubSys_Apriltag subSysApriltag;
    private SubSys_Tensorflow subSysTensorflow;
    private TfodProcessor tensorflowProcessor;
    private AprilTagProcessor aprilTagProcessor;

    public SubSys_Visionportal(SubSys_Tensorflow subSysTensorflow, SubSys_Apriltag subSysApriltag, HardwareMap hardwareMap) {
        this.tensorflowProcessor = subSysTensorflow.getProcessor();
        this.aprilTagProcessor = subSysApriltag.getProcessor();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "front"))
                .addProcessor(aprilTagProcessor)
                .addProcessor(tensorflowProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .build();

        // Assign global vars
        this.subSysApriltag = subSysApriltag;
        this.subSysTensorflow = subSysTensorflow;
    }

    /**
     * disables/enables tensorflow pipeline. Enabled by default
     * @param enable true if enabled, false to disable
     * */
    public void enableTensorflow(boolean enable) {
        portal.setProcessorEnabled(tensorflowProcessor, enable);
    }

    /**
     * disables/enables apriltag pipeline. Enabled by default
     * @param enable true if enabled, false to disable
     * */
    public void enableApriltag(boolean enable) {
        portal.setProcessorEnabled(aprilTagProcessor, enable);
    }

    /**
     * Is the tensorflow pipeline enabled?
     * @return boolean, true if enabled, false if not
     * */
    public boolean tensorflowEnabled() {
        return portal.getProcessorEnabled(tensorflowProcessor);
    }

    /**
     * Is the apriltag pipeline enabled?
     * @return boolean, true if enabled, false if not
     * */
    public boolean apriltagEnabled() {
        return portal.getProcessorEnabled(aprilTagProcessor);
    }

    public boolean deviceInReadyState() {
        return portal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY;
    }

    /**
     * WARNING: ONLY USE THIS IF YOU KNOW WHAT YOU ARE DOING
     * Returns the FULL portal object. NO ERROR HANDLING!
     * */
    public VisionPortal getPortal() {
        return portal;
    }

    @Override
    public void periodic() {
        // Runs once every scheduler run
    }


}
