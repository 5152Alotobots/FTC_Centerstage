package org.firstinspires.ftc.teamcode.subsystem.markerdetection;

import static org.firstinspires.ftc.teamcode.subsystem.markerdetection.SubSys_MarkerDetection_Constants.MotorIds.DROPPER;
import static org.firstinspires.ftc.teamcode.subsystem.markerdetection.SubSys_MarkerDetection_Constants.SensorIds.LEFT_SENSOR;
import static org.firstinspires.ftc.teamcode.subsystem.markerdetection.SubSys_MarkerDetection_Constants.SensorIds.RIGHT_SENSOR;
import static org.firstinspires.ftc.teamcode.subsystem.markerdetection.SubSys_MarkerDetection_Constants.Tuning.MINIMUM_DETECTED_RED;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

public class SubSys_MarkerDetection extends SubsystemBase
{
    private RevColorSensorV3 leftColor;
    private RevColorSensorV3 rightColor;
    private Servo pixelDrop;
    
    public SubSys_MarkerDetection(HardwareMap hwMap) {
        this.leftColor = hwMap.get(RevColorSensorV3.class, LEFT_SENSOR);
        this.rightColor = hwMap.get(RevColorSensorV3.class, RIGHT_SENSOR);
        this.pixelDrop = hwMap.get(Servo.class, DROPPER);
    }

    /**
     * Initializes the sensors for use
     * */
    public void initializeSensors() {
        leftColor.initialize();
        rightColor.initialize();
    }

    /**
     * Closes the sensors after finishing
     * */
    public void closeSensors() {
        leftColor.close();
        rightColor.close();
    }

    public void dropPixel() {
        pixelDrop.setPosition(-1);
    }

    public void holdPixel() {
        pixelDrop.setPosition(1);
    }

    @Override
    public void periodic() {
        // Runs once every scheduler run
    }

    public MARKER_POSITION getMarkerPosition() {
        NormalizedRGBA leftDetected = leftColor.getNormalizedColors();
        NormalizedRGBA rightDetected = rightColor.getNormalizedColors();
        if (leftDetected.red > MINIMUM_DETECTED_RED) {
            return MARKER_POSITION.LEFT;
        } else if (rightDetected.red > MINIMUM_DETECTED_RED) {
            return MARKER_POSITION.RIGHT;
        } else {
            return MARKER_POSITION.CENTER;
        }
    }

    /**
     * Gets the red value of both color sensors
     * @return float[left, right]
     * */
    public float[] getRed() {
        NormalizedRGBA leftDetected = leftColor.getNormalizedColors();
        NormalizedRGBA rightDetected = rightColor.getNormalizedColors();
        return new float[]{leftDetected.red, rightDetected.red};
    }

    /**
     * Gets the blue value of both color sensors
     * @return float[left, right]
     * */
    public float[] getBlue() {
        NormalizedRGBA leftDetected = leftColor.getNormalizedColors();
        NormalizedRGBA rightDetected = rightColor.getNormalizedColors();
        return new float[]{leftDetected.blue, rightDetected.blue};
    }

    /**
     * Gets the green value of both color sensors
     * @return float[left, right]
     * */
    public float[] getGreen() {
        NormalizedRGBA leftDetected = leftColor.getNormalizedColors();
        NormalizedRGBA rightDetected = rightColor.getNormalizedColors();
        return new float[]{leftDetected.green, rightDetected.green};
    }

    /**
     * Gets the RGB values of both color sensors
     * @return float[left[r, g, b], right[r, g, b]]
     * */
    public float[][] getRGB() {
        NormalizedRGBA leftDetected = leftColor.getNormalizedColors();
        NormalizedRGBA rightDetected = rightColor.getNormalizedColors();
        return new float[][]{
                {leftDetected.red, leftDetected.green, leftDetected.blue},
                {rightDetected.red, rightDetected.blue, rightDetected.green}
        };
    }

    /**
     * Checks to see if the current value of the LEFT sensor is within the limits
     * @param maxRed upper red limit
     * @param minRed lower red limit
     * */
    public boolean redWithinLeft(float minRed, float maxRed) {
        NormalizedRGBA leftDetected = leftColor.getNormalizedColors();
        return leftDetected.red > minRed && leftDetected.red < maxRed;
    }

    /**
     * Checks to see if the current value of the RIGHT sensor is within the limits
     * @param maxRed upper red limit
     * @param minRed lower red limit
     * */
    public boolean redWithinRight(float minRed, float maxRed) {
        NormalizedRGBA rightDetected = rightColor.getNormalizedColors();
        return rightDetected.red > minRed && rightDetected.red < maxRed;
    }


}


