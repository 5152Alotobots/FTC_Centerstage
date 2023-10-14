package org.firstinspires.ftc.teamcode.subsystem.hand;

import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.MotorIds.WRIST;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SubSys_Hand extends SubsystemBase
{
    private final Servo wrist;
    // private final DistanceSensor leftSensor;
    // private final DistanceSensor rightSensor;

    public SubSys_Hand(HardwareMap hwMap) {
        wrist = hwMap.get(Servo.class, WRIST);
        // leftSensor = hwMap.get(DistanceSensor.class, "leftsensor");
        // rightSensor = hwMap.get(DistanceSensor.class, "rightsensor");

    }

    /**
     * Rotates the wrist to a given degree
     * @param degree The degree to rotate to
     * */
    public void rotateWristToDegree(double degree) {
        wrist.setPosition(degree);
    }

    /**
     * Is the left box currently occupied
     * @return true if occupied
     * */
    public boolean leftBoxOccupied() {
        return false; //leftSensor.getDistance(DistanceUnit.CM) < OCCUPIED_DISTANCE_CM;
    }


    /**
     * Is the right box currently occupied
     * @return true if occupied
     * */
    public boolean rightBoxOccupied() {
        return false; //rightSensor.getDistance(DistanceUnit.CM) < OCCUPIED_DISTANCE_CM;
    }

    /**
     * Gets the current position of the wrist rotation in degrees
     * @return Degrees
     * */
    public double getWristDegrees() {
        return wrist.getPosition();
    }


    @Override
    public void periodic() {
    }

}
