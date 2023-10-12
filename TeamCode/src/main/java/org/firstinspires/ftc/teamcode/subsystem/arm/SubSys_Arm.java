package org.firstinspires.ftc.teamcode.subsystem.arm;

import static org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm_Constants.Specs.EXTENSION_TICKS_PER_METER;
import static org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm_Constants.Specs.ROTATION_TICKS_PER_DEGREE;

import android.text.method.Touch;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm_Constants.MotorIds;
public class SubSys_Arm extends SubsystemBase
{
    private HardwareMap hwMap;
    private Motor rotateMotor;
    private Motor extendMotor;
    private TouchSensor downTouch;
    private TouchSensor upTouch;
    private TouchSensor inTouch;

    public SubSys_Arm(HardwareMap hwMap) {
        this.hwMap = hwMap;
        rotateMotor = new Motor(hwMap, MotorIds.ROTATE);
        extendMotor = new Motor(hwMap, MotorIds.EXTEND);
        downTouch = hwMap.get(TouchSensor.class, "downtouch");
        upTouch = hwMap.get(TouchSensor.class, "uptouch");
        inTouch = hwMap.get(TouchSensor.class, "inTouch");
    }

    /**
     * Rotates the arm at the specified power
     * @param power Percent (%) power to rotate at
     * */
    public void rotate(double power) {
        if (upTouch.isPressed() && power > 0) {
            rotateMotor.set(0); // Force NO OUTPUT DOWN
        } else if (downTouch.isPressed() && power < 0){
            rotateMotor.set(0); // Force NO OUTPUT UP
        } else {
            rotateMotor.set(power); // Run output
        }
    }

    /**
     * Extends the arm at the specified power
     * @param power Percent (%) power to extend at
     * */
    public void extend(double power) {
        if (inTouch.isPressed() && power < 0) {
            extendMotor.set(0); // Force NO OUTPUT IN
        } else {
            extendMotor.set(power); // Run output
        }
    }

    /**
     * Takes motor rotation ticks and convert to degrees
     * @param ticks Motor ticks
     * @return Degrees
     * */
    public double rotationTicksToDegrees(int ticks) {
        return (ticks / ROTATION_TICKS_PER_DEGREE);
    }

    /**
     * Takes motor extension ticks and convert to meters
     * @param ticks Motor ticks
     * @return Meters
     * */
    public double extendTicksToMeters(int ticks) {
        return (ticks / EXTENSION_TICKS_PER_METER);
    }

    /**
     * Is the arm currently touching the BACK boundary
     * @return true if touching
     * */
    public boolean rotateAtBackLimit() {
        return upTouch.isPressed();
    }

    /**
     * Is the arm currently touching the FRONT boundary
     * @return true if touching
     * */
    public boolean rotateAtFrontLimit() {
        return downTouch.isPressed();
    }

    /**
     * Is the arm currently all the way retracted
     * @return true if touching
     * */
    public boolean armRetracted() {
        return inTouch.isPressed();
    }

    /**
     * Gets the current position of the arm rotation in degrees
     * @return Degrees
     * */
    public double getRotationDegrees() {
        return rotationTicksToDegrees(rotateMotor.getCurrentPosition());
    }

    /**
     * Gets the current position of the arm extension in meters
     * @return Meters
     * */
    public double getExtensionMeters() {
        return extendTicksToMeters(extendMotor.getCurrentPosition());
    }

    /**
     * Resets the position of the rotation motor to ZERO
     * */
    public void resetRotationPosition() {
        rotateMotor.resetEncoder();
    }

    /**
     * Resets the position of the extension motor to ZERO
     * */
    public void resetExtensionPosition() {
        extendMotor.resetEncoder();
    }



    @Override
    public void periodic() {
    }

}
