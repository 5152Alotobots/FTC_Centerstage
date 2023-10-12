package org.firstinspires.ftc.teamcode.subsystem.arm;

import static org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm_Constants.Specs.EXTENSION_TICKS_PER_CENTIMETER;
import static org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm_Constants.Specs.ROTATION_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm_Constants.Tuning.INTAKE_EXTEND_ROTATE_SOFT_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm_Constants.Tuning.MAX_EXT_AT_INTAKE;
import static org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm_Constants.Tuning.OUTER_EXTEND_LIMIT;

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
    private TouchSensor backTouch;
    private TouchSensor frontTouch;
    private TouchSensor inTouch;

    public SubSys_Arm(HardwareMap hwMap) {
        this.hwMap = hwMap;
        rotateMotor = new Motor(hwMap, MotorIds.ROTATE);
        extendMotor = new Motor(hwMap, MotorIds.EXTEND);
        backTouch = hwMap.get(TouchSensor.class, "downtouch");
        frontTouch = hwMap.get(TouchSensor.class, "uptouch");
        inTouch = hwMap.get(TouchSensor.class, "exttouch");

        // Reset the values when init
        resetExtensionPosition();
        resetRotationPosition();
    }

    /**
     * Rotates the arm at the specified power
     * @param power Percent (%) power to rotate at
     * */
    public void rotate(double power) {
        boolean frontLimit = frontTouch.isPressed() && power > 0;
        boolean backLimit = backTouch.isPressed() && power < 0;
        boolean intakeSoftLimit = getExtensionCentimeters() > MAX_EXT_AT_INTAKE && getRotationDegrees() > INTAKE_EXTEND_ROTATE_SOFT_LIMIT;
        if (frontLimit || backLimit || intakeSoftLimit) {
            rotateMotor.set(0); // FORCE NO OUTPUT
        } else {
            rotateMotor.set(power); // Run output
        }
    }

    /**
     * Extends the arm at the specified power
     * @param power Percent (%) power to extend at
     * */

    public void extend(double power) {
        boolean inLimit = inTouch.isPressed() && power < 0;
        boolean outLimit = OUTER_EXTEND_LIMIT < getExtensionCentimeters() && power > 0;
        if (inLimit || outLimit) {
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
     * Takes motor extension ticks and convert to centimeters
     * @param ticks Motor ticks
     * @return Centimeters
     * */
    public double extendTicksToCentimeters(int ticks) {
        return (ticks / EXTENSION_TICKS_PER_CENTIMETER);
    }

    /**
     * Is the arm currently touching the BACK boundary
     * @return true if touching
     * */
    public boolean rotateAtBackLimit() {
        return backTouch.isPressed();
    }

    /**
     * Is the arm currently touching the FRONT boundary
     * @return true if touching
     * */
    public boolean rotateAtFrontLimit() {
        return frontTouch.isPressed();
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
     * Gets the current position of the arm extension in centimeters
     * @return Centimeters
     * */
    public double getExtensionCentimeters() {
        return extendTicksToCentimeters(extendMotor.getCurrentPosition());
    }

    /**
     * Gets the current position of the arm extension in ticks
     * @return Ticks
     * */
    public double getExtensionTicks() {
        return extendMotor.getCurrentPosition();
    }

    /**
     * Gets the current position of the arm rotation in ticks
     * @return Ticks
     * */
    public double getRotationTicks() {
        return rotateMotor.getCurrentPosition();
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
        if (frontTouch.isPressed() && getRotationDegrees() != 0) resetRotationPosition();
        if (inTouch.isPressed() && getExtensionCentimeters() != 0) resetExtensionPosition();
    }

}
