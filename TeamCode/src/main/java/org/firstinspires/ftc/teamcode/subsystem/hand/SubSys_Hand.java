package org.firstinspires.ftc.teamcode.subsystem.hand;

import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.MotorIds.LEFT_BACK_SERVO;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.MotorIds.LEFT_FRONT_SERVO;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.MotorIds.RIGHT_BACK_SERVO;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.MotorIds.RIGHT_FRONT_SERVO;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.MotorIds.WRIST;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.Specs.ROTATION_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.Tuning.CLOSED_LEFT_BACK_POS;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.Tuning.CLOSED_LEFT_FRONT_POS;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.Tuning.CLOSED_RIGHT_BACK_POS;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.Tuning.CLOSED_RIGHT_FRONT_POS;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.Tuning.MAX_ROTATION_SPEED;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.Tuning.OPEN_LEFT_BACK_POS;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.Tuning.OPEN_LEFT_FRONT_POS;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.Tuning.OPEN_RIGHT_BACK_POS;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.Tuning.OPEN_RIGHT_FRONT_POS;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.Tuning.ROTATION_DOWN_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.Tuning.ROTATION_UP_LIMIT;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;

public class SubSys_Hand extends SubsystemBase
{
    private SubSys_Arm subSysArm;

    private Motor wristMotor;
    private ColorSensor leftColor;
    private ColorSensor rightColor;
    private Servo leftGate;
    private Servo leftDrop;
    private Servo rightGate;
    private Servo rightDrop;

    // Toggles for servos
    private boolean leftFrontOpen;
    private boolean leftBackOpen;
    private boolean rightFrontOpen;
    private boolean rightBackOpen;

    public SubSys_Hand(SubSys_Arm subSysArm, HardwareMap hwMap) {
        this.subSysArm = subSysArm;
       wristMotor = new Motor(hwMap, WRIST);
       leftGate = hwMap.get(Servo.class, LEFT_FRONT_SERVO);
       leftDrop = hwMap.get(Servo.class, LEFT_BACK_SERVO);
       rightGate = hwMap.get(Servo.class, RIGHT_FRONT_SERVO);
       rightDrop = hwMap.get(Servo.class, RIGHT_BACK_SERVO);

       // leftColor = hwMap.get(ColorSensor.class, LEFT_SENSOR);
       // rightColor = hwMap.get(ColorSensor.class, RIGHT_SENSOR);
       wristMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftFrontOpen = true;
        leftBackOpen = false;
        rightFrontOpen = true;
        rightBackOpen = false;
    }

    /**Rotates the wrist at a given power output.
     * @param power The power to rotate the wrist at.
     * */
    public void rotate(double power) {
        boolean handUpLimit = getRotationDegrees() > ROTATION_UP_LIMIT  && power > 0; // Don't rotate up if the rotation is greater than the limit, allow down.
        boolean handDownLimit = getRotationDegrees() < ROTATION_DOWN_LIMIT  && power < 0; // Don't rotate down if the rotation is greater than the limit, allow up.


        if (handDownLimit || handUpLimit) {
            wristMotor.set(0); // Force no output
        } else {
            wristMotor.set(MathUtils.clamp(power, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED));
        }

    }

    /**
     * Takes wrist motor rotation ticks and convert to degrees
     * @param ticks Motor ticks
     * @return Degrees
     * */
    public double rotationTicksToDegrees(int ticks) {
        return (ticks / ROTATION_TICKS_PER_DEGREE);
    }

    /**
     * Gets the current position of the wrist rotation in degrees
     * @return Degrees
     * */
    public double getRotationDegrees() {
        return rotationTicksToDegrees(wristMotor.getCurrentPosition());
    }

    /**
     * Gets the current position of the wrist rotation in degrees
     * @return Degrees
     * */
    public int getRotationTicks() {
        return wristMotor.getCurrentPosition();
    }

    /** Toggle the left back servo (left drop)*/
    public void toggleLeftBack() {
        openLeftBack(!leftBackOpen);
        leftBackOpen = !leftBackOpen;
    }

    /** Toggle the left front servo (left gate)*/
    public void toggleLeftFront() {
        openLeftFront(!leftFrontOpen);
        leftFrontOpen = !leftFrontOpen;
    }

    /** Toggle the right back servo (right drop)*/
    public void toggleRightBack() {
        openRightBack(!rightBackOpen);
        rightBackOpen = !rightBackOpen;
    }

    /** Toggle the right front servo (right gate)*/
    public void toggleRightFront() {
        openRightFront(!rightFrontOpen);
        rightFrontOpen = !rightFrontOpen;
    }

    /**
     * Opens the left back servo (left drop) if set to true, closes if false
     * @param open true: open, false: closed
     * */
    public void openLeftBack(boolean open) {
        if (open) {
            leftDrop.setPosition(OPEN_LEFT_BACK_POS);
        } else {
            leftDrop.setPosition(CLOSED_LEFT_BACK_POS);
        }
    }

    /**
     * Opens the left front servo (left gate) if set to true, closes if false
     * @param open true: open, false: closed
     * */
    public void openLeftFront(boolean open) {
        if (open) {
            leftGate.setPosition(OPEN_LEFT_FRONT_POS);
        } else {
            leftGate.setPosition(CLOSED_LEFT_FRONT_POS);
        }
    }

    /**
     * Opens the right back servo (right drop) if set to true, closes if false
     * @param open true: open, false: closed
     * */
    public void openRightBack(boolean open) {
        if (open) {
            rightDrop.setPosition(OPEN_RIGHT_BACK_POS);
        } else {
            rightDrop.setPosition(CLOSED_RIGHT_BACK_POS);
        }
    }

    /**
     * Opens the right front servo (right gate) if set to true, closes if false
     * @param open true: open, false: closed
     * */
    public void openRightFront(boolean open) {
        if (open) {
            rightGate.setPosition(OPEN_RIGHT_FRONT_POS);
        } else {
            rightGate.setPosition(CLOSED_RIGHT_FRONT_POS);
        }

    }
    /**
     * Resets the encoder in the wrist to 0
     * */
    public void resetRotationPosition() {
        wristMotor.resetEncoder();
    }

    public boolean isLeftOccupied() {
        throw new UnsupportedOperationException(); // Not implemented
    }

    public boolean isRightOccupied() {
        throw new UnsupportedOperationException(); // Not implemented
    }

    @Override
    public void periodic() {
        if (subSysArm.rotateAtFrontLimit()) {
            resetRotationPosition();
        }
    }

}
