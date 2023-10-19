package org.firstinspires.ftc.teamcode.subsystem.hand;

import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.MotorIds.WRIST;
import static org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand_Constants.Specs.ROTATION_TICKS_PER_DEGREE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubSys_Hand extends SubsystemBase
{
    private Motor wristMotor;
    public SubSys_Hand(HardwareMap hwMap) {
        wristMotor = new Motor(hwMap, WRIST);
       //wristMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    /**Rotates the wrist at a given power output.
     * @param power The power to rotate the wrist at.
     * */
    public void rotate(double power) {
        wristMotor.set(power);

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
    public double getWristRotationDegrees() {
        return rotationTicksToDegrees(wristMotor.getCurrentPosition());
    }

    /**
     * Gets the current position of the wrist rotation in degrees
     * @return Degrees
     * */
    public int getWristRotationTicks() {
        return wristMotor.getCurrentPosition();
    }

    @Override
    public void periodic() {
    }

}
