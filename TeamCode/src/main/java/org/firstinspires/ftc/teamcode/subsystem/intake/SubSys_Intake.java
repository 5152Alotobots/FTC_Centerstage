package org.firstinspires.ftc.teamcode.subsystem.intake;

import static org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake_Constants.Specs.INTAKE_TICKS_PER_DEGREE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake_Constants.MotorIds;
public class SubSys_Intake extends SubsystemBase
{
    private final Motor intakeMotor;

    public SubSys_Intake(HardwareMap hwMap) {
        intakeMotor = new Motor(hwMap, MotorIds.INTAKE);

        // Reset the values when init
        resetIntakePosition();
    }

    /**
     * Rotates the intake at the specified power
     * @param power Percent (%) power to rotate at
     * */
    public void intake(double power) {
        intakeMotor.set(power);
    }

    /**
     * Takes motor rotation ticks and convert to degrees
     * @param ticks Motor ticks
     * @return Degrees
     * */
    public double intakeTicksToDegrees(int ticks) {
        return (ticks / INTAKE_TICKS_PER_DEGREE);
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
     * Gets the current position of the intake rotation in degrees
     * @return Degrees
     * */
    public double getIntakeDegrees() {
        return intakeTicksToDegrees(intakeMotor.getCurrentPosition());
    }

    /**
     * Gets the current position of the intake rotation in ticks
     * @return Ticks
     * */
    public double getIntakeTicks() {
        return intakeMotor.getCurrentPosition();
    }

    /**
     * Resets the position of the intake motor to ZERO
     * */
    public void resetIntakePosition() {
        intakeMotor.resetEncoder();
    }


    @Override
    public void periodic() {
    }

}