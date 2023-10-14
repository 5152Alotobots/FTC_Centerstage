package org.firstinspires.ftc.teamcode.subsystem.driverstation;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class SubSys_DriverStation extends SubsystemBase
{

    private GamepadEx driverController;
    private GamepadEx coDriverController;
    /* Buttons */
    // Driver
    public final GamepadButton resetGyroButton;

    // CoDriver
    public final GamepadButton armIntakePositionButton;
    public final GamepadButton armMidPositionButton;
    public final GamepadButton armHighPositionButton;
    public final GamepadButton wristButton;


    public SubSys_DriverStation(Gamepad gamepad1, Gamepad gamepad2) {

        /* Driver controller */
        GamepadEx driverController = new GamepadEx(gamepad1);
        this.driverController = driverController;
        resetGyroButton = new GamepadButton(driverController, GamepadKeys.Button.A);

        /* CoDriver controller */
        GamepadEx coDriverController = new GamepadEx(gamepad2);
        this.coDriverController = coDriverController;
        armIntakePositionButton = new GamepadButton(coDriverController, GamepadKeys.Button.A);
        armMidPositionButton = new GamepadButton(coDriverController, GamepadKeys.Button.B);
        armHighPositionButton = new GamepadButton(coDriverController, GamepadKeys.Button.Y);
        wristButton = new GamepadButton(coDriverController, GamepadKeys.Button.X);
    }

    /* Driver controller */
    public double getDriverLeftX() {
        return driverController.getLeftX();
    }
    public double getDriverLeftY() {
        return driverController.getLeftY();
    }
    public double getDriverRightX() {
        return driverController.getRightX();
    }
    public double getDriverRightY() {
        return driverController.getRightY();
    }

    /* CoDriver controller */
    public double getCoDriverLeftX() {
        return coDriverController.getLeftX();
    }
    public double getCoDriverLeftY() {
        return coDriverController.getLeftY();
    }
    public double getCoDriverRightX() {
        return coDriverController.getRightX();
    }
    public double getCoDriverRightY() {
        return coDriverController.getRightY();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


}
