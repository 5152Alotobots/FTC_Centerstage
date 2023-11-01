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
    public final GamepadButton armLauncherButton;
    public final GamepadButton fireLauncherButton;

    // CoDriver
    public final GamepadButton armIntakePositionButton;
    public final GamepadButton armMidPositionButton;
    public final GamepadButton armHighPositionButton;
    public final GamepadButton toggleLeftDropButton;
    public final GamepadButton toggleRightDropButton;
    public final GamepadButton toggleGatesButton;

    public final GamepadButton testButton;


    public SubSys_DriverStation(Gamepad gamepad1, Gamepad gamepad2) {

        /* Driver controller */
        GamepadEx driverController = new GamepadEx(gamepad1);
        this.driverController = driverController;
        resetGyroButton = new GamepadButton(driverController, GamepadKeys.Button.A);
        armLauncherButton = new GamepadButton(driverController, GamepadKeys.Button.DPAD_DOWN);
        fireLauncherButton = new GamepadButton(driverController, GamepadKeys.Button.DPAD_UP);

        /* CoDriver controller */
        GamepadEx coDriverController = new GamepadEx(gamepad2);
        this.coDriverController = coDriverController;
        armIntakePositionButton = new GamepadButton(coDriverController, GamepadKeys.Button.A);
        armMidPositionButton = new GamepadButton(coDriverController, GamepadKeys.Button.B);
        toggleGatesButton = new GamepadButton(coDriverController, GamepadKeys.Button.X);
        armHighPositionButton = new GamepadButton(coDriverController, GamepadKeys.Button.Y);
        toggleLeftDropButton = new GamepadButton(coDriverController, GamepadKeys.Button.LEFT_BUMPER);
        toggleRightDropButton = new GamepadButton(coDriverController, GamepadKeys.Button.RIGHT_BUMPER);
        testButton = new GamepadButton(coDriverController, GamepadKeys.Button.X);
    }

    /* Driver controller */
    public double getDriverLeftX() {
        return driverController.getLeftX();
    }
    public double getDriverLeftY() {
        return driverController.getLeftY();
    }
    public double getDriverLeftTrigger() {
        return driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    }
    public double getDriverRightX() {
        return driverController.getRightX();
    }
    public double getDriverRightY() {
        return driverController.getRightY();
    }
    public double getDriverRightTrigger() {
        return driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }


    /* CoDriver controller */
    public double getCoDriverLeftX() {
        return coDriverController.getLeftX();
    }
    public double getCoDriverLeftY() {
        return coDriverController.getLeftY();
    }
    public double getCoDriverLeftTrigger() {
        return  coDriverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    }
    public double getCoDriverRightX() {
        return coDriverController.getRightX();
    }
    public double getCoDriverRightY() {
        return coDriverController.getRightY();
    }
    public double getCoDriverRightTrigger() {
        return  coDriverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


}
