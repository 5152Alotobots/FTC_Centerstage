package org.firstinspires.ftc.teamcode.subsystem.driverstation;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class SubSys_DriverStation extends SubsystemBase
{
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public SubSys_DriverStation(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    // Driver controller
    // public GamepadEx driverController = new GamepadEx(gamepad1);

    // CoDriver controller
    // public GamepadEx coDriverController = new GamepadEx(gamepad2);

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
