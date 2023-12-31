package org.firstinspires.ftc.teamcode.subsystem.launcher;

import static org.firstinspires.ftc.teamcode.subsystem.launcher.SubSys_Launcher_Constants.MotorIds.LAUNCHER;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SubSys_Launcher extends SubsystemBase
{
    private Servo launcherServo;
    public SubSys_Launcher(HardwareMap hwMap) {

        launcherServo = hwMap.get(Servo.class, LAUNCHER);
    }
    /** Rotates the servo to a specified position
     * @param position -1.0-1.0
     * */
    public void rotateToPosition(double position) {
        launcherServo.setPosition(position);
    }
}
