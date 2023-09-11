package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_JoystickDefault;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;

@TeleOp(name="Teleop", group = "teleop")
public class Teleop extends LinearOpMode
{
    public final SubSys_Gyro subSysGyro = new SubSys_Gyro(hardwareMap);
    public final SubSys_Drive subSysDrive = new SubSys_Drive(subSysGyro, hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        subSysDrive.setDefaultCommand(
                new Cmd_SubSys_Drive_JoystickDefault(
                        subSysDrive,
                        () -> gamepad1.left_stick_x,
                        () -> gamepad1.left_stick_y,
                        () -> gamepad1.right_stick_x,
                        () -> true
                ));
    }
}
