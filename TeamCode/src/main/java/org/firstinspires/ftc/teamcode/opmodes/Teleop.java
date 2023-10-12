package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.arm.commands.Cmd_SubSys_Arm_JoystickDefault;
import org.firstinspires.ftc.teamcode.subsystem.arm.commands.Cmd_SubSys_Arm_RotateToDegree;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_JoystickDefault;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_JoystickRotateAroundRecognition;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_PIDFTuning;
import org.firstinspires.ftc.teamcode.subsystem.driverstation.SubSys_DriverStation;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.SubSys_Visionportal;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag.SubSys_Apriltag;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow;

@TeleOp(name="Teleop", group = "teleop")
public class Teleop extends CommandOpMode
{

    private SubSys_Gyro subSysGyro;
    private SubSys_Drive subSysDrive;
    private SubSys_Tensorflow subSysTensorflow;
    private SubSys_Apriltag subSysApriltag;
    private SubSys_Visionportal subSysVisionportal;
    private SubSys_Arm subSysArm;
    private SubSys_DriverStation subSysDriverStation;

    @Override
    public void initialize() {
        // MOVE TO SUBSYS_DRIVERSTAION
        GamepadEx driverController = new GamepadEx(gamepad1);
        GamepadEx coDriverController = new GamepadEx(gamepad2);


        subSysDriverStation = new SubSys_DriverStation(gamepad1, gamepad2);
        subSysGyro = new SubSys_Gyro(hardwareMap);
        subSysDrive = new SubSys_Drive(subSysGyro, hardwareMap);
        subSysArm = new SubSys_Arm(hardwareMap);
        // subSysApriltag = new SubSys_Apriltag();
        // subSysTensorflow = new SubSys_Tensorflow();
        // subSysVisionportal = new SubSys_Visionportal(subSysTensorflow, subSysApriltag, hardwareMap);
        register(subSysDriverStation, subSysGyro, subSysDrive, subSysArm);

        subSysDrive.setDefaultCommand(
                new Cmd_SubSys_Drive_JoystickDefault(
                        subSysDrive,
                        telemetry,
                        driverController::getLeftX,
                        driverController::getLeftY,
                        driverController::getRightX,
                        () -> true
                ));
        subSysArm.setDefaultCommand(
                new Cmd_SubSys_Arm_JoystickDefault(
                        subSysArm,
                        telemetry,
                        coDriverController::getRightY,
                        coDriverController::getLeftY
                )
        );


    coDriverController.getGamepadButton(GamepadKeys.Button.B).whenHeld(
            new Cmd_SubSys_Arm_RotateToDegree(
                    subSysArm,
                    telemetry,
                    () -> 35
            ).withTimeout(2000)
    );


        /*
        runPIDF.whenHeld(new Cmd_SubSys_Drive_JoystickRotateAroundRecognition(
                subSysDrive,
                subSysVisionportal,
                () -> m_driverController.getLeftX(),
                () -> m_driverController.getLeftY(),
                () -> false
        ));
        */
    }

}
