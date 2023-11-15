package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_JoystickRotateAroundRecognition;
import org.firstinspires.ftc.teamcode.subsystem.driverstation.SubSys_DriverStation;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand;
import org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake;
import org.firstinspires.ftc.teamcode.subsystem.launcher.SubSys_Launcher;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.SubSys_Visionportal;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag.SubSys_Apriltag;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow;

@TeleOp(name="RotateAroundRecognition", group = "experiment")
public class RotateAroundRecognition extends CommandOpMode
{
    
    private SubSys_Tensorflow subSysTensorflow;
    private SubSys_Apriltag subSysApriltag;
    private SubSys_Visionportal subSysVisionportal;

    @Override
    public void initialize() {

        SubSys_DriverStation subSysDriverStation = new SubSys_DriverStation(gamepad1, gamepad2);
        SubSys_Gyro subSysGyro = new SubSys_Gyro(hardwareMap);
        SubSys_Drive subSysDrive = new SubSys_Drive(subSysGyro, hardwareMap);
        SubSys_Arm subSysArm = new SubSys_Arm(hardwareMap);
        SubSys_Hand subSysHand = new SubSys_Hand(subSysArm, hardwareMap);
        SubSys_Intake subSysIntake = new SubSys_Intake(hardwareMap);
        SubSys_Launcher subSysLauncher = new SubSys_Launcher(hardwareMap);

        subSysApriltag = new SubSys_Apriltag();
        subSysTensorflow = new SubSys_Tensorflow();
        subSysVisionportal = new SubSys_Visionportal(subSysTensorflow, subSysApriltag, hardwareMap);
        register(subSysDriverStation, subSysGyro, subSysDrive, subSysArm, subSysHand, subSysIntake, subSysLauncher, subSysApriltag, subSysTensorflow, subSysVisionportal);

        /* Default commands */
        subSysDrive.setDefaultCommand(
                new Cmd_SubSys_Drive_JoystickRotateAroundRecognition(
                        subSysDrive,
                        subSysVisionportal,
                        telemetry,
                        subSysDriverStation::getDriverLeftX,
                        subSysDriverStation::getDriverLeftY,
                        () -> false
                ));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler, update telemetry
        while (!isStopRequested() && opModeIsActive()) {
            run();
            telemetry.update();
        }
        reset();
    }

}
