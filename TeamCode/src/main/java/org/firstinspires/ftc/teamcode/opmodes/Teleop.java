package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandGroups.teleop.CmdGrpParallel_ArmPosIntake;
import org.firstinspires.ftc.teamcode.commandGroups.teleop.CmdGrpParallel_ArmPosLow;
import org.firstinspires.ftc.teamcode.commandGroups.teleop.CmdGrpParallel_ArmPosMid;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.arm.commands.Cmd_SubSys_Arm_JoystickDefault;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_JoystickDefault;
import org.firstinspires.ftc.teamcode.subsystem.driverstation.SubSys_DriverStation;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand;
import org.firstinspires.ftc.teamcode.subsystem.hand.commands.Cmd_SubSys_Hand_JoystickDefault;
import org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake;
import org.firstinspires.ftc.teamcode.subsystem.intake.commands.Cmd_SubSys_Intake_JoystickDefault;
import org.firstinspires.ftc.teamcode.subsystem.launcher.SubSys_Launcher;
import org.firstinspires.ftc.teamcode.subsystem.markerdetection.SubSys_MarkerDetection;
import org.firstinspires.ftc.teamcode.subsystem.markerdetection.commands.Cmd_SubSys_MarkerDetection_Test;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.SubSys_Visionportal;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag.SubSys_Apriltag;
import org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow;

@TeleOp(name="Teleop", group = "teleop")
public class Teleop extends CommandOpMode
{
    private SubSys_MarkerDetection subSysMarkerDetection;
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
        SubSys_MarkerDetection subSysMarkerDetection = new SubSys_MarkerDetection(hardwareMap);

        // subSysApriltag = new SubSys_Apriltag();
        // subSysTensorflow = new SubSys_Tensorflow();
        // subSysVisionportal = new SubSys_Visionportal(subSysTensorflow, subSysApriltag, hardwareMap);
        register(subSysDriverStation, subSysGyro, subSysDrive, subSysArm, subSysHand, subSysIntake, subSysLauncher, subSysMarkerDetection);

        subSysMarkerDetection.setDefaultCommand(
                new Cmd_SubSys_MarkerDetection_Test(
                        subSysMarkerDetection,
                        telemetry
                )
        );

        /* Default commands */
        subSysDrive.setDefaultCommand(
                new Cmd_SubSys_Drive_JoystickDefault(
                        subSysDrive,
                        telemetry,
                        subSysDriverStation::getDriverLeftX,
                        subSysDriverStation::getDriverLeftY,
                        subSysDriverStation::getDriverRightX,
                        () -> false,
                        subSysDriverStation::isDriverLeftTriggerDown,
                        subSysDriverStation::isDriverRightTriggerDown
                ));
        subSysArm.setDefaultCommand(
                new Cmd_SubSys_Arm_JoystickDefault(
                        subSysArm,
                        telemetry,
                        subSysDriverStation::getCoDriverRightY,
                        subSysDriverStation::getCoDriverLeftY
                )
        );
        subSysHand.setDefaultCommand(
                new Cmd_SubSys_Hand_JoystickDefault(
                        subSysHand,
                        subSysArm,
                        telemetry,
                        () -> subSysDriverStation.isCoDriverDpadUp()
                                -subSysDriverStation.isCoDriverDpadDown()
                )
        );
        subSysIntake.setDefaultCommand(
                new Cmd_SubSys_Intake_JoystickDefault(
                        subSysIntake,
                        subSysArm,
                        telemetry,
                        () -> subSysDriverStation.getCoDriverRightTrigger()
                                -subSysDriverStation.getCoDriverLeftTrigger()
                )
        );

        /* Arm positions */
        subSysDriverStation.armIntakePositionButton.whenPressed(
                new CmdGrpParallel_ArmPosIntake(
                        subSysArm,
                        subSysHand,
                        telemetry
                ).withTimeout(3000)
        );
        subSysDriverStation.armLowPositionButton.whenPressed(
                new CmdGrpParallel_ArmPosLow(
                        subSysArm,
                        subSysHand,
                        telemetry
                ).withTimeout(3000)
        );
        subSysDriverStation.armMidPositionButton.whenPressed(
                new CmdGrpParallel_ArmPosMid(
                        subSysArm,
                        subSysHand,
                        telemetry
                ).withTimeout(3000)
        );
        /*subSysDriverStation.armHighPositionButton.whenPressed(
                new Cmd_SubSys_Arm_RotateAndExtend(
                        subSysArm,
                        telemetry,
                        () -> 65,
                        () -> 70
                ).withTimeout(3000)
        );
         */

        /* Intake default command - Use controller if input, else auto intake when at 0deg */

        /* Hand Commands */
        subSysDriverStation.toggleLeftDropButton.whenPressed(
                new InstantCommand(
                        subSysHand::toggleLeftBack
                )
        );
        subSysDriverStation.toggleRightDropButton.whenPressed(
                new InstantCommand(
                        subSysHand::toggleRightBack
                )
        );
        subSysDriverStation.toggleGatesButton.whenPressed(
                new InstantCommand(
                        () -> {
                            subSysHand.toggleLeftFront();
                            subSysHand.toggleRightFront();
                        }
                )
        );

        /* Misc */
        subSysDriverStation.resetGyroButton.whenPressed(
                new InstantCommand(subSysGyro::resetYaw)
        );
        subSysDriverStation.armLauncherButton.whenPressed(
                new InstantCommand(() -> subSysLauncher.rotateToPosition(1))
        );
        subSysDriverStation.fireLauncherButton.whenPressed(
                new InstantCommand(() -> subSysLauncher.rotateToPosition(-1.0))
        );

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
