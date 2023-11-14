package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.autonomous.red.CmdGrpSequential_RedLeft;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_GlobalPoseStorage;
import org.firstinspires.ftc.teamcode.subsystem.driverstation.SubSys_DriverStation;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand;
import org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake;
import org.firstinspires.ftc.teamcode.subsystem.launcher.SubSys_Launcher;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.SubSys_RoadRunner;

@Autonomous
public class DetectAndPlace extends CommandOpMode
{
    enum ALLIANCE {
        RED, BLUE
    }

    enum SIDE {
        LEFT, RIGHT
    }

    enum START_POS {
        RED_LEFT, RED_RIGHT,
        BLUE_LEFT, BLUE_RIGHT
    }

    private CmdGrpSequential_RedLeft cmdGrpSequentialRedLeft;
    @Override
    public void initialize() {
        SubSys_DriverStation subSysDriverStation = new SubSys_DriverStation(gamepad1, gamepad2);
        SubSys_Gyro subSysGyro = new SubSys_Gyro(hardwareMap);
        SubSys_Drive subSysDrive = new SubSys_Drive(subSysGyro, hardwareMap);
        SubSys_RoadRunner subSysRoadRunner = new SubSys_RoadRunner(hardwareMap);
        SubSys_Arm subSysArm = new SubSys_Arm(hardwareMap);
        SubSys_Hand subSysHand = new SubSys_Hand(subSysArm, hardwareMap);
        SubSys_Intake subSysIntake = new SubSys_Intake(hardwareMap);
        SubSys_Launcher subSysLauncher = new SubSys_Launcher(hardwareMap);
        SubSys_Drive_GlobalPoseStorage.currentPose = new Pose2d(new Translation2d(60, 12), new Rotation2d(Math.toRadians(180)));
        this.cmdGrpSequentialRedLeft =
                new CmdGrpSequential_RedLeft(
                subSysDrive,
                subSysRoadRunner,
                subSysGyro,
                subSysArm,
                subSysIntake,
                subSysHand,
                telemetry);

        // subSysApriltag = new SubSys_Apriltag();
        // subSysTensorflow = new SubSys_Tensorflow();
        // subSysVisionportal = new SubSys_Visionportal(subSysTensorflow, subSysApriltag, hardwareMap);

        register(subSysDriverStation, subSysGyro, subSysDrive, subSysArm, subSysHand, subSysIntake, subSysLauncher);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        cmdGrpSequentialRedLeft.schedule();
        // run the scheduler, update telemetry
        while (!isStopRequested() && opModeIsActive()) {
            run();
            telemetry.update();
        }
        reset();
    }

}
