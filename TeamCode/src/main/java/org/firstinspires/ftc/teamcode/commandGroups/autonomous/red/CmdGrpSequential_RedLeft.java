package org.firstinspires.ftc.teamcode.commandGroups.autonomous.red;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.arm.commands.Cmd_SubSys_Arm_RotateAndExtend;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand;
import org.firstinspires.ftc.teamcode.subsystem.hand.commands.Cmd_SubSys_Hand_RotateWristToPosition;
import org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.SubSys_RoadRunner;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.commands.Cmd_SubSys_Drive_FollowRoadrunnerTraj;
import org.firstinspires.ftc.teamcode.trajectories.testTraj;

public class CmdGrpSequential_RedLeft  extends SequentialCommandGroup
{
    private SubSys_Drive subSysDrive;
    private SubSys_RoadRunner subSysRoadRunner;
    private SubSys_Gyro subSysGyro;
    private SubSys_Arm subSysArm;
    private SubSys_Intake subSysIntake;
    private SubSys_Hand subSysHand;
    private Telemetry telemetry;

    public CmdGrpSequential_RedLeft(
            SubSys_Drive subSysDrive,
            SubSys_RoadRunner subSysRoadRunner,
            SubSys_Gyro subSysGyro,
            SubSys_Arm subSysArm,
            SubSys_Intake subSysIntake,
            SubSys_Hand subSysHand,
            Telemetry telemetry) {

        addCommands(
                new Cmd_SubSys_Drive_FollowRoadrunnerTraj(subSysDrive, subSysRoadRunner, subSysGyro, telemetry, testTraj.testTraj(subSysRoadRunner)),
                new ParallelCommandGroup(
                        new Cmd_SubSys_Arm_RotateAndExtend(
                                subSysArm,
                                telemetry,
                                () -> 29.5,
                                () -> 40
                        ),
                        new Cmd_SubSys_Hand_RotateWristToPosition(
                                subSysHand,
                                telemetry,
                                () -> 50
                        )).withTimeout(3000),
                new InstantCommand(
                    () -> subSysHand.openLeftBack(true)
                )
        );
    }
}
