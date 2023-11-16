package org.firstinspires.ftc.teamcode.commandGroups.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand;
import org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake;
import org.firstinspires.ftc.teamcode.subsystem.markerdetection.MARKER_POSITION;
import org.firstinspires.ftc.teamcode.subsystem.markerdetection.SubSys_MarkerDetection;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.SubSys_RoadRunner;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.commands.Cmd_SubSys_RoadRunner_FollowTrajectory;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.commands.Cmd_SubSys_RoadRunner_SetStartPose;
import org.firstinspires.ftc.teamcode.trajectories.detectplacepark.SpikeMark;
import org.firstinspires.ftc.teamcode.trajectories.detectplacepark.spikeposition.SpikeMarkCenterPosition;
import org.firstinspires.ftc.teamcode.trajectories.detectplacepark.spikeposition.SpikeMarkLeftPosition;
import org.firstinspires.ftc.teamcode.trajectories.detectplacepark.spikeposition.SpikeMarkRightPosition;

import java.util.HashMap;

public class CmdGrpSequential_DetectPlacePark extends SequentialCommandGroup
{
    public CmdGrpSequential_DetectPlacePark(
            SubSys_Drive subSysDrive,
            SubSys_RoadRunner subSysRoadRunner,
            SubSys_Gyro subSysGyro,
            SubSys_Arm subSysArm,
            SubSys_Intake subSysIntake,
            SubSys_Hand subSysHand,
            SubSys_MarkerDetection subSysMarkerDetection,
            Telemetry telemetry) {


        Cmd_SubSys_RoadRunner_FollowTrajectory rightPos = new Cmd_SubSys_RoadRunner_FollowTrajectory(
            subSysDrive,
                subSysRoadRunner,
                subSysGyro,
                telemetry,
                SpikeMarkRightPosition.GetTrajectory(subSysRoadRunner)
        );

        Cmd_SubSys_RoadRunner_FollowTrajectory centerPos = new Cmd_SubSys_RoadRunner_FollowTrajectory(
                subSysDrive,
                subSysRoadRunner,
                subSysGyro,
                telemetry,
                SpikeMarkCenterPosition.GetTrajectory(subSysRoadRunner)
        );

        Cmd_SubSys_RoadRunner_FollowTrajectory leftPos = new Cmd_SubSys_RoadRunner_FollowTrajectory(
                subSysDrive,
                subSysRoadRunner,
                subSysGyro,
                telemetry,
                SpikeMarkLeftPosition.GetTrajectory(subSysRoadRunner)
        );
        addCommands(
                new Cmd_SubSys_RoadRunner_SetStartPose(subSysRoadRunner, SpikeMark.GetTrajectory(subSysRoadRunner).start()),
                new InstantCommand(subSysMarkerDetection::initializeSensors),
                new Cmd_SubSys_RoadRunner_FollowTrajectory(subSysDrive, subSysRoadRunner, subSysGyro, telemetry, SpikeMark.GetTrajectory(subSysRoadRunner)),
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(MARKER_POSITION.LEFT, leftPos);
                            put(MARKER_POSITION.RIGHT, rightPos);
                            put(MARKER_POSITION.CENTER, centerPos);
                        }},
                        // Selector
                        subSysMarkerDetection::getMarkerPosition
                ),
                new InstantCommand(subSysMarkerDetection::dropPixel),
                new InstantCommand(subSysMarkerDetection::closeSensors)
        );
    }
}
