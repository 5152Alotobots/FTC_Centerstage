package org.firstinspires.ftc.teamcode.trajectories.detectplacepark;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystem.roadrunner.SubSys_RoadRunner;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.trajectorysequence.TrajectorySequence;

public class SpikeMark
{
    public static TrajectorySequence GetTrajectory(SubSys_RoadRunner subSysRoadRunner) {
        return subSysRoadRunner.trajectorySequenceBuilder(new Pose2d(64, 12.75, Math.toRadians(180.00)))
                .splineTo(new Vector2d(41, 12.75), Math.toRadians(180.00))
                .build();

    }
}
