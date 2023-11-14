package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystem.roadrunner.SubSys_RoadRunner;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.trajectorysequence.TrajectorySequence;

public class testTraj
{
    public static TrajectorySequence testTraj(SubSys_RoadRunner subSysRoadRunner) {
        return subSysRoadRunner.trajectorySequenceBuilder(new Pose2d(60.98, 12.79, Math.toRadians(180.00)))
                .splineTo(new Vector2d(14.30, 12.95), Math.toRadians(181.61))
                .splineTo(new Vector2d(36.42, 46.85), Math.toRadians(90.00))
                .build();

    }
}
