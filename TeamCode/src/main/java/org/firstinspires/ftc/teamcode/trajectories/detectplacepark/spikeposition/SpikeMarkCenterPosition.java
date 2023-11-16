package org.firstinspires.ftc.teamcode.trajectories.detectplacepark.spikeposition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystem.roadrunner.SubSys_RoadRunner;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.trajectorysequence.TrajectorySequence;

public class SpikeMarkCenterPosition
{
    public static TrajectorySequence GetTrajectory(SubSys_RoadRunner subSysRoadRunner) {
        return subSysRoadRunner.trajectorySequenceBuilder(new Pose2d(40.00, 12.62, Math.toRadians(180.00)))
                .splineTo(new Vector2d(18.40, 12.11), Math.toRadians(180.00))
                .build();

    }
}
