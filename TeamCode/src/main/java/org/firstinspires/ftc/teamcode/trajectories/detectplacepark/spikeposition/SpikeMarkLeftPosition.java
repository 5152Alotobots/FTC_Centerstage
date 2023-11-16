package org.firstinspires.ftc.teamcode.trajectories.detectplacepark.spikeposition;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.subsystem.roadrunner.SubSys_RoadRunner;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.trajectorysequence.TrajectorySequence;

public class SpikeMarkLeftPosition
{
    public static TrajectorySequence GetTrajectory(SubSys_RoadRunner subSysRoadRunner) {
        return subSysRoadRunner.trajectorySequenceBuilder(new Pose2d(40.00, 12.62, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(29.94, 7.57, Math.toRadians(90.00)))
                .build();

    }
}
