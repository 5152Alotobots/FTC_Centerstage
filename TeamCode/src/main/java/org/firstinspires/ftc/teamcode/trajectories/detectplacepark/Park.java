package org.firstinspires.ftc.teamcode.trajectories.detectplacepark;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystem.roadrunner.SubSys_RoadRunner;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.trajectorysequence.TrajectorySequence;

public class Park
{
    public static TrajectorySequence GetTrajectory(SubSys_RoadRunner subSysRoadRunner) {
        return subSysRoadRunner.trajectorySequenceBuilder(new Pose2d(35.24, 12.03, Math.toRadians(90.00)))
                .splineTo(new Vector2d(60.81, 61.23), Math.toRadians(90.00))
                .build();


    }
}
