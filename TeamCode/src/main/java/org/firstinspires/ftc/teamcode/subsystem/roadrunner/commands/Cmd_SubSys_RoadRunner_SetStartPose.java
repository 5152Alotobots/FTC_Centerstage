package org.firstinspires.ftc.teamcode.subsystem.roadrunner.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.roadrunner.SubSys_RoadRunner;

public class Cmd_SubSys_RoadRunner_SetStartPose extends InstantCommand
{
    private SubSys_RoadRunner subSysRoadRunner;
    private Pose2d startPose;

    public Cmd_SubSys_RoadRunner_SetStartPose(
            SubSys_RoadRunner subSysRoadRunner,
            Pose2d startPose) {
        this.subSysRoadRunner = subSysRoadRunner;
        this.startPose = startPose;
    }
    @Override
    public void initialize() {
        subSysRoadRunner.setPoseEstimate(startPose);
    }
}
