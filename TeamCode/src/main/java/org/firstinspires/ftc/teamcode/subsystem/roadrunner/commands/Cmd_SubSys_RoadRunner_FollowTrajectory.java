package org.firstinspires.ftc.teamcode.subsystem.roadrunner.commands;
//package org.firstinspires.ftc.teamcode.subsystem.drive.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_GlobalPoseStorage;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.SubSys_RoadRunner;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.trajectorysequence.TrajectorySequence;

public class Cmd_SubSys_RoadRunner_FollowTrajectory extends CommandBase
 {
     private SubSys_Drive subSysDrive;
     private SubSys_RoadRunner subSysRoadRunner;
     private SubSys_Gyro subSysGyro;
     private TrajectorySequence trajectorySequence;
     private Telemetry telemetry;
     private Pose2d currentPose;

     public Cmd_SubSys_RoadRunner_FollowTrajectory(
             SubSys_Drive subSysDrive,
             SubSys_RoadRunner subSysRoadRunner,
             SubSys_Gyro subSysGyro,
             Telemetry telemetry,
             TrajectorySequence trajectorySequence) {

         this.subSysDrive = subSysDrive;
         this.subSysRoadRunner = subSysRoadRunner;
         this.subSysGyro = subSysGyro;
         this.telemetry = telemetry;
         this.trajectorySequence = trajectorySequence;

         // Don't add roadrunner subsystem as it isn't REALLY a subsystem.
         addRequirements(subSysDrive, subSysGyro);
     }

     /**
      * The initial subroutine of a command.  Called once when the command is initially scheduled.
      */
     @Override
     public void initialize() {
         // currentPose = SubSys_Drive_GlobalPoseStorage.currentPose;
         // subSysRoadRunner.setPoseEstimate(currentPose);
         subSysRoadRunner.followTrajectorySequence(trajectorySequence);
     }

     /**
      * The main body of a command.  Called repeatedly while the command is scheduled.
      */
     @Override
     public void execute() {
         subSysRoadRunner.update();
         currentPose = subSysRoadRunner.getPoseEstimate();
         telemetry.addLine("--- RoadRunner ---");
         telemetry.addLine("STATUS: RUNNING");
         telemetry.addData("Pose X/Y/Z", String.format("%s / %s / %s", currentPose.getX(), currentPose.getY(), currentPose.getHeading()));
     }

     /**
      * The action to take when the command ends.  Called when either the command finishes normally,
      * or when it interrupted/canceled.
      *
      * @param interrupted whether the command was interrupted/canceled
      */
     @Override
     public void end(boolean interrupted) {
         Pose2d endPose = subSysRoadRunner.getPoseEstimate();
         SubSys_Drive_GlobalPoseStorage.currentPose = endPose;
     }

     /**
      * Whether the command has finished.  Once a command finishes, the scheduler will call its
      * end() method and un-schedule it.
      *
      * @return whether the command has finished.
      */
     @Override
     public boolean isFinished() {
         return !subSysRoadRunner.isBusy();
     }
 }
