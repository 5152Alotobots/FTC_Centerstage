
// //package org.firstinspires.ftc.teamcode.subsystem.drive.commands;

// import com.acmerobotics.roadrunner.geometry.Pose2d;
// import com.acmerobotics.roadrunner.trajectory.Trajectory;
// import com.arcrobotics.ftclib.command.CommandBase;

// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
// import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive_GlobalPoseStorage;
// import org.firstinspires.ftc.teamcode.subsystem.drive.roadrunner.drive.SubSys_Drive_RoadRunnerDrive;
// import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;

// public class Cmd_SubSys_Drive_FollowRoadrunnerTraj extends CommandBase
// {
//     private SubSys_Drive subSysDrive;
//     private SubSys_Drive_RoadRunnerDrive subSysDriveRoadRunnerDrive;
//     private SubSys_Gyro subSysGyro;
//     private Trajectory trajectory;
//     private Telemetry telemetry;
//     private Pose2d currentPose;

//     public Cmd_SubSys_Drive_FollowRoadrunnerTraj(
//             SubSys_Drive subSysDrive,
//             SubSys_Drive_RoadRunnerDrive subSysDriveRoadRunnerDrive,
//             SubSys_Gyro subSysGyro,
//             Telemetry telemetry,
//             Trajectory trajectory) {

//         this.subSysDrive = subSysDrive;
//         this.subSysDriveRoadRunnerDrive = subSysDriveRoadRunnerDrive;
//         this.subSysGyro = subSysGyro;
//         this.telemetry = telemetry;
//         this.trajectory = trajectory;
//     }

//     /**
//      * The initial subroutine of a command.  Called once when the command is initially scheduled.
//      */
//     @Override
//     public void initialize() {
//         currentPose = subSysDrive.convertPose(SubSys_Drive_GlobalPoseStorage.currentPose);
//         subSysDriveRoadRunnerDrive.setPoseEstimate(currentPose);
//         subSysDriveRoadRunnerDrive.followTrajectory(trajectory);
//     }

//     /**
//      * The main body of a command.  Called repeatedly while the command is scheduled.
//      */
//     @Override
//     public void execute() {
//         subSysDriveRoadRunnerDrive.updatePoseEstimate();
//         currentPose = subSysDriveRoadRunnerDrive.getPoseEstimate();
//         telemetry.addLine("--- RoadRunner ---");
//         telemetry.addLine("STATUS: RUNNING");
//         telemetry.addData("Pose X/Y/Z", String.format("%s / %s / %s", currentPose.getX(), currentPose.getY(), currentPose.getHeading()));
//     }

//     /**
//      * The action to take when the command ends.  Called when either the command finishes normally,
//      * or when it interrupted/canceled.
//      *
//      * @param interrupted whether the command was interrupted/canceled
//      */
//     @Override
//     public void end(boolean interrupted) {
//         com.arcrobotics.ftclib.geometry.Pose2d endPose = subSysDrive.convertPose(subSysDriveRoadRunnerDrive.getPoseEstimate());
//         SubSys_Drive_GlobalPoseStorage.currentPose = endPose;
//     }

//     /**
//      * Whether the command has finished.  Once a command finishes, the scheduler will call its
//      * end() method and un-schedule it.
//      *
//      * @return whether the command has finished.
//      */
//     @Override
//     public boolean isFinished() {
//         return !subSysDriveRoadRunnerDrive.isBusy();
//     }
// }
