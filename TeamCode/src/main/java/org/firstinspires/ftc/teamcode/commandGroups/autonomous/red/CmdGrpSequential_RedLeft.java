package org.firstinspires.ftc.teamcode.commandGroups.autonomous.red;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
// import org.firstinspires.ftc.teamcode.subsystem.drive.roadrunner.drive.SubSys_Drive_RoadRunnerDrive;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand;
import org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake;

public class CmdGrpSequential_RedLeft  extends SequentialCommandGroup
{
    private SubSys_Drive subSysDrive;
   // private SubSys_Drive_RoadRunnerDrive subSysDriveRoadRunnerDrive;
    private SubSys_Gyro subSysGyro;
    private SubSys_Arm subSysArm;
    private SubSys_Intake subSysIntake;
    private SubSys_Hand subSysHand;

    public CmdGrpSequential_RedLeft(
            SubSys_Drive subSysDrive,
          //  SubSys_Drive_RoadRunnerDrive subSysDriveRoadRunnerDrive,
            SubSys_Gyro subSysGyro,
            SubSys_Arm subSysArm,
            SubSys_Intake subSysIntake,
            SubSys_Hand subSysHand) {

        addCommands(
        );
    }
}
