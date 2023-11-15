package org.firstinspires.ftc.teamcode.commandGroups.autonomous;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commandGroups.autonomous.red.CmdGrpSequential_RedLeft;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_MoveToPoseRelative;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand;
import org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.SubSys_RoadRunner;

public class AutoCommands_DetectAndPlace
{
    private SubSys_Drive subSysDrive;
    private SubSys_RoadRunner subSysRoadRunner;
    private SubSys_Gyro subSysGyro;
    private SubSys_Arm subSysArm;
    private SubSys_Intake subSysIntake;
    private SubSys_Hand subSysHand;
    private Telemetry telemetry;

    public AutoCommands_DetectAndPlace(
            SubSys_Drive subSysDrive,
            SubSys_RoadRunner subSysRoadRunner,
            SubSys_Gyro subSysGyro,
            SubSys_Arm subSysArm,
            SubSys_Intake subSysIntake,
            SubSys_Hand subSysHand,
            Telemetry telemetry) {

        this.subSysDrive = subSysDrive;
        this.subSysRoadRunner = subSysRoadRunner;
        this.subSysGyro = subSysGyro;
        this.subSysArm = subSysArm;
        this.subSysIntake = subSysIntake;
        this.subSysHand = subSysHand;
        this.telemetry = telemetry;

    }
    public class RedLeft extends SequentialCommandGroup
    {
        public RedLeft() {
            addCommands(
                    new CmdGrpSequential_RedLeft(
                            subSysDrive,
                            subSysRoadRunner,
                            subSysGyro,
                            subSysArm,
                            subSysIntake,
                            subSysHand,
                            telemetry));
        }
    }
    
    public class RedRight extends SequentialCommandGroup
    {
        public RedRight() {
            addCommands(
                    new Cmd_SubSys_Drive_MoveToPoseRelative(
                            subSysDrive,
                            telemetry,
                            new Pose2d(
                                    new Translation2d(0, 45),
                                    new Rotation2d()
                            ))
            );
        }
    }
    
    public class BlueLeft extends SequentialCommandGroup
    {
        public BlueLeft() {
            addCommands(
                    new Cmd_SubSys_Drive_MoveToPoseRelative(
                            subSysDrive,
                            telemetry,
                            new Pose2d(
                                    new Translation2d(0, -45),
                                    new Rotation2d()
                            ))
            );
        }
    }
    
    public class BlueRight extends SequentialCommandGroup
    {
        public BlueRight() {
            addCommands(
                    new Cmd_SubSys_Drive_MoveToPoseRelative(
                            subSysDrive,
                            telemetry,
                            new Pose2d(
                                    new Translation2d(0, -95),
                                    new Rotation2d()
                            ))
            );
        }
    }
    
}
