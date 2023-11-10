package org.firstinspires.ftc.teamcode.commandGroups.autonomous;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.drive.commands.Cmd_SubSys_Drive_MoveToPoseRelative;

public class AutoCommands_Basic
{
    private SubSys_Drive subSysDrive;
    private Telemetry telemetry;

    public AutoCommands_Basic(
            SubSys_Drive subSysDrive,
            Telemetry telemetry) {

        this.subSysDrive = subSysDrive;
        this.telemetry = telemetry;

    }
    public class RedLeft extends SequentialCommandGroup
    {
        public RedLeft() {
            addCommands(
                    new Cmd_SubSys_Drive_MoveToPoseRelative(
                            subSysDrive,
                            telemetry,
                            new Pose2d(
                                    new Translation2d(0, 95),
                                    new Rotation2d()
                            ))
            );
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
