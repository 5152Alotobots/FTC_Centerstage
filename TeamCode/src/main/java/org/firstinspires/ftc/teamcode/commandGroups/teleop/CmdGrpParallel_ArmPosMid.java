package org.firstinspires.ftc.teamcode.commandGroups.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.arm.commands.Cmd_SubSys_Arm_RotateAndExtend;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand;
import org.firstinspires.ftc.teamcode.subsystem.hand.commands.Cmd_SubSys_Hand_RotateWristToPosition;

public class CmdGrpParallel_ArmPosMid extends ParallelCommandGroup
{
    private SubSys_Arm subSysArm;
    private SubSys_Hand subSysHand;
    private Telemetry telemetry;

    public CmdGrpParallel_ArmPosMid(SubSys_Arm subSysArm, SubSys_Hand subSysHand, Telemetry telemetry) {
        this.subSysArm = subSysArm;
        this.subSysHand = subSysHand;
        this.telemetry = telemetry;

        addCommands(
                new Cmd_SubSys_Arm_RotateAndExtend(
                        subSysArm,
                        telemetry,
                        () -> 39,
                        () -> 66
                ),
                new Cmd_SubSys_Hand_RotateWristToPosition(
                        subSysHand,
                        telemetry,
                        () -> 56
                ),
                 new InstantCommand(
                         () -> {
                             subSysHand.openLeftFront(true);
                             subSysHand.openRightFront(true);
                         }
                 )
        );
        addRequirements(subSysArm, subSysHand);
    }

}
