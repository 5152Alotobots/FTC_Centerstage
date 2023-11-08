package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.autonomous.AutoCommands_Basic;
import org.firstinspires.ftc.teamcode.subsystem.arm.SubSys_Arm;
import org.firstinspires.ftc.teamcode.subsystem.drive.SubSys_Drive;
import org.firstinspires.ftc.teamcode.subsystem.driverstation.SubSys_DriverStation;
import org.firstinspires.ftc.teamcode.subsystem.gyro.SubSys_Gyro;
import org.firstinspires.ftc.teamcode.subsystem.hand.SubSys_Hand;
import org.firstinspires.ftc.teamcode.subsystem.intake.SubSys_Intake;
import org.firstinspires.ftc.teamcode.subsystem.launcher.SubSys_Launcher;

import java.util.HashMap;

@Autonomous
public class Basic extends CommandOpMode
{
    enum ALLIANCE {
        RED, BLUE
    }

    enum SIDE {
        LEFT, RIGHT
    }

    enum START_POS {
        RED_LEFT, RED_RIGHT,
        BLUE_LEFT, BLUE_RIGHT
    }

    private final SubSys_DriverStation subSysDriverStation = new SubSys_DriverStation(gamepad1, gamepad2);
    private final SubSys_Gyro subSysGyro = new SubSys_Gyro(hardwareMap);
    private final SubSys_Drive subSysDrive = new SubSys_Drive(subSysGyro, hardwareMap);
    private final SubSys_Arm subSysArm = new SubSys_Arm(hardwareMap);
    private final SubSys_Hand subSysHand = new SubSys_Hand(subSysArm, hardwareMap);
    private final SubSys_Intake subSysIntake = new SubSys_Intake(hardwareMap);
    private final SubSys_Launcher subSysLauncher = new SubSys_Launcher(hardwareMap);
    private final AutoCommands_Basic autoCommandsBasic = new AutoCommands_Basic(subSysDrive, telemetry);

    // subSysApriltag = new SubSys_Apriltag();
    // subSysTensorflow = new SubSys_Tensorflow();
    // subSysVisionportal = new SubSys_Visionportal(subSysTensorflow, subSysApriltag, hardwareMap);

    public ALLIANCE alliance() {
        if (subSysDriverStation.redSideButton.get()) {
            telemetry.addLine("Alliance selected as: RED");
            return ALLIANCE.RED;
        } else if (subSysDriverStation.blueSideButton.get()){
            telemetry.addLine("Alliance selected as: BLUE");
            return ALLIANCE.BLUE;
        } else {
            telemetry.addLine("CRITICAL: NO ALLIANCE SELECTED! PLEASE SELECT");
            return ALLIANCE.RED; // default
        }
    }

    public SIDE side() {
        if (subSysDriverStation.leftButton.get()) {
            telemetry.addLine("Side selected as: LEFT");
            return SIDE.LEFT;
        } else if (subSysDriverStation.rightButton.get()) {
            telemetry.addLine("Side selected as: RIGHT");
            return SIDE.RIGHT;
        } else {
            telemetry.addLine("CRITICAL: NO SIDE SELECTED! PLEASE SELECT");
            return SIDE.LEFT; // default
        }
    }

    public START_POS startPos() {
        ALLIANCE alliance = alliance();
        SIDE side = side();

        if (alliance == ALLIANCE.BLUE) {
            if (side == SIDE.RIGHT) {
                return START_POS.BLUE_RIGHT;
            } else {
                return START_POS.BLUE_LEFT;
            }
        } else {
            if (side == SIDE.RIGHT) {
                return START_POS.RED_RIGHT;
            } else {
                return START_POS.RED_LEFT;
            }
        }
    }

    @Override
    public void initialize() {
        register(subSysDriverStation, subSysGyro, subSysDrive, subSysArm, subSysHand, subSysIntake, subSysLauncher);

        SelectCommand autoCommand = new SelectCommand(
                new HashMap<Object, Command>() {{
                    put(START_POS.RED_LEFT, autoCommandsBasic.new RedLeft());
                    put(START_POS.RED_RIGHT, autoCommandsBasic.new RedRight());
                    put(START_POS.BLUE_LEFT, autoCommandsBasic.new BlueLeft());
                    put(START_POS.BLUE_RIGHT, autoCommandsBasic.new BlueRight());
                }},
                // Selector
                this::startPos
        );

        autoCommand.schedule(false);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler, update telemetry
        while (!isStopRequested() && opModeIsActive()) {
            run();
            telemetry.update();
        }
        reset();
    }

}
