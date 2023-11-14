package org.firstinspires.ftc.teamcode.subsystem.roadrunner.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.SubSys_RoadRunner;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.SubSys_RoadRunner_Constants;

import java.util.List;

/*
 * This routine is designed to tune the PID coefficients used by the REV Expansion Hubs for closed-
 * loop velocity control. Although it may seem unnecessary, tuning these coefficients is just as
 * important as the positional parameters. Like the other manual tuning routines, this op mode
 * relies heavily upon the dashboard. To access the dashboard, connect your computer to the RC's
 * WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're using the RC
 * phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once you've successfully
 * connected, start the program, and your robot will begin moving forward and backward according to
 * a motion profile. Your job is to graph the velocity errors over time and adjust the PID
 * coefficients (note: the tuning variable will not appear until the op mode finishes initializing).
 * Once you've found a satisfactory set of gains, add them to the DriveConstants.java file under the
 * MOTOR_VELO_PID field.
 *
 * Recommended tuning process:
 *
 * 1. Increase kP until any phase lag is eliminated. Concurrently increase kD as necessary to
 *    mitigate oscillations.
 * 2. Add kI (or adjust kF) until the steady state/constant velocity plateaus are reached.
 * 3. Back off kP and kD a little until the response is less oscillatory (but without lag).
 *
 * Pressing Y/Δ (Xbox/PS4) will pause the tuning process and enter driver override, allowing the
 * user to reset the position of the bot in the event that it drifts off the path.
 * Pressing B/O (Xbox/PS4) will cede control back to the tuning process.
 */
@Config
@Autonomous(group = "drive")
public class DriveVelocityPIDTuner extends LinearOpMode {
    public static double DISTANCE = 72; // in

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, SubSys_RoadRunner_Constants.MAX_VEL, SubSys_RoadRunner_Constants.MAX_ACCEL);
    }

    @Override
    public void runOpMode() {
        if (!SubSys_RoadRunner_Constants.RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("%s does not need to be run if the built-in motor velocity" +
                    "PID is not in use", getClass().getSimpleName());
        }

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SubSys_RoadRunner drive = new SubSys_RoadRunner(hardwareMap);

        Mode mode = Mode.TUNING_MODE;

        double lastKp = SubSys_RoadRunner_Constants.MOTOR_VELO_PID.p;
        double lastKi = SubSys_RoadRunner_Constants.MOTOR_VELO_PID.i;
        double lastKd = SubSys_RoadRunner_Constants.MOTOR_VELO_PID.d;
        double lastKf = SubSys_RoadRunner_Constants.MOTOR_VELO_PID.f;

        drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SubSys_RoadRunner_Constants.MOTOR_VELO_PID);

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();


        while (!isStopRequested()) {
            telemetry.addData("mode", mode);

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }

                    // calculate and set the motor power
                    double profileTime = clock.seconds() - profileStart;

                    if (profileTime > activeProfile.duration()) {
                        // generate a new profile
                        movingForwards = !movingForwards;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    MotionState motionState = activeProfile.get(profileTime);
                    double targetPower = SubSys_RoadRunner_Constants.kV * motionState.getV();
                    drive.setDrivePower(new Pose2d(targetPower, 0, 0));

                    List<Double> velocities = drive.getWheelVelocities();

                    // update telemetry
                    telemetry.addData("targetVelocity", motionState.getV());
                    for (int i = 0; i < velocities.size(); i++) {
                        telemetry.addData("measuredVelocity" + i, velocities.get(i));
                        telemetry.addData(
                                "error" + i,
                                motionState.getV() - velocities.get(i)
                        );
                    }
                    break;
                case DRIVER_MODE:
                    if (gamepad1.b) {
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );
                    break;
            }

            if (lastKp != SubSys_RoadRunner_Constants.MOTOR_VELO_PID.p || lastKd != SubSys_RoadRunner_Constants.MOTOR_VELO_PID.d
                    || lastKi != SubSys_RoadRunner_Constants.MOTOR_VELO_PID.i || lastKf != SubSys_RoadRunner_Constants.MOTOR_VELO_PID.f) {
                drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SubSys_RoadRunner_Constants.MOTOR_VELO_PID);

                lastKp = SubSys_RoadRunner_Constants.MOTOR_VELO_PID.p;
                lastKi = SubSys_RoadRunner_Constants.MOTOR_VELO_PID.i;
                lastKd = SubSys_RoadRunner_Constants.MOTOR_VELO_PID.d;
                lastKf = SubSys_RoadRunner_Constants.MOTOR_VELO_PID.f;
            }

            telemetry.update();
        }
    }
}
