package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Utility.applyDeadzone;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "MainTeleop", group = "Main")
public class MainTeleop extends LinearOpMode {
    private Robot robot;

    private TelemetryManager panelsTelemetry;

    private double gamepad2RightTrigger;
    private boolean autoTurn = false;
    private boolean slowMode = false;
    private final static double SLOW_MODE_MULTIPLIER = 0.4;
    private double driveSpeed = 1;

    public void runOpMode() {
        robot = new Robot(this, false);
        robot.init();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        Alliance a = Alliance.BLUE;
        gamepad1.rumble(500); // reminder to set alliance team
        while (opModeInInit()) {
            if (gamepad1.leftBumperWasPressed()) {
                a = Alliance.BLUE;
            } else if (gamepad1.rightBumperWasPressed()) {
                a = Alliance.RED;
            }

            panelsTelemetry.debug(
                    "Select Alliance",
                    "Left Bumper: Blue Alliance",
                    "Right Bumper: Red Alliance",
                    " ------------------------- ",
                    "Current Alliance: " + a.toString()
            );
            panelsTelemetry.update(telemetry);
        }
        robot.setAlliance(a);

        robot.follower.setStartingPose(Robot.endPose == null ? new Pose() : Robot.endPose);
        robot.follower.startTeleOpDrive(false);
        robot.update();

        waitForStart();

        double angleOffset = Robot.alliance == Alliance.RED ? 0 : Math.toRadians(180);
        while (opModeIsActive()) { // hi
            robot.update();

            readController();
            driveSpeed = slowMode ? SLOW_MODE_MULTIPLIER : 1;

            if (autoTurn) {
                robot.follower.holdPoint(
                        new BezierPoint(robot.currentPose),
                        Robot.scorePose.getHeading(),
                        false);
            } else {
                if (robot.isRobotCentric) {
                    robot.follower.setTeleOpDrive(
                            -gamepad1.left_stick_y * driveSpeed,
                            -gamepad1.left_stick_x * driveSpeed,
                            -gamepad1.right_stick_x * driveSpeed,
                            true
                    );
                } else {
                    robot.follower.setTeleOpDrive(
                            -gamepad1.left_stick_y * driveSpeed,
                            -gamepad1.left_stick_x * driveSpeed,
                            -gamepad1.right_stick_x * driveSpeed,
                            false,
                            angleOffset
                    );
                }
            }


            flywheelControl();
            intakeAndTransferControl();

            reloadTelemetry();
            telemetry.update();
        }
    }

    public void readController() {
        slowMode = gamepad1.right_bumper;
        autoTurn = gamepad1.dpad_up;

        gamepad2RightTrigger = applyDeadzone(gamepad2.right_trigger, 0.2);
    }


    private void flywheelControl() {
        if (gamepad2.triangleWasPressed()) {
            robot.flywheel.setPosition(Flywheel.FLYWHEEL_SPIN_POSITION.FAR);
        } else if (gamepad2.crossWasPressed()) {
            robot.flywheel.setPosition(Flywheel.FLYWHEEL_SPIN_POSITION.CLOSE);
        }

        if (gamepad2.dpadUpWasPressed()) {
            robot.flywheel.spinToSpeed();
        } else if (gamepad2.dpadDownWasPressed()) {
            robot.flywheel.stop();
        }
    }

    private void intakeAndTransferControl() {
        // if we are shooting then run transfer AND intake
        if (gamepad2.square) {
            robot.transfer.start();
            robot.intake.intake();
            return; // skip any other logic
        }
        robot.transfer.stop(); // if we are not shooting stop transfer

        // intake logic
        if (gamepad2.right_bumper) {
            robot.intake.outtake();
        } else if (gamepad2RightTrigger > 0) {
            robot.intake.intake(gamepad2RightTrigger);
        } else {
            robot.intake.stop();
        }
    }


    private void reloadTelemetry() {
        List<String> lines = new ArrayList<>();

        lines.add("Alliance: " + Robot.alliance);
        lines.add("Slow mode: " + (slowMode ? "ON" : "OFF"));
        lines.add("AutoTurn: " + (autoTurn ? "ON" : "OFF"));
        lines.add("Pose: " + robot.currentPose);

        lines.addAll(robot.intake.getTelemetry());
        lines.addAll(robot.flywheel.getTelemetry());
        lines.addAll(robot.transfer.getTelemetry());

        panelsTelemetry.debug(lines.toArray(new String[0]));
        panelsTelemetry.update(telemetry);
    }

}
