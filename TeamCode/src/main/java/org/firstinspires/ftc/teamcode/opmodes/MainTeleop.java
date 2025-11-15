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

@TeleOp(name = "MainTeleop", group = "Main")
public class MainTeleop extends LinearOpMode {
    Robot robot = new Robot(this);

    private TelemetryManager panelsTelemetry;

    private double gamepad2RightTrigger;
    private boolean autoTurn = false;
    private boolean slowMode = false;
    private final static double SLOW_MODE_MULTIPLIER = 0.4;

    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        robot.init();

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
        Robot.alliance = a;

        robot.follower.setStartingPose(Robot.endPose == null ? new Pose() : Robot.endPose);
        robot.update();

        waitForStart();

        robot.follower.startTeleOpDrive();

        double angleOffset = Robot.alliance == Alliance.RED ? 0 : Math.toRadians(180);
        while (opModeIsActive()) { // hi
            robot.update();

            readController();

            if (autoTurn) {
                robot.follower.holdPoint(
                        new BezierPoint(robot.currentPose),
                        Robot.scorePose.getHeading(),
                        false);
            }
            if (!slowMode) {
                robot.follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false,
                        angleOffset
                );
            } else {
                robot.follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * SLOW_MODE_MULTIPLIER,
                        -gamepad1.left_stick_x * SLOW_MODE_MULTIPLIER,
                        -gamepad1.right_stick_x * SLOW_MODE_MULTIPLIER,
                        false,
                        angleOffset
                );
            }


            flywheelControl();
            intakeControl();

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

    private void intakeControl() {
        if (gamepad2.right_bumper) {
            robot.intake.outtake();
        } else if (gamepad2RightTrigger != 0 && robot.canIntake()) {
            robot.intake.intake(gamepad2RightTrigger);
        } else {
            robot.intake.stop();
        }
    }


    private void reloadTelemetry() {
        panelsTelemetry.debug(robot.intake.getTelemetry());
        panelsTelemetry.update(telemetry);
    }
}
