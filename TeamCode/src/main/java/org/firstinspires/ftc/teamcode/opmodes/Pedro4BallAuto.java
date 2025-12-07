// to view the path, upload Pedro4BallAutoBlue.pp into https://visualizer.pedropathing.com/

package org.firstinspires.ftc.teamcode.opmodes;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;


/* This autonomous routine shoots 4 balls
 * starting position: Robot intake against the back wall, right edge of robot on the tape such that both wheels are in the triangle
 * Preload: 1 ball
 * Steps to autonomous:
 * 1. Robot advances forwards and shoots preload
 * 2. Robot grabs rightmost ball of row closest to the back wall
 * 3. Robot goes back to small triangle and shoots
 * 4. Robot grabs middlemost ball of row closest to the back wall
 * 5. Robot goes back to small triangle and shoots
 * 6. Robot grabs leftmost ball of row closest to the back wall
 * 7. Robot goes back to small triangle and shoots
 * 8. Robot parks close to the lever with intake facing loading zone
 *
 * NOTE: This autonomous uses PedroPathing, an extension of RoadRunner. Learn about PedroPathing here:https://pedropathing.com/docs/pathing
 */
@Autonomous(name = "Pedro 4 Ball", group = "Main", preselectTeleOp = "MainTeleop")
@Configurable // required for Panels (Dashboard so you can see stuff while the robot is running)
@SuppressWarnings("FieldMayBeFinal") // SILENCE PUNY WARNINGS
public class Pedro4BallAuto extends LinearOpMode {
    @IgnoreConfigurable
    private Robot robot = new Robot(this); // robot object
    /* ball naming scheme
    c b a
    o o o 3

    o o o 2

    o o o 1

    a1 is closest to small triangle
    */
    // PEDRO STUFF
    @IgnoreConfigurable
    private TelemetryManager panelsTelemetry;

    @IgnoreConfigurable
    private final Timer pathTimer = new Timer(); // timer that resets every time we move to a new action
    private final Timer opmodeTimer = new Timer(); // timer that runs the entire match

    // paths that we will use (initialized in buildPaths())
    @IgnoreConfigurable
    private PathChain scorePreload, score_a1, score_b1, score_c1, moveToLever, grab_a1, grab_b1, grab_c1;

    // tracks the current state of our autonomous
    private AutonomousState state = AutonomousState.SCORE_PRELOAD;

    // fixed poses. you can check https://visualizer.pedropathing.com/. Center of robot is tracking point
    private static Pose startPose = new Pose(56, 9, Math.toRadians(270));
    private static Pose row1ApproachPose = new Pose(46, 36, Math.toRadians(180));
    private static Pose row1ApproachControlPoint = new Pose(56, 36);
    private static Pose grab_b1_Pose = new Pose(31, 36, Math.toRadians(180));
    private static Pose grab_c1_Pose = new Pose(26, 36, Math.toRadians(180));
    public static Pose leverPose = new Pose(22, 72, Math.toRadians(270));


    @Override
    public void runOpMode() {
        // initialize all our stuff
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        robot.init();

        panelsTelemetry.addData("Status", "Initialized");
        panelsTelemetry.update(telemetry);

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

        waitForStart();

        // necessary to do here because now we know the alliance colors
        robot.setAlliance(a);
        mirrorPoses();
        buildPaths();
        robot.follower.setStartingPose(startPose);
        robot.flywheel.setPosition(Flywheel.FLYWHEEL_SPIN_POSITION.FAR);

        // set timers to 0 now that we have begun
        pathTimer.resetTimer();
        opmodeTimer.resetTimer();

        // this is our main loop which will run over and over again until the user presses stop
        while (opModeIsActive()) {
            robot.update(); // update Pedro Pathing's robot
            telemetry.update();

            // update the FSM
            autonomousPathUpdate();

            // log values to both the Control Hub logger and to Panels logger
            panelsTelemetry.debug(
                    "Time: " + opmodeTimer.getElapsedTimeSeconds(),
                    "X: " + robot.currentPose.getX(),
                    "Y: " + robot.currentPose.getY(),
                    "Heading: " + robot.currentPose.getHeading(),
                    "State: " + state.toString(),
                    "Alliance: " + Robot.alliance.toString()
            );
            panelsTelemetry.update(telemetry);
        }
    }


    private void autonomousPathUpdate() {
        switch (state) {
            case SCORE_PRELOAD:
                if (robot.flywheel.flywheelState == Flywheel.FLYWHEEL_STATE.IDLE) {
                    robot.flywheel.spinToSpeed();
                }
                sleep(2000);
                robot.follower.followPath(scorePreload); // holds position
                setState(AutonomousState.GRAB_B1);
                break;

            case GRAB_B1:
                if (robot.follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                sleep(500);
                robot.flywheel.stop();

                robot.follower.followPath(grab_b1, false);
                setState(AutonomousState.SCORE_B1);
                break;

            case SCORE_B1:
                if (robot.follower.isBusy()) { // still arriving to point
                    break;
                }

                sleep(200);
                robot.intake.stop();
                if (robot.flywheel.flywheelState == Flywheel.FLYWHEEL_STATE.IDLE) {
                    robot.flywheel.spinToSpeed();
                }
                robot.follower.followPath(score_b1);
                setState(AutonomousState.GRAB_C1);
                break;

            case GRAB_C1:
                if (robot.follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                sleep(500);
                robot.flywheel.stop();

                robot.follower.followPath(grab_c1, false);
                setState(AutonomousState.SCORE_C1);
                break;

            case SCORE_C1:
                if (robot.follower.isBusy()) { // still arriving to point
                    break;
                }

                sleep(200);
                robot.intake.stop();
                if (robot.flywheel.flywheelState == Flywheel.FLYWHEEL_STATE.IDLE) {
                    robot.flywheel.spinToSpeed();
                }
                robot.follower.followPath(score_c1);
                setState(AutonomousState.MOVE_TO_LEVER);
                break;

            case MOVE_TO_LEVER:
                if (robot.follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                sleep(500);
                robot.intake.stop();
                robot.flywheel.stop();

                robot.follower.followPath(moveToLever);
                setState(AutonomousState.DEFAULT); // default
                break;

            case DEFAULT:
                break;
        }
    }


    private void mirrorPoses() {
        // flipping poses for alliances
        if (Robot.alliance == Alliance.RED) {
            startPose = startPose.mirror();
            row1ApproachPose = row1ApproachPose.mirror();
            row1ApproachControlPoint = row1ApproachControlPoint.mirror();
            grab_b1_Pose = grab_b1_Pose.mirror();
            grab_c1_Pose = grab_c1_Pose.mirror();
            leverPose = leverPose.mirror();
        }
    }


    private void buildPaths() {
        scorePreload = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, Robot.scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), Robot.scorePose.getHeading())
                .build();

        grab_c1 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(Robot.scorePose, row1ApproachControlPoint, row1ApproachPose))
                .setLinearHeadingInterpolation(Robot.scorePose.getHeading(), row1ApproachPose.getHeading())
                .addPath(new BezierLine(row1ApproachPose, grab_c1_Pose))
                .setTangentHeadingInterpolation()
                .build();

        score_c1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(grab_c1_Pose, Robot.scorePose))
                .setLinearHeadingInterpolation(grab_c1_Pose.getHeading(), Robot.scorePose.getHeading())
                .build();

        moveToLever = robot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(Robot.scorePose, leverPose)))
                .setLinearHeadingInterpolation(Robot.scorePose.getHeading(), leverPose.getHeading())
                .build();
    }


    public void setState(AutonomousState state) {
        this.state = state;
        pathTimer.resetTimer();
    }

    public enum AutonomousState {
        SCORE_PRELOAD,
        GRAB_B1,
        SCORE_B1,
        GRAB_C1,
        SCORE_C1,
        MOVE_TO_LEVER,
        DEFAULT
    }
}
