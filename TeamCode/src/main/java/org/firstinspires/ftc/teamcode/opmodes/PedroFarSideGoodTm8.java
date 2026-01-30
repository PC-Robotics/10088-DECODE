package org.firstinspires.ftc.teamcode.opmodes;


import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

@Autonomous(name = "Pedro Far Side - Good Teammate", group = "Main", preselectTeleOp = "MainTeleop")
public class PedroFarSideGoodTm8 extends LinearOpMode {
    @IgnoreConfigurable
    private Robot robot = new Robot(this, false); // robot object
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
    private PathChain scorePreload, grab_b1, score_b1, grab_b2, score_b2, grab_loadzone, score_loadzone, moveToLoadZone;

    // tracks the current state of our autonomous
    private AutonomousState state = AutonomousState.SCORE_PRELOAD;


    private static Pose startPose          = new Pose(60, 9, Math.toRadians(270));
    private static Pose row1ApproachPose   = new Pose(42, 36 , Math.toRadians(180));
    private static Pose row1ApproachCP     = new Pose(60, 36);
    private static Pose grab_b1_pose       = new Pose(31, 36 , Math.toRadians(180));
    private static Pose row2ApproachPose   = new Pose(42, 60 , Math.toRadians(180));
    private static Pose row2ApproachCP     = new Pose(60, 60);
    private static Pose grab_b2_pose       = new Pose(31, 60 , Math.toRadians(180));
    private static Pose grab_loadzone_pose = new Pose(10, 18);
    private static Pose loadzoneCP1 = new Pose(44, 41);
    private static Pose loadzoneCP2        = new Pose(12, 41);
    private static Pose loadzonePose       = new Pose(22, 72 , Math.toRadians(180));



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
        robot.flywheel.setPosition(Flywheel.FLYWHEEL_SPIN_POSITION.CLOSE);
        robot.flywheel.spinToSpeed();

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
                robot.follower.followPath(scorePreload); // holds position
                setState(AutonomousState.GRAB_B1);
                break;

            case GRAB_B1:
                if (robot.follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                robot.transfer.start();
                sleep(4000);
                robot.transfer.stop();

                robot.follower.followPath(grab_b1, false);
                setState(AutonomousState.SCORE_B1);
                break;

            case SCORE_B1:
                if (robot.follower.isBusy()) { // still arriving to point
                    break;
                }

                sleep(200);
                robot.intake.stop();
                robot.follower.followPath(score_b1);
                setState(AutonomousState.GRAB_B2);
                break;

            case GRAB_B2:
                if (robot.follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                robot.transfer.start();
                sleep(4000);
                robot.transfer.stop();

                robot.follower.followPath(grab_b2, false);
                setState(AutonomousState.SCORE_B2);
                break;

            case SCORE_B2:
                if (robot.follower.isBusy()) { // still arriving to point
                    break;
                }

                sleep(200);
                robot.intake.stop();
                robot.follower.followPath(score_b2);
                setState(AutonomousState.GRAB_LOADZONE);
                break;

            case GRAB_LOADZONE:
                if (robot.follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                robot.transfer.start();
                sleep(4000);
                robot.transfer.stop();

                robot.follower.followPath(grab_loadzone, false);
                setState(AutonomousState.SCORE_LOADZONE);
                break;

            case SCORE_LOADZONE:
                if (robot.follower.isBusy()) { // still arriving to point
                    break;
                }

                sleep(200);
                robot.intake.stop();
                robot.follower.followPath(score_loadzone, false);
                setState(AutonomousState.MOVETO_LOADZONE);
                break;

            case MOVETO_LOADZONE:
                if (robot.follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                robot.transfer.start();
                sleep(4000);
                robot.transfer.stop();

                robot.follower.followPath(moveToLoadZone);
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
            row1ApproachCP = row1ApproachCP.mirror();
            grab_b1_pose = grab_b1_pose.mirror();
            row2ApproachPose = row2ApproachPose.mirror();
            row2ApproachCP = row2ApproachCP.mirror();
            grab_b2_pose = grab_b2_pose.mirror();
            grab_loadzone_pose = grab_loadzone_pose.mirror();
            loadzoneCP1 = loadzoneCP1.mirror();
            loadzoneCP2 = loadzoneCP2.mirror();
            loadzonePose = loadzonePose.mirror();
        }
    }


    private void buildPaths() {
        scorePreload = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, Robot.farScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), Robot.farScorePose.getHeading())
                .build();

        grab_b1 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(Robot.farScorePose, row1ApproachPose, row1ApproachCP))
                .setLinearHeadingInterpolation(Robot.farScorePose.getHeading(), row1ApproachPose.getHeading())
                .addPath(new BezierLine(row1ApproachPose, grab_b1_pose))
                .setTangentHeadingInterpolation()
                .build();

        score_b1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(grab_b1_pose, Robot.farScorePose))
                .setLinearHeadingInterpolation(grab_b1_pose.getHeading(), Robot.farScorePose.getHeading())
                .build();

        grab_b2 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(Robot.farScorePose, row2ApproachPose, row2ApproachCP))
                .setLinearHeadingInterpolation(Robot.farScorePose.getHeading(), row2ApproachPose.getHeading())
                .addPath(new BezierLine(row2ApproachPose, grab_b2_pose))
                .setTangentHeadingInterpolation()
                .build();

        score_b2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(grab_b2_pose, Robot.farScorePose))
                .setLinearHeadingInterpolation(grab_b2_pose.getHeading(), Robot.farScorePose.getHeading())
                .build();

        grab_loadzone = robot.follower.pathBuilder()
                .addPath(new BezierCurve(Robot.farScorePose, grab_loadzone_pose, loadzoneCP1, loadzoneCP2))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                0.5,
                                HeadingInterpolator.linear(Robot.farScorePose.getHeading(), Math.toRadians(180))
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                0.5,
                                1,
                                HeadingInterpolator.tangent
                        )
                ))
                .setBrakingStrength(0.5) // i guessed for this number lol
                .build();

        score_loadzone = robot.follower.pathBuilder()
                .addPath(new BezierLine(grab_loadzone_pose, Robot.farScorePose))
                .setLinearHeadingInterpolation(grab_loadzone_pose.getHeading(), Robot.farScorePose.getHeading())
                .build();

        moveToLoadZone = robot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(Robot.farScorePose, loadzonePose)))
                .setLinearHeadingInterpolation(Robot.farScorePose.getHeading(), loadzonePose.getHeading())
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
        GRAB_B2,
        SCORE_B2,
        GRAB_LOADZONE,
        SCORE_LOADZONE,
        MOVETO_LOADZONE,
        DEFAULT
    }
}
