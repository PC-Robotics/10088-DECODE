package org.firstinspires.ftc.teamcode.opmodes;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import java.util.Map;
import java.util.Objects;


/* This autonomous routine shoots 4 balls from the red side
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
@Autonomous(name = "Pedro 4 Ball (Red Side)", group = "Main", preselectTeleOp = "MainTeleop")
@Configurable // required for Panels (Dashboard so you can see stuff while the robot is running)
@SuppressWarnings("FieldMayBeFinal") // SILENCE PUNY WARNINGS
public class Pedro4BallAutoRed extends LinearOpMode {
    @IgnoreConfigurable
    private Robot robot = new Robot(this); // robot object
    /* ball naming scheme
    c b a
    o o o 3

    o o o 2

    o o o 1

    a1 is closest to small triangle
    */

    // converts column names (a, b, c) to numbers for the program (1, 2, 3)
    private static final Map<String, Integer> COLUMN_TO_NUMBER = Map.of(
            "a", 1,
            "b", 2,
            "c", 3
    );

    private static final int ballDiameter = 5; // inches
    private static final int BallA_XPosition = 36; // inches from the left wall
    private static int OffsetFromBallA_XPosition = 10; // how far should the the front of the robot be from the first ball? (inches)

    private static final int Ball1_YPosition = 36; // inches from the back wall
    private static final int BallRowOffset = 24; // inches between rows of balls


    // PEDRO STUFF
    @IgnoreConfigurable
    private Follower follower; // our main robot object in Pedro Pathing
    @IgnoreConfigurable
    private TelemetryManager panelsTelemetry; // telemetry for Panels
    @IgnoreConfigurable
    private Pose currentPose; // current position (called a Pose which is x, y, and heading)
    private final Timer pathTimer = new Timer(); // timer that resets every time we move to a new action
    private final Timer opmodeTimer = new Timer(); // timer that runs the entire match

    // paths that we will use (initialized in buildPaths())
    @IgnoreConfigurable
    private Path scorePreload, score_a1, score_b1, score_c1, moveToLever;
    @IgnoreConfigurable
    private PathChain grab_a1, grab_b1, grab_c1;

    // tracks the current state of our autonomous
    private int pathState;

    // fixed poses. you can check https://visualizer.pedropathing.com/. Center of robot is tracking point
    private static Pose startPose = new Pose(56, 9, Math.toRadians(270));
    private static Pose scorePose = new Pose(startPose.getX(), 18); // heading is calculated in runOpMode
    private static Pose leverPose = new Pose(22, 72, Math.toRadians(270));
    
    
    @Override
    public void runOpMode() {
        // find the angle between the red goal and the shooting position, then flip it because our shooter is on the back
        scorePose = scorePose.setHeading(Math.toRadians(298));

        // initialize all our stuff
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        buildPaths(); // pre-build paths so we don't waste runtime
        follower.setStartingPose(startPose);
        robot.init();

        log("Status", "Initialized");
        telemetry.update();

        waitForStart(); // waits for the user to press play
        // set timers to 0 now that we have begun
        pathTimer.resetTimer();
        opmodeTimer.resetTimer();

        // this is our main loop which will run over and over again until the user presses stop
        while (opModeIsActive()) {
            follower.update(); // update Pedro Pathing's robot
            panelsTelemetry.update();
            currentPose = follower.getPose(); // grab our current pose

            // update the FSM
            autonomousPathUpdate();

            // log values to both the Control Hub logger and to Panels logger
            log("Time", opmodeTimer.getElapsedTimeSeconds());
            log("X", currentPose.getX());
            log("Y", currentPose.getY());
            log("Heading", currentPose.getHeading());
            telemetry.update();
        }
    }



    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (robot.flywheel.state == Flywheel.FLYWHEEL_STATE.IDLE) {
                    robot.flywheel.spinToSpeed();
                }
                sleep(2000);
                follower.followPath(scorePreload); // holds position
                setPathState(1);
                break;

            case 1:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                // TODO - the robots
                sleep(500);
                robot.flywheel.stop();

                follower.followPath(grab_a1, false);
                setPathState(2);
                break;

            case 2:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                sleep(200);
                robot.intake.stop();
                if (robot.flywheel.state == Flywheel.FLYWHEEL_STATE.IDLE) {
                    robot.flywheel.spinToSpeed();
                }
                follower.followPath(score_a1);
                setPathState(3);
                break;

            case 3:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                sleep(500);
                robot.flywheel.stop();

                follower.followPath(grab_b1, false);
                setPathState(4);
                break;

            case 4:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                sleep(200);
                robot.intake.stop();
                if (robot.flywheel.state == Flywheel.FLYWHEEL_STATE.IDLE) {
                    robot.flywheel.spinToSpeed();
                }
                follower.followPath(score_b1);
                setPathState(5);
                break;

            case 5:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                sleep(500);
                robot.flywheel.stop();

                follower.followPath(grab_c1, false);
                setPathState(6);
                break;

            case 6:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                sleep(200);
                robot.intake.stop();
                if (robot.flywheel.state == Flywheel.FLYWHEEL_STATE.IDLE) {
                    robot.flywheel.spinToSpeed();
                }
                follower.followPath(score_c1);
                setPathState(7);
                break;

            case 7:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                sleep(500);
                robot.intake.stop();
                robot.flywheel.stop();

                follower.followPath(moveToLever);
                setPathState(-1); // default
                break;

            default:
                break;
        }
    }

    
    private void buildPaths() {
        scorePreload = scoreFromPose(startPose);

        grab_a1 = follower.pathBuilder()
                .addPath(approachRow(1))
                .addPath(getBall("a", 1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(180))
                .build();

        score_a1 = scoreFromPose(grab_a1.endPose());

        grab_b1 = follower.pathBuilder()
                .addPath(approachRow(1))
                .addPath(getBall("b", 1))
                .build();

        score_b1 = scoreFromPose(grab_b1.endPose());

        grab_c1 = follower.pathBuilder()
                .addPath(approachRow(1))
                .addPath(getBall("c", 1))
                .build();

        score_c1 = scoreFromPose(grab_c1.endPose());

        moveToLever = new Path(new BezierLine(scorePose, leverPose));
        moveToLever.setLinearHeadingInterpolation(scorePose.getHeading(), leverPose.getHeading());
    }


    private Path scoreFromPose(Pose start) {
        Path p = new Path(new BezierLine(start, scorePose));
        p.setLinearHeadingInterpolation(start.getHeading(), scorePose.getHeading());
        return p;
    }

    private Path approachRow(int row) {
        validateRow(row);

        int rowYPosition = Ball1_YPosition + (BallRowOffset * (row - 1));

        Path p = new Path(new BezierCurve(
                scorePose,
                new Pose(scorePose.getX(), rowYPosition),
                new Pose(BallA_XPosition + OffsetFromBallA_XPosition, rowYPosition)

        ));
        p.setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(180));

        return p;
    }

    private Path getBall(String col, int row) {
        validateRow(row);
        validateCol(col);

        int rowYPosition = Ball1_YPosition + (BallRowOffset * (row - 1));
        int colXPosition = BallA_XPosition - (ballDiameter * (Objects.requireNonNull(COLUMN_TO_NUMBER.get(col)) - 1)); // SILENCE PUNY NPE WARNINGS


        Path p = new Path(new BezierLine(
                new Pose(BallA_XPosition + OffsetFromBallA_XPosition, rowYPosition),
                new Pose(colXPosition, rowYPosition))
        );
        p.setTangentHeadingInterpolation();

        return p;
    }


    private void validateRow(int row) {
        if (row < 1 || row > 3)
            throw new IllegalArgumentException("Row " + row + " is not valid.");
    }

    private void validateCol(String col) {
        if (!COLUMN_TO_NUMBER.containsKey(col))
            throw new IllegalArgumentException("Column " + col + " is not valid.");
    }


    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }


    // found in https://pedropathing.com/docs/pathing/examples/apriltagpatternauto
    // Custom logging function to support telemetry and Panels
    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }




}
