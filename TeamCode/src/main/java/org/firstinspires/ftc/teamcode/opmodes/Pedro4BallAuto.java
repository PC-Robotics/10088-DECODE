// to view the path, upload Pedro4BallAutoBlue.pp into https://visualizer.pedropathing.com/

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
    private PathChain scorePreload, score_a1, score_b1, score_c1, moveToLever, grab_a1, grab_b1, grab_c1;

    // tracks the current state of our autonomous
    private AutonomousState pathState = AutonomousState.SCORE_PRELOAD;

    // fixed poses. you can check https://visualizer.pedropathing.com/. Center of robot is tracking point
    private static Pose startPose = new Pose(56, 9, Math.toRadians(270));
    private static Pose scorePose = new Pose(56, 18, Math.toRadians(298));
    private static Pose row1ApproachPose = new Pose(46, 36, Math.toRadians(180));
    private static Pose row1ApproachControlPoint = new Pose(56, 36);
    private static Pose grab_a1_Pose = new Pose(36, 36, Math.toRadians(180));
    private static Pose grab_b1_Pose = new Pose(31, 36, Math.toRadians(180));
    private static Pose grab_c1_Pose = new Pose(26, 36, Math.toRadians(180));
    private static Pose leverPose = new Pose(22, 72, Math.toRadians(270));
    
    
    @Override
    public void runOpMode() {
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
            case SCORE_PRELOAD:
                if (robot.flywheel.state == Flywheel.FLYWHEEL_STATE.IDLE) {
                    robot.flywheel.spinToSpeed();
                }
                sleep(2000);
                follower.followPath(scorePreload); // holds position
                setPathState(AutonomousState.GRAB_A1);
                break;

            case GRAB_A1:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                // TODO - the robots
                sleep(500);
                robot.flywheel.stop();

                follower.followPath(grab_a1, false);
                setPathState(AutonomousState.SCORE_A1);
                break;

            case SCORE_A1:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                sleep(200);
                robot.intake.stop();
                if (robot.flywheel.state == Flywheel.FLYWHEEL_STATE.IDLE) {
                    robot.flywheel.spinToSpeed();
                }
                follower.followPath(score_a1);
                setPathState(AutonomousState.GRAB_B1);
                break;

            case GRAB_B1:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                sleep(500);
                robot.flywheel.stop();

                follower.followPath(grab_b1, false);
                setPathState(AutonomousState.SCORE_B1);
                break;

            case SCORE_B1:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                sleep(200);
                robot.intake.stop();
                if (robot.flywheel.state == Flywheel.FLYWHEEL_STATE.IDLE) {
                    robot.flywheel.spinToSpeed();
                }
                follower.followPath(score_b1);
                setPathState(AutonomousState.GRAB_C1);
                break;

            case GRAB_C1:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                sleep(500);
                robot.flywheel.stop();

                follower.followPath(grab_c1, false);
                setPathState(AutonomousState.SCORE_C1);
                break;

            case SCORE_C1:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                sleep(200);
                robot.intake.stop();
                if (robot.flywheel.state == Flywheel.FLYWHEEL_STATE.IDLE) {
                    robot.flywheel.spinToSpeed();
                }
                follower.followPath(score_c1);
                setPathState(AutonomousState.MOVE_TO_LEVER);
                break;

            case MOVE_TO_LEVER:
                if (follower.isBusy()) { // still arriving to point
                    break;
                }

                robot.intake.intake();
                sleep(500);
                robot.intake.stop();
                robot.flywheel.stop();

                follower.followPath(moveToLever);
                setPathState(AutonomousState.DEFAULT); // default
                break;

            case DEFAULT:
                break;
        }
    }


    private void buildPaths() {
        scorePreload = scoreFromPose(startPose);
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grab_a1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, row1ApproachControlPoint, row1ApproachPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), row1ApproachPose.getHeading())
                .addPath(new BezierLine(row1ApproachPose, grab_a1_Pose))
                .setTangentHeadingInterpolation()
                .build();

        score_a1 = follower.pathBuilder()
                .addPath(new BezierLine(grab_a1_Pose, scorePose))
                .setLinearHeadingInterpolation(grab_a1_Pose.getHeading(), scorePose.getHeading())
                .build();

        grab_b1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, row1ApproachControlPoint, row1ApproachPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), row1ApproachPose.getHeading())
                .addPath(new BezierLine(row1ApproachPose, grab_b1_Pose))
                .setTangentHeadingInterpolation()
                .build();

        score_b1 = follower.pathBuilder()
                .addPath(new BezierLine(grab_b1_Pose, scorePose))
                .setLinearHeadingInterpolation(grab_b1_Pose.getHeading(), scorePose.getHeading())
                .build();

        grab_c1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, row1ApproachControlPoint, row1ApproachPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), row1ApproachPose.getHeading())
                .addPath(new BezierLine(row1ApproachPose, grab_c1_Pose))
                .setTangentHeadingInterpolation()
                .build();

        score_c1 = follower.pathBuilder()
                .addPath(new BezierLine(grab_c1_Pose, scorePose))
                .setLinearHeadingInterpolation(grab_c1_Pose.getHeading(), scorePose.getHeading())
                .build();

        moveToLever = follower.pathBuilder()
                .addPath(new Path(new BezierLine(scorePose, leverPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leverPose.getHeading())
                .build();
    }


/* these arent used because i've decided it's easier to originally debug and test with explicit values
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
*/

    private void validateRow(int row) {
        if (row < 1 || row > 3)
            throw new IllegalArgumentException("Row " + row + " is not valid.");
    }

    private void validateCol(String col) {
        if (!COLUMN_TO_NUMBER.containsKey(col))
            throw new IllegalArgumentException("Column " + col + " is not valid.");
    }


    public void setPathState(AutonomousState pathState) {
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

    public enum AutonomousState {
        SCORE_PRELOAD,
        GRAB_A1,
        SCORE_A1,
        GRAB_B1,
        SCORE_B1,
        GRAB_C1,
        SCORE_C1,
        MOVE_TO_LEVER,
        DEFAULT
    }
}
