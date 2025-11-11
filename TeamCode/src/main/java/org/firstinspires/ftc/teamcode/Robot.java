package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/*
 * Control -
 *      Servo -
 *          0: wrist
 *          2: (CR) intake
 *          5: wrist
 *      Motors -
 *          0: backRight(new)
 *          1: frontRight
 *          2: backLeft(new)
 *          3: frontLeft(new)
 * Expansion -
 *      Servo -
 *          0: right
 *          2: left
 *          4: claw
 *      Motors -
 *          0: linearSlide
 */
public class Robot {
    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
    private final Timer loopTimer;

    public DriveBase driveBase;
    public Intake intake;
    public Flywheel flywheel;

    // pedro
    public Follower follower;
    public Alliance alliance = Alliance.BLUE;

    public Pose currentPose;
    public static Pose endPose; // static variables are saved between auto and teleop so this variable helps us do that
    public static Pose scorePose = new Pose(56, 18, Math.toRadians(298));

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opMode) {
        this.myOpMode = opMode;
        driveBase = new DriveBase(myOpMode);
        intake = new Intake(myOpMode);
        flywheel = new Flywheel(myOpMode);
        follower = Constants.createFollower(opMode.hardwareMap);
        loopTimer = new Timer();

        loopTimer.resetTimer();
    }

    // initialize (main function)
    public void init() {
        driveBase.init();
        intake.init();
        flywheel.init();
    }

    public void update() {
        follower.update();
        currentPose = follower.getPose();
    }

    public void stop() {
        endPose = follower.getPose();
    }

    public void setAlliance(Alliance alliance) {
        if (this.alliance != alliance) {
            scorePose = scorePose.mirror();
        }

        this.alliance = alliance;
    }
}