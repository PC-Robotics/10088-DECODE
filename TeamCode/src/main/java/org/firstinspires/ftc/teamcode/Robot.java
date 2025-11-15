package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class Robot {
    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    public DriveBase driveBase;
    public Intake intake;
    public Flywheel flywheel;

    // pedro
    public Follower follower;
    public static Alliance alliance = Alliance.BLUE;

    public Pose currentPose;
    public static Pose endPose; // static variables are saved between auto and teleop so this variable helps us do that
    public static Pose scorePose = new Pose(56, 18, Math.toRadians(315));

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opMode) {
        this.myOpMode = opMode;
        driveBase = new DriveBase(myOpMode);
        intake = new Intake(myOpMode);
        flywheel = new Flywheel(myOpMode);
        currentPose = null;
        endPose = null;
    }

    // initialize (main function)
    public void init() {
        follower = Constants.createFollower(myOpMode.hardwareMap);
        driveBase.init();
        intake.init();
        flywheel.init();
    }

    public void update() {
        follower.update();
        currentPose = follower.getPose();
        intake.update();
    }

    public void stop() {
        endPose = follower.getPose();
    }

    public void setAlliance(Alliance alliance) {
        if (Robot.alliance != alliance) {
            scorePose = scorePose.mirror();
        }

        Robot.alliance = alliance;
    }

    /*
     * if flywheel is spinning, then yes
     * if flywheel is not spinning and no ball detected, then yes
     * if flywheel is not spinning and ball detected then no
     */
    public boolean canIntake() {
        if (flywheel.flywheelState == Flywheel.FLYWHEEL_STATE.SPINNING) {
            return true;
        } else {
            return !intake.detectingBall;
        }
    }
}