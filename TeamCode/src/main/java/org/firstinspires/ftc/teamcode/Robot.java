package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public class Robot {
    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    public DriveBase driveBase;
    public Intake intake;
    public Flywheel flywheel;
    public Transfer transfer;

    // pedro
    public Follower follower;
    public static Alliance alliance = Alliance.BLUE;

    public Pose currentPose;
    public static Pose endPose; // static variables are saved between auto and teleop so this variable helps us do that
    public static Pose farScorePose = new Pose(56, 18, Math.toRadians(315));
    public static Pose closeScorePose = new Pose(60, 84, Math.toRadians(315));

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opMode) {
        this.myOpMode = opMode;
        driveBase = new DriveBase(myOpMode);
        intake = new Intake(myOpMode);
        flywheel = new Flywheel(myOpMode);
        transfer = new Transfer(myOpMode);
        currentPose = null;
    }

    // initialize (main function)
    public void init() {
        follower = Constants.createFollower(myOpMode.hardwareMap);
        driveBase.init();
        intake.init();
        flywheel.init();
        transfer.init();
    }

    public void update() {
        follower.update();
        currentPose = follower.getPose();

        driveBase.update();
        intake.update();
        flywheel.update();
        transfer.update();
    }


    public void stop() {
        endPose = follower.getPose();
    }

    public void setAlliance(Alliance alliance) {
        if (Robot.alliance != alliance) {
            farScorePose = farScorePose.mirror();
            closeScorePose = closeScorePose.mirror();
        }

        Robot.alliance = alliance;
    }
}