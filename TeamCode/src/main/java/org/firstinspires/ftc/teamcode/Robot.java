package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.List;

public class Robot {
    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    public DriveBase driveBase;
    public Intake intake;
    public Flywheel flywheel;
    public Transfer transfer;
    private List<Subsystem> subsystems = List.of(
            driveBase,
            intake,
            flywheel,
            transfer
    );

    // pedro
    public Follower follower;
    public static Alliance alliance = Alliance.BLUE;

    public Pose currentPose;
    public static Pose endPose; // static variables are saved between auto and teleop so this variable helps us do that
    public static Pose scorePose = new Pose(56, 18, Math.toRadians(315));
    public static Pose goalPose = new Pose(0, 144);

    public double distanceToGoal;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opMode) {
        this.myOpMode = opMode;
        driveBase = new DriveBase(myOpMode);
        intake = new Intake(myOpMode);
        flywheel = new Flywheel(myOpMode, true);
        transfer = new Transfer(myOpMode);
        currentPose = null;
    }

    // initialize (main function)
    public void init() {
        follower = Constants.createFollower(myOpMode.hardwareMap);
        for (Subsystem s : subsystems) {
            s.init();
        }
    }

    public void update() {
        follower.update();
        currentPose = follower.getPose();
        updateGoalPose();


        flywheel.setDistanceToGoal(distanceToGoal);
        for (Subsystem s : subsystems) {
            s.update();
        }
    }


    private void updateGoalPose() {
        distanceToGoal = currentPose.distanceFrom(Robot.goalPose);
    }


    public void stop() {
        endPose = follower.getPose();
        for (Subsystem s : subsystems) {
            s.stop();
        }
    }

    public void setAlliance(Alliance alliance) {
        if (Robot.alliance != alliance) {
            scorePose = scorePose.mirror();
            goalPose = goalPose.mirror();
        }

        Robot.alliance = alliance;
    }
}