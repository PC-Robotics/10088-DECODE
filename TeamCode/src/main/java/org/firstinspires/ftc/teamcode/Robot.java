package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    public ElapsedTime timer = new ElapsedTime();

    public DriveBase driveBase;
    public Intake intake;
    public Flywheel flywheel;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opMode) {
        this.myOpMode = opMode;
        driveBase = new DriveBase(myOpMode);
        intake = new Intake(myOpMode);
        flywheel = new Flywheel(myOpMode);
    }

    // initialize (main function)
    public void init() {
        driveBase.init();
        intake.init();
        flywheel.init();
    }
}