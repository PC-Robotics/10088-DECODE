// start with the line on the middle

package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Auto", group = "Main", preselectTeleOp = "MainTeleop")
public class Auto extends LinearOpMode {
    Robot robot = new Robot(this);

    public void runOpMode() {
        robot.init();
        robot.imu.resetYaw();
        waitForStart();

        // start on the back field,
        robot.flywheel.spinToSpeed(.65);

        // drive to shooting position
        robot.driveDistance(12);
        robot.turnAbsolute(-24.444); // -(90 - tan^-1(5.5/2.5))
        sleep(1000); // wait for spin up
        robot.intake.intake(); // release one ball
        sleep(200); // wait for release
        robot.intake.stop(); // stop
        sleep(500); // wait for flywheel to reload
        robot.intake.intake(); // release second ball
        sleep(500); // wait for release

        robot.turnAbsolute(0);
        robot.driveDistance(24);
        robot.turnAbsolute(-90);
        robot.driveDistance(3 + 24 - 4);
        sleep(200);
        robot.intake.stop();
        robot.driveDistance(-(3 + 24 - 4));
        robot.turnAbsolute(0);
        robot.driveDistance(-24);

        // back to shooting point
        robot.intake.intake(); // release third ball
        sleep(500); // wait for release

        // get ball number 4
        robot.turnAbsolute(0);
        robot.driveDistance(24);
        robot.turnAbsolute(-90);
        robot.driveDistance(3 + 24 - 4 + 5);
        sleep(200);
        robot.intake.stop();
        robot.driveDistance(-(3 + 24 - 4 + 5));
        robot.turnAbsolute(0);
        robot.driveDistance(-24);

        // back to shooting point
        robot.intake.intake(); // release 4th ball
        sleep(500); // wait for release

        // get ball number 5
        robot.turnAbsolute(0);
        robot.driveDistance(24);
        robot.turnAbsolute(-90);
        robot.driveDistance(3 + 24 - 4 + 5 + 5);
        sleep(200);
        robot.intake.stop();
        robot.driveDistance(-(3 + 24 - 4 + 5 + 5));
        robot.turnAbsolute(0);
        robot.driveDistance(-24);

        // back to shooting point
        robot.intake.intake(); // release 5th ball
        sleep(500); // wait for release

        robot.flywheel.stop();
        robot.turnAbsolute(90);
    }

}