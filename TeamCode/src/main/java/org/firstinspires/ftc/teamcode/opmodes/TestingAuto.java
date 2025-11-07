// start with the line on the middle

package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Testing Auto", group = "Main")
public class TestingAuto extends LinearOpMode {
    Robot robot = new Robot(this);

    public void runOpMode() {
        robot.init();
        robot.imu.resetYaw();
        waitForStart();

        robot.driveDistance(24, 10);
    }

}