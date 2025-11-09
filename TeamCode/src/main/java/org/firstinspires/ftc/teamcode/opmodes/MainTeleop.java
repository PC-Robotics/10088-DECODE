package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Utility.applyDeadzone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Settings;

/**
 * Controller 1 - Driver
 * joysticks - mecanum drive
 * bumper left - fine control (half sped)
 * options - reset heading
 * trigger left - trolley go in (values wrong)
 * trigger right - trolley go out
 * square - claw close (set to bucket now)
 * circle - claw open
 * <p>
 * Controller 2 - Operator
 * bumper left - wrist go "in" (not working well)
 * bumper right - wrist go "out"
 * trigger right - intake spin
 * trigger left - intake spin backwards
 * triangle - bucket release (not working)
 * cross - bucket pickup (slow)
 * dpadUp - slide index up (not working)
 * dpadDown - slide index down
 * left joystick y - slide up down manual control
 */

@TeleOp(name = "MainTeleop", group = "Main")
public class MainTeleop extends LinearOpMode {
    Robot robot = new Robot(this);

    private double straight, turn, strafe, heading;

    private double gamepad1LeftTrigger, gamepad1RightTrigger, gamepad2LeftTrigger, gamepad2RightTrigger, gamepad2RightJoystickY;
    private boolean gamepad2LeftBumper = false, gamepad2RightBumper = false, gamepad2DpadRight = false, gamepad2DpadLeft = false;

    public void runOpMode() {
        robot.init();
        waitForStart();

        startingPositions();

        while (opModeIsActive()) { // hi

            readController();
            readSensors();

            robot.driveBase.mecanumDrive(straight, strafe, turn, gamepad1.left_bumper);
            flywheelControl();
            intakeControl();

            reloadTelemetry();
            telemetry.update();
        }
    }

    private void startingPositions() {
    }

    public void readController() {
        straight = -applyDeadzone(gamepad1.left_stick_y, Settings.DEADZONE_THRESHOLD);
        strafe = applyDeadzone(gamepad1.left_stick_x, Settings.DEADZONE_THRESHOLD);
        turn = applyDeadzone(gamepad1.right_stick_x, Settings.DEADZONE_THRESHOLD);

        gamepad1RightTrigger = applyDeadzone(gamepad1.right_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad1LeftTrigger = applyDeadzone(gamepad1.left_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad2LeftTrigger = applyDeadzone(gamepad2.left_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad2RightTrigger = applyDeadzone(gamepad2.right_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad2RightJoystickY = -applyDeadzone(gamepad2.right_stick_y, Settings.DEADZONE_THRESHOLD);
    }

    private void readSensors() {

    }


    private void flywheelControl() {
        if (gamepad2.dpad_up) {
            robot.flywheel.spinToSpeed();
        } else if (gamepad2.dpad_down) {
            robot.flywheel.stop();
        }
    }

    private void intakeControl() {
        if (gamepad2.right_bumper) {
            robot.intake.outtake();
        } else if (gamepad2RightTrigger != 0) {
            robot.intake.intake(gamepad2RightTrigger);
        } else {
            robot.intake.stop();
        }
    }


    private void reloadTelemetry() {
        // robot.intake.telemetry();

        telemetry.addData("DRIVE BASE", "-----------");
        robot.driveBase.telemetry();

        telemetry.update();
    }
}
