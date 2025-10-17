package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Simplistic drive base capable of using mecanum wheels and driving around
 */
public class DriveBase
{
    protected LinearOpMode myOpMode = null;

    protected boolean fieldCentric = false;

    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftRearDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightRearDrive = null;

    protected IMU imu = null;

    public DriveBase (LinearOpMode opMode, boolean isFC)
    {
        myOpMode = opMode;
        fieldCentric = isFC;
    }

    /**
     * Initializes the motors and sets their direction. Can also add other default settings.
     *  - Set Mode: run with/without encoders
     *  - Zero power behavior
     */
    public void init()
    {
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        leftRearDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class,"rightFront");
        rightRearDrive = myOpMode.hardwareMap.get(DcMotor.class,"rightRear");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        // TODO: Update this based on how the hub is mounted on the robot
        imu = myOpMode.hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        myOpMode.telemetry.addData("Status","Initialized");
        myOpMode.telemetry.update();

    }

    /**
     * Standard POV Mecanum drive code
     * @param axial left joystick y value
     * @param lateral left joystick x value
     * @param yaw right joystick x value
     */
    public void drive(double axial, double lateral, double yaw)
    {
        double max;

        // This conversion is based on gmZero.org code
        if(fieldCentric)
        {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
            axial = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

            lateral = rotX;
        }

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double denominator = Math.max(Math.abs(axial)+Math.abs(lateral)+Math.abs(yaw),1);
        double leftFrontPower  = (axial + lateral + yaw)/denominator;
        double leftRearPower   = (axial - lateral + yaw)/denominator;
        double rightFrontPower = (axial - lateral - yaw)/denominator;
        double rightRearPower  = (axial + lateral - yaw)/denominator;

        // Apply the power to the motors
        leftFrontDrive.setPower(leftFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightRearDrive.setPower(rightRearPower);

        // Add data to the telemetry for displaying the current motor powers
        myOpMode.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        myOpMode.telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftRearPower, rightRearPower);
        myOpMode.telemetry.addData("Heading",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
}
