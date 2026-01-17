package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.4) // kg
            .forwardZeroPowerAcceleration(-30.33758730435513)
            .lateralZeroPowerAcceleration(-53.48519897968198)
            /* tune PID values: https://pedropathing.com/docs/pathing/tuning/pids
            * https://www.youtube.com/watch?v=vihb2LPtSK0
            * if one PID doesnt work then tune secondary PID system.
             */

            /*
             * Tune F first:
             * Find value of F such that you head coil whine but no robot shaking
             * Give it a high value first like .07 then a low value like .01
             * Remember to press the "refresh" button on Panels
             * Slowly search for the position that gives you the loudest coil whine without actually shaking the robot
             * To test it you can push the robot and see if it shakes at all. if so, it's too high.
             *
             * Tune P next:
             * Find value of P such that it corrects back to its starting position without overcorrecting too much
             * Place the robot such that it's center is on a known point. Set P to a really high value like 5.
             * Push the robot laterally and observe if it overcorrects and oscillates. This is too high
             * Then set P to something super low like 0.1. Notice how the robot, when pushed, doesn't correct at all or it corrects very slow.
             * Search for a value that corrects quickly without overshoot.
             *
             * Tune D next:
             * D helps the P not overshoot but if D is too high the robot moves too slow
             * Set D to something super high like 0.2 and watch the robot correct super slow.
             * Set D to something super low like 0.0001 and watch the robot overshoot
             * Find the D value such that the robot can move back to its original point quickly but minimizes overshoots.
             * D SHOULD (might not) be 0.0x
             *
             *
             */
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.07,
                    0,
                    0.01,
                    0.033
            ))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.2,
                    0,
                    0.01,
                    0.01
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    2,
                    0,
                    0.08,
                    0.03
            ))
            // TODO - FIX DRIVE COEFFICIENTS
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                            .015,
                    0,
                    0.01,
                    0.6,
                    0.33
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    .02,
                    0,
                    0.000005,
                    0.6,
                    0.1
            ))

            // TODO - TUNE THESE
//            .translationalPIDFSwitch(4)
//            .drivePIDFSwitch(15)
//            .centripetalScaling(0.0005)
            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontright")
            .rightRearMotorName("backright")
            .leftRearMotorName("backleft")
            .leftFrontMotorName("frontleft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(65.10661351962354)
            .yVelocity(53.372667207492626);


    public static TwoWheelConstants twoWheelLocalizerConstants = new TwoWheelConstants()
            .forwardPodY(0)
            .strafePodX(0)
            .forwardEncoder_HardwareMapName("backleft")
            .strafeEncoder_HardwareMapName("frontleft")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            )
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(.00201)
            .strafeTicksToInches(.00195);


    public static PinpointConstants pinpointLocalizerConstants = new PinpointConstants()
            .forwardPodY(6.14)// 6.14
            .strafePodX(-6.61)// -6.61
            .distanceUnit(DistanceUnit.INCH)//changed from inches to MM
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.25, // TODO - TUNE THIS
            10,
            1 // TODO - TUNE THIS
    );


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(pinpointLocalizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
