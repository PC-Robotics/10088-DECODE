package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.motorInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Flywheel implements Subsystem {
    public enum FLYWHEEL_STATE {
        IDLE,
        SPINNING
    }

    public enum FLYWHEEL_SPIN_POSITION {
        CLOSE(0.60),
        FAR(0.95);

        final double power;
        FLYWHEEL_SPIN_POSITION(double power) {
            this.power = power;
        }
    }

    private LinearOpMode opMode;

    public DcMotorEx flywheelLeft;
    public DcMotorEx flywheelRight;

    public FLYWHEEL_STATE flywheelState = FLYWHEEL_STATE.IDLE;
    public FLYWHEEL_SPIN_POSITION spinPosition = FLYWHEEL_SPIN_POSITION.FAR;


    public Flywheel(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init() {
        flywheelLeft = motorInit(opMode.hardwareMap, "flywheelleft", DcMotorSimple.Direction.REVERSE);
        flywheelRight = motorInit(opMode.hardwareMap, "flywheelright", DcMotorSimple.Direction.FORWARD);

        flywheelLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void setPosition(FLYWHEEL_SPIN_POSITION position) {
        if (position != spinPosition) {
            spinPosition = position;

            if (flywheelState == FLYWHEEL_STATE.SPINNING) {
                spinToSpeed();
            }
        }
    }

    public void spinToSpeed(double power) {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
        flywheelState = FLYWHEEL_STATE.SPINNING;
    }

    public void spinToSpeed() {
        spinToSpeed(spinPosition.power);
    }

    @Override
    public String[] getTelemetry() {
        return new String[] {
                "Flywheel State: " + flywheelState,
                "Flywheel Position: " + spinPosition,
                "Flywheel Power: " +
                        String.format("%.2f", (flywheelLeft.getPower() + flywheelRight.getPower()) / 2.0)
        };
    }
}
