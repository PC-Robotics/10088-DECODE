package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.motorInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Flywheel implements Subsystem {
    public enum FLYWHEEL_STATE {
        IDLE,
        SPINNING
    }

    private LinearOpMode opMode;

    public DcMotorEx flywheelLeft;
    public DcMotorEx flywheelRight;

    public FLYWHEEL_STATE state;

    public Flywheel(LinearOpMode opMode) {
        this.opMode = opMode;
        state = FLYWHEEL_STATE.IDLE;
    }

    @Override
    public void init() {
        flywheelLeft = motorInit(opMode.hardwareMap, "flywheelleft", DcMotorSimple.Direction.REVERSE);
        flywheelRight = motorInit(opMode.hardwareMap, "flywheelright", DcMotorSimple.Direction.FORWARD);

        flywheelLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void spinToSpeed(double power) {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
        state = FLYWHEEL_STATE.SPINNING;
    }

    public void spinToSpeed() {
        spinToSpeed(.65);
    }

    @Override
    public void stop() {
        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
        state = FLYWHEEL_STATE.IDLE;
    }

    @Override
    public String[] getTelemetry() {
        return new String[] {
                "Flywheel State: " + state.toString(),
                "Flywheel Power: " + (flywheelLeft.getPower() + flywheelRight.getPower())/2.0
        };
    }
}
