package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HardwareUtility.motorInit;
import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.lerp;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;


public class Flywheel implements Subsystem {
    public enum FLYWHEEL_STATE {
        IDLE,
        SPINNING
    }

    public enum FLYWHEEL_SPIN_POSITION {
        CLOSE(3000, 0.53),
        FAR(5500, 0.90);

        final int rpm;
        final double power;
        FLYWHEEL_SPIN_POSITION(int rpm, double power) {
            this.rpm = rpm;
            this.power = power;
        }
    }

    public enum LED_COLOR {
        OFF(0.0),
        GREEN(0.5),
        RED(0.29),
        YELLOW(0.388),
        BLUE(0.65),
        WHITE(1.0);

        final double value;

        LED_COLOR(double value) {
            this.value = value;
        }
    }


    private static final double fifteenOverPI = 15 / Math.PI;

    private static final double READY_RPM_TOLERANCE = 75;
    private static final double READY_DERIVATIVE_TOLERANCE = 50;
    private static final int READY_CYCLES_REQUIRED = 5;

    private static final double LOOKUP_TABLE_STEPS = 5.0;
    private static final double MIN_RPM = 500.0;
    private static final double MAX_RPM = 6000.0;
    private static final double MIN_DISTANCE = 0;
    private static final double MAX_DISTANCE = 200.0;

    private LinearOpMode opMode;

    private DcMotorEx flywheelLeft;
    private DcMotorEx flywheelRight;
    private ServoImplEx light; // this is controlled like a servo using setPosition()

    private final boolean runPIDF;
    private double currentRPM;
    private double power;

    private boolean readyToShoot = false;
    private int readyCycles = 0;

    private double distanceToGoal = 0.0;

    private FLYWHEEL_STATE flywheelState = FLYWHEEL_STATE.IDLE;
    private FLYWHEEL_SPIN_POSITION spinPosition = FLYWHEEL_SPIN_POSITION.CLOSE;
    private TreeMap<Double, Double> distanceToRPM;

    private final PIDFController controller;

    // TODO - tune this
    private static final PIDFCoefficients coefficients = new PIDFCoefficients(
            0.0005,
            0,
            0,
            1.0
    );


    public Flywheel(LinearOpMode opMode, boolean runPIDF) {
        this.opMode = opMode;
        this.runPIDF = runPIDF;
        controller = new PIDFController(coefficients);
        if (runPIDF) {
            controller.setTargetPosition(spinPosition.rpm);
        }

        for (double i = MIN_DISTANCE; i <= MAX_DISTANCE; i += LOOKUP_TABLE_STEPS) {
            distanceToRPM.put(i, getRPM(i));
        }
    }


    @Override
    public void init() {
        DcMotor.RunMode runMode = runPIDF ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : DcMotor.RunMode.RUN_USING_ENCODER;
        flywheelLeft = motorInit(
                opMode.hardwareMap,
                "flywheelleft",
                DcMotorSimple.Direction.FORWARD,
                runMode
        );
        flywheelRight = motorInit(
                opMode.hardwareMap,
                "flywheelright",
                DcMotorSimple.Direction.REVERSE,
                runMode
        );

        flywheelLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        light = opMode.hardwareMap.get(ServoImplEx.class, "light");
        light.setPosition(LED_COLOR.OFF.value);
    }


    @Override
    public void update() {
        currentRPM = getRPM();

        if (flywheelState != FLYWHEEL_STATE.SPINNING) {
            readyToShoot = false;
            readyCycles = 0;
            return;
        }

        if (runPIDF) {
            controller.setTargetPosition(lookupRPM(distanceToGoal));
            controller.updatePosition(currentRPM);
            controller.updateFeedForwardInput(spinPosition.power);

            power = clamp(controller.run(), -1.0, 1.0);
            flywheelLeft.setPower(power);
            flywheelRight.setPower(power);

            updateReadyToShootPIDF();
        } else {
            updateReadyToShootSimple();
        }

        light.setPosition(readyToShoot ? LED_COLOR.GREEN.value : LED_COLOR.RED.value);
    }


    private void updateReadyToShootPIDF() {
        if (
                Math.abs(controller.getError()) <= READY_RPM_TOLERANCE &&
                Math.abs(controller.getErrorDerivative()) <= READY_DERIVATIVE_TOLERANCE &&
                ++readyCycles >= READY_CYCLES_REQUIRED // ++readyCycles adds 1 to readyCycles then compares
        ) {
            readyToShoot = true;
        } else {
            readyToShoot = false;
            readyCycles = 0;
        }
    }


    private void updateReadyToShootSimple() {
        if (
                Math.abs(spinPosition.rpm - currentRPM) <= READY_RPM_TOLERANCE &&
                ++readyCycles >= READY_CYCLES_REQUIRED
        ) {
            readyToShoot = true;
        } else {
            readyToShoot = false;
            readyCycles = 0;
        }
    }


    public void setPosition(FLYWHEEL_SPIN_POSITION position) {
        // if we try and set the position to the one we already have
        if (position == spinPosition) {
            return;
        }

        spinPosition = position;

        if (runPIDF) {
            controller.reset();
            controller.setTargetPosition(position.rpm);
        } else if (flywheelState == FLYWHEEL_STATE.SPINNING) {
            spinPower();
        }
    }


    public void spin() {
        if (flywheelState == FLYWHEEL_STATE.IDLE && runPIDF) {
            controller.reset();
            controller.setTargetPosition(spinPosition.rpm);
            readyToShoot = false;
            readyCycles = 0;
        }

        flywheelState = FLYWHEEL_STATE.SPINNING;

        if (!runPIDF) {
            spinPower();
        }
    }



    // fallback if pidf style doesnt work
    public void spinPower(double power) {
        flywheelState = FLYWHEEL_STATE.SPINNING;
        this.power = clamp(power, -1.0, 1.0);
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }


    public void spinPower() {
        spinPower(spinPosition.power);
    }


    public double getRPM() {
        // Avg Angular Velocity = ((flywheelL + flywheelR) / 2)
        // RPM = Avg Angular Velocity * (60 / 2pi)
        return (
                flywheelLeft.getVelocity(AngleUnit.RADIANS) +
                flywheelRight.getVelocity(AngleUnit.RADIANS)
        ) * fifteenOverPI;
    }


    // for testing purposes
    public void setTargetRPM(double rpm) {
        if (runPIDF) {
            controller.setTargetPosition(rpm);
        }
    }


    @Override
    public void stop() {
        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
        flywheelState = FLYWHEEL_STATE.IDLE;
        controller.reset();
        readyToShoot = false;
        readyCycles = 0;
        power = 0.0;
        light.setPosition(LED_COLOR.OFF.value);
    }


    private static double getRPM(double distance) {
        return clamp(Math.pow(distance, 1.6064) * 1.78533, MIN_RPM, MAX_RPM);
    }


    private double lookupRPM(double distance) {
        Map.Entry<Double, Double> lowerEntry = distanceToRPM.floorEntry(distance);
        Map.Entry<Double, Double> upperEntry = distanceToRPM.ceilingEntry(distance);

        if (lowerEntry == null) {
            return distanceToRPM.firstEntry().getValue();
        }
        if (upperEntry == null) {
            return distanceToRPM.lastEntry().getValue();
        }

        return lerp(
                lowerEntry.getValue(),
                upperEntry.getValue(),
                (distance - lowerEntry.getKey()) / (upperEntry.getKey() - lowerEntry.getKey())
        );
    }


    @Override
    public List<String> getTelemetry() {
        double avgPower = (flywheelLeft.getPower() + flywheelRight.getPower()) * 0.5;

        List<String> out = new java.util.ArrayList<>();
        out.add("State: " + flywheelState);
        out.add("Preset: " + spinPosition);
        out.add("Mode: " + (runPIDF ? "PIDF" : "OPEN_LOOP"));
        out.add("Ready: " + readyToShoot);
        out.add("Current RPM: " + String.format(Locale.ROOT, "%.0f", currentRPM));
        out.add("Average Power: " + String.format(Locale.ROOT, "%.3f", avgPower));

        if (runPIDF) {
            out.add("Target RPM: " + controller.getTargetPosition());
            out.add("PIDF Output: " + String.format(Locale.ROOT, "%.2f", power));
        }

        return out;
    }


    public boolean isReadyToShoot() {
        return readyToShoot;
    }


    public FLYWHEEL_STATE getState() {
        return flywheelState;
    }


    public FLYWHEEL_SPIN_POSITION getSpinPosition() {
        return spinPosition;
    }


    public void setDistanceToGoal(double distanceToGoal) {
        this.distanceToGoal = distanceToGoal;
    }
}