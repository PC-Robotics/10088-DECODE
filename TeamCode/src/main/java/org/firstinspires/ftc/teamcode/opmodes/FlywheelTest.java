package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "FlywheelTest", group = "Test")
public class FlywheelTest extends LinearOpMode {

    // Toggle between PIDF mode and open-loop mode for testing
    private static final boolean USE_PIDF = true;

    // Keep these in sync with your Flywheel presets
    private static final double CLOSE_RPM   = 3000;
    private static final double FAR_RPM     = 5500;
    private static final double CLOSE_POWER = 0.53;
    private static final double FAR_POWER   = 0.90;

    private static final double RPM_STEP   = 50.0;   // per press when using PIDF
    private static final double POWER_STEP = 0.02;   // per press when NOT using PIDF

    private static final double SHOT_LOG_DURATION = 5.0; // seconds

    private Flywheel flywheel;
    private TelemetryManager panelsTelemetry;

    private final ElapsedTime runtime    = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private boolean measuringReady     = false;
    private double  timeToReadySeconds = -1.0;

    // Test-level targets
    private double targetRPM      = CLOSE_RPM;
    private double openLoopPower  = CLOSE_POWER;

    // Shot log snapshot
    private List<String> lastShotLog = new ArrayList<>();
    private double shotLogDisplayUntil = -1.0;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        flywheel = new Flywheel(this, USE_PIDF);
        flywheel.init();

        // Default targets to CLOSE preset
        targetRPM     = CLOSE_RPM;
        openLoopPower = CLOSE_POWER;

        if (USE_PIDF) {
            flywheel.setTargetRPM(targetRPM);
        }

        // INIT loop – optional live monitoring
        while (opModeInInit()) {
            flywheel.update();
            sendTelemetry();
            telemetry.update();
        }

        waitForStart();
        runtime.reset();
        readyTimer.reset();

        while (opModeIsActive()) {
            flywheel.update();

            handleControls();
            handleReadyTiming();

            sendTelemetry();
            telemetry.update();
        }

        flywheel.stop();
    }

    private void handleControls() {
        // Preset selection
        if (gamepad1.triangleWasPressed()) {
            flywheel.setPosition(Flywheel.FLYWHEEL_SPIN_POSITION.FAR);
            targetRPM     = FAR_RPM;
            openLoopPower = FAR_POWER;

            if (USE_PIDF) {
                flywheel.setTargetRPM(targetRPM);
            }
            resetReadyMeasurement();
        } else if (gamepad1.crossWasPressed()) {
            flywheel.setPosition(Flywheel.FLYWHEEL_SPIN_POSITION.CLOSE);
            targetRPM     = CLOSE_RPM;
            openLoopPower = CLOSE_POWER;

            if (USE_PIDF) {
                flywheel.setTargetRPM(targetRPM);
            }
            resetReadyMeasurement();
        }

        // Nudge up/down: RPM (PIDF) or power (open-loop)
        if (gamepad1.dpadRightWasPressed()) {
            if (USE_PIDF) {
                targetRPM += RPM_STEP;
                flywheel.setTargetRPM(targetRPM);
            } else {
                openLoopPower += POWER_STEP;
                openLoopPower = clamp01(openLoopPower);
                // If we’re already spinning, immediately apply new power
                flywheel.spinPower(openLoopPower);
            }
            resetReadyMeasurement();
        } else if (gamepad1.dpadLeftWasPressed()) {
            if (USE_PIDF) {
                targetRPM -= RPM_STEP;
                flywheel.setTargetRPM(targetRPM);
            } else {
                openLoopPower -= POWER_STEP;
                openLoopPower = clamp01(openLoopPower);
                flywheel.spinPower(openLoopPower);
            }
            resetReadyMeasurement();
        }

        // Spin up
        if (gamepad1.dpadUpWasPressed()) {
            if (USE_PIDF) {
                flywheel.spin();              // PIDF-controlled
                flywheel.setTargetRPM(targetRPM); // ensure controller uses nudged target
            } else {
                flywheel.spinPower(openLoopPower); // open-loop, uses nudged power
            }

            readyTimer.reset();
            measuringReady = true;
            timeToReadySeconds = -1.0;
        }

        // Stop
        if (gamepad1.dpadDownWasPressed()) {
            flywheel.stop();
            measuringReady = false;
        }

        // Log a "shot" snapshot
        if (gamepad1.squareWasPressed()) {
            logShotSnapshot();
        }
    }

    private void handleReadyTiming() {
        if (measuringReady && flywheel.isReadyToShoot()) {
            timeToReadySeconds = readyTimer.seconds();
            measuringReady = false;
        }
    }

    private void resetReadyMeasurement() {
        measuringReady = false;
        timeToReadySeconds = -1.0;
        readyTimer.reset();
    }

    private void logShotSnapshot() {
        lastShotLog = new ArrayList<>();

        lastShotLog.add("=== SHOT LOG ===");
        lastShotLog.add("Mode: " + (USE_PIDF ? "PIDF" : "OPEN_LOOP"));
        if (USE_PIDF) {
            lastShotLog.add("Target RPM (test): " +
                    String.format(Locale.ROOT, "%.0f", targetRPM));
        } else {
            lastShotLog.add("Open-loop Power (test): " +
                    String.format(Locale.ROOT, "%.3f", openLoopPower));
        }
        lastShotLog.add("Ready: " + flywheel.isReadyToShoot());
        lastShotLog.add("Time to Ready (s): " +
                (timeToReadySeconds < 0
                        ? "N/A"
                        : String.format(Locale.ROOT, "%.2f", timeToReadySeconds)));

        shotLogDisplayUntil = runtime.seconds() + SHOT_LOG_DURATION;
    }

    private void sendTelemetry() {
        List<String> lines = new ArrayList<>();

        boolean showingShotLog = runtime.seconds() < shotLogDisplayUntil;

        if (showingShotLog && !lastShotLog.isEmpty()) {
            // Show frozen shot info for 5 seconds
            lines.addAll(lastShotLog);
        } else {
            // Live telemetry
            lines.add("=== FLYWHEEL TEST ===");
            lines.add("Mode: " + (USE_PIDF ? "PIDF" : "OPEN_LOOP"));

            if (USE_PIDF) {
                lines.add("Target RPM (test): " +
                        String.format(Locale.ROOT, "%.0f", targetRPM));
            } else {
                lines.add("Open-loop Power (test): " +
                        String.format(Locale.ROOT, "%.3f", openLoopPower));
            }

            lines.add("Time to Ready (s): " +
                    (timeToReadySeconds < 0
                            ? "N/A"
                            : String.format(Locale.ROOT, "%.2f", timeToReadySeconds)));
            lines.add(""); // spacer

            // Append subsystem telemetry
            lines.addAll(flywheel.getTelemetry());
        }

        panelsTelemetry.debug(lines.toArray(new String[0]));
        panelsTelemetry.update(telemetry);

        for (String line : lines) {
            telemetry.addLine(line);
        }
    }

    private double clamp01(double x) {
        return Math.max(0.0, Math.min(1.0, x));
    }
}
