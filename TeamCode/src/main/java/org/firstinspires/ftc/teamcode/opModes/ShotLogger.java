package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

/**
 * Measures what a real shot does to each flywheel. Load balls, hold the shoot button, and it records every shot
 * on its own: how far the speed drops, how long it takes to get back, and how much it overshoots.
 *
 * Controls (gamepad1): right bumper or right trigger = shoot (flywheels are always spinning), left bumper = intake,
 * square = reverse, triangle = hold gates open, D-pad up/down = target speed +/- 25 (both wheels),
 * Cross = clear the stats.
 *
 * A shot starts when a wheel that was holding its target drops more than DROP_TRIGGER ticks/s below it.
 * The shot ends once the speed has stayed within BAND_PCT of the target for SETTLE_MS.
 * Uses whatever PIDF is in Shooter.java, so it also tests the values the tuner gave you.
 * Results are on the Driver Station and in /sdcard/FIRST/shot_logger_events_*.csv (one row per shot) and
 * shot_logger_raw_*.csv (every speed reading).
 */
@TeleOp(name = "Shot Logger", group = "Tuning")
public class ShotLogger extends LinearOpMode {
    // Driver Station and Panels (laptop) gamepads combined: read pad1 / pad2 instead of gamepad1 / gamepad2
    private final org.firstinspires.ftc.teamcode.subsystems.LaptopInput input = new org.firstinspires.ftc.teamcode.subsystems.LaptopInput();
    private com.qualcomm.robotcore.hardware.Gamepad pad1;
    private com.qualcomm.robotcore.hardware.Gamepad pad2;
    static final double LOGGER_SPEED = 1350;       // the flywheel speed this logger runs at (ticks/s); D-pad changes it by 25
    static final double DROP_TRIGGER = 50;       // a shot starts when the speed falls this far below the target (ticks/s)
    static final double ARM_BAND_PCT = 3.0;      // the wheel must hold within this % of the target...
    static final double ARM_MS = 300;            // ...for this long before it can count a shot
    static final double BAND_PCT = 1.5;          // recovered = within this % of the target
    static final double SETTLE_MS = 300;         // ...and staying there this long
    static final double MAX_EVENT_MS = 3000;     // give up on a shot that hasn't recovered by then
    static final double SMOOTH_MS = 40;          // readings are averaged over this long before judging them
    static final double PRE_MS = 300;            // how much speed history is kept from before the shot starts

    private static class Shot {
        int number;
        double target, pre, low, timeToLowMs, recoveryMs, overshootPct;
        int balls;
        boolean recovered;

        double dipPct() {
            return (target - low) / target * 100;
        }
    }

    private File eventsCsv;
    private File rawCsv;
    private int shotCount = 0;

    /** Watches one flywheel's speed and picks the shots out of it. */
    private class Watch {
        final String name;
        double target;
        final ArrayDeque<double[]> recent = new ArrayDeque<>();   // {time, speed} for the last PRE_MS
        final List<Shot> shots = new ArrayList<>();
        boolean armed = false;
        double inArmBandSince = -1;
        String status = "waiting for speed";
        double speed;

        boolean active = false;
        double startT, pre, minV, minT, maxV, inBandSince;
        int dips;
        boolean below;
        final List<double[]> eventSamples = new ArrayList<>();   // {ms from start, speed}

        Watch(String name, double target) {
            this.name = name;
            this.target = target;
        }

        void setTarget(double target) {
            this.target = target;
            armed = false;
            inArmBandSince = -1;
            active = false;
            status = "waiting for speed";
        }

        double smoothed(double t) {
            double sum = 0;
            int n = 0;
            for (double[] s : recent) {
                if (s[0] >= t - SMOOTH_MS) {
                    sum += s[1];
                    n++;
                }
            }
            return n == 0 ? speed : sum / n;
        }

        void sample(double t, double v) {
            speed = v;
            recent.addLast(new double[]{t, v});
            while (recent.peekFirst()[0] < t - PRE_MS) recent.removeFirst();
            double s = smoothed(t);

            if (!active) {
                if (Math.abs(s - target) <= ARM_BAND_PCT / 100 * target) {
                    if (inArmBandSince < 0) inArmBandSince = t;
                    if (t - inArmBandSince >= ARM_MS) armed = true;
                } else {
                    inArmBandSince = -1;
                }
                status = armed ? "ready: waiting for a shot" : "waiting for speed";
                if (armed && s < target - DROP_TRIGGER) begin(t);
                return;
            }

            status = "shot in progress";
            eventSamples.add(new double[]{t - startT, v});
            if (s < minV) {
                minV = s;
                minT = t;
            }
            maxV = Math.max(maxV, s);
            if (!below && s < target - DROP_TRIGGER) {
                dips++;
                below = true;
            } else if (below && s > target - DROP_TRIGGER / 2) {
                below = false;
            }
            if (Math.abs(s - target) <= BAND_PCT / 100 * target) {
                if (inBandSince < 0) inBandSince = t;
            } else {
                inBandSince = -1;
            }
            if (inBandSince >= 0 && t - inBandSince >= SETTLE_MS) {
                finish(true, inBandSince - startT);
            } else if (t - startT >= MAX_EVENT_MS) {
                finish(false, 0);
            }
        }

        private void begin(double t) {
            active = true;
            startT = t;
            double sum = 0;
            int n = 0;
            for (double[] s : recent) {
                if (s[0] < t - 80) {
                    sum += s[1];
                    n++;
                }
            }
            pre = n == 0 ? target : sum / n;
            eventSamples.clear();
            for (double[] s : recent) eventSamples.add(new double[]{s[0] - t, s[1]});
            minV = smoothed(t);
            minT = t;
            maxV = Double.NEGATIVE_INFINITY;
            dips = 1;
            below = true;
            inBandSince = -1;
        }

        private void finish(boolean recovered, double recoveryMs) {
            Shot shot = new Shot();
            shot.number = ++shotCount;
            shot.target = target;
            shot.pre = pre;
            shot.low = minV;
            shot.timeToLowMs = minT - startT;
            shot.recovered = recovered;
            shot.recoveryMs = recoveryMs;
            shot.overshootPct = Math.max(0, (maxV - target) / target * 100);
            shot.balls = dips;
            shots.add(shot);

            StringBuilder raw = new StringBuilder();
            for (double[] s : eventSamples) {
                raw.append(String.format("%d,%s,%.1f,%.1f%n", shot.number, name, s[0], s[1]));
            }
            append(rawCsv, raw.toString());
            append(eventsCsv, String.format("%d,%s,%.0f,%.1f,%.1f,%.2f,%.0f,%s,%.2f,%d,%s%n", shot.number, name, target,
                    shot.pre, shot.low, shot.dipPct(), shot.timeToLowMs,
                    recovered ? String.format("%.0f", recoveryMs) : "", shot.overshootPct, shot.balls, recovered));
            RobotLog.ii("ShotLogger", String.format("%s shot %d: low %.0f (%.1f%%), recovery %s, overshoot %.1f%%, dips %d",
                    name, shot.number, shot.low, shot.dipPct(), recovered ? String.format("%.0f ms", recoveryMs) : "NOT RECOVERED",
                    shot.overshootPct, shot.balls));

            active = false;
            armed = false;
            inArmBandSince = -1;
        }
    }

    @Override
    public void runOpMode() {
        telemetry = org.firstinspires.ftc.teamcode.subsystems.PanelsSupport.joined(telemetry);   // also show telemetry on Panels
        input.update(gamepad1, gamepad2); pad1 = input.pad1; pad2 = input.pad2;
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, intake, gamepad1);
        shooter.setRumbleEnabled(false);

        double pollenTarget = LOGGER_SPEED;
        Watch pollen = new Watch("Pollen", pollenTarget);

        String stamp = String.valueOf(System.currentTimeMillis());
        eventsCsv = new File(AppUtil.FIRST_FOLDER, "shot_logger_events_" + stamp + ".csv");
        rawCsv = new File(AppUtil.FIRST_FOLDER, "shot_logger_raw_" + stamp + ".csv");
        append(eventsCsv, "shot,wheel,target,before,low,dip_pct,time_to_low_ms,recovery_ms,overshoot_pct,dips,recovered");
        append(rawCsv, "shot,wheel,t_ms,velocity");

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Load balls, then press Play. Hold the shoot button to fire; each shot is recorded by itself.");
            telemetry.addData("Pollen target", pollenTarget);
            telemetry.addLine(String.format("A shot is a drop of more than %.0f below the target.", DROP_TRIGGER));
            telemetry.update();
        }
        if (isStopRequested()) return;

        ElapsedTime clock = new ElapsedTime();
        while (opModeIsActive()) {
            input.update(gamepad1, gamepad2); pad1 = input.pad1; pad2 = input.pad2;
            if (pad1.dpadUpWasPressed() || pad1.dpadDownWasPressed()) {
                double step = pad1.dpad_up ? 25 : -25;
                pollenTarget += step;
                pollen.setTarget(pollenTarget);
            }
            if (pad1.crossWasPressed()) {
                pollen.shots.clear();
            }

            if (pad1.left_bumper) {
                intake.setState(Intake.CaseModes.ON);
            } else if (pad1.square) {
                intake.setState(Intake.CaseModes.REVERSE);
                shooter.openGate();
            } else {
                intake.setState(Intake.CaseModes.OFF);
            }
            if (pad1.right_bumper || pad1.right_trigger_pressed) {
                shooter.setState(Shooter.CaseModes.SHOOT);
            } else {
                shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED);
            }
            shooter.setManualGateOpen(pad1.triangle);
            shooter.setTargetSpeed(pollenTarget);
            shooter.update();
            intake.update();

            double t = clock.milliseconds();
            pollen.sample(t, shooter.getPollenShooterVelocity());

            show(pollen);
            telemetry.addLine("Cross clears the stats. D-pad up/down changes the target.");
            telemetry.addLine("Files: " + eventsCsv.getName());
            telemetry.update();
        }
    }

    private void show(Watch w) {
        telemetry.addLine("--- " + w.name + " (target " + (int) w.target + ") ---");
        telemetry.addData("Speed", "%.0f   %s", w.speed, w.status);
        if (w.shots.isEmpty()) {
            telemetry.addLine("No shots yet.");
            return;
        }
        Shot last = w.shots.get(w.shots.size() - 1);
        telemetry.addLine(String.format("Last shot #%d: before %.0f, low %.0f (-%.1f%%), %s, overshoot %.1f%%, dips %d",
                last.number, last.pre, last.low, last.dipPct(),
                last.recovered ? String.format("back in %.0f ms", last.recoveryMs) : "DID NOT RECOVER",
                last.overshootPct, last.balls));

        double dip = 0, over = 0, sum = 0, worst = 0;
        int recovered = 0;
        for (Shot s : w.shots) {
            dip += s.dipPct();
            over += s.overshootPct;
            if (s.recovered) {
                sum += s.recoveryMs;
                worst = Math.max(worst, s.recoveryMs);
                recovered++;
            }
        }
        int n = w.shots.size();
        double mean = recovered == 0 ? 0 : sum / recovered;
        double var = 0;
        for (Shot s : w.shots) if (s.recovered) var += (s.recoveryMs - mean) * (s.recoveryMs - mean);
        double std = recovered == 0 ? 0 : Math.sqrt(var / recovered);
        telemetry.addLine(String.format("Average of %d: dip %.1f%%, recovery %.0f ms (+/- %.0f), worst %.0f ms, overshoot %.1f%%, not recovered %d",
                n, dip / n, mean, std, worst, over / n, n - recovered));
    }

    private void append(File file, String text) {
        if (file == null) return;
        try (FileWriter fw = new FileWriter(file, true)) {
            fw.write(text);
            if (!text.endsWith("\n")) fw.write("\n");
        } catch (Exception ignored) {
        }
    }
}
