package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.tuning.FlywheelTunerLogic;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

/**
 * Tunes F and P for the pollen flywheel at a target speed, with no balls.
 *
 * Init: D-pad up/down changes the target speed by 25.
 * Press Play and keep the area around the flywheels clear. B stops it at any time.
 *
 * F: runs with P = 0 and scales F until the speed hits the target.
 * Kick calibration: finds the brake pulse length that dips the speed about 6% (the average real ball dip measured by
 * ShotLogger), then uses that same pulse for every P test, so every P is judged on the same hit.
 * P: holds the target, applies the brake pulse, lets go and measures how it recovers, for a range of P values.
 * The battery voltage during the F stage is shown with the result, because the F needed changes with the battery.
 * The result is shown on the Driver Station and written to /sdcard/FIRST/flywheel_tuner_*.csv.
 */
@TeleOp(name = "Flywheel Auto Tuner", group = "Tuning")
public class FlywheelAutoTuner extends LinearOpMode {
    // Driver Station and Panels (laptop) gamepads combined: read pad1 / pad2 instead of gamepad1 / gamepad2
    private final org.firstinspires.ftc.teamcode.subsystems.LaptopInput input = new org.firstinspires.ftc.teamcode.subsystems.LaptopInput();
    private com.qualcomm.robotcore.hardware.Gamepad pad1;
    private com.qualcomm.robotcore.hardware.Gamepad pad2;
    static final double TUNER_SPEED = 1350;      // the flywheel target the tuner starts at (ticks/s); D-pad changes it by 25
    static final double KICK_POWER = -0.5;       // braking power used to knock the speed down
    // The "ball": a brake pulse of a fixed length. Its length is calibrated once (at CAL_P) so the dip is
    // DIP_FRACTION, then every P is tested with exactly the same pulse, the way every ball takes the same bite.
    static final double START_PULSE_MS = 20;     // first guess for the pulse length
    static final double MIN_PULSE_MS = 3;
    static final double MAX_PULSE_MS = 200;
    static final double CAL_P = 100;             // P used while calibrating the pulse (a middle value)
    static final int CAL_TRIES = 8;              // calibration attempts at most
    static final double CAL_TOL_PCT = 0.75;      // a calibration attempt counts if its dip is this close to the aim (% points)
    static final double NOMINAL_V = 12.5;        // F is also shown scaled to this battery voltage
    static final double HOLD_BAND = 0.015;       // must be within 1.5% of the target before each test
    static final double HOLD_MS = 700;           // ...for this long
    static final double MAX_SPIN_UP_MS = 10000;
    static final double LOW_BATTERY_V = 12.3;

    private static class Wheel {
        final String name;
        final DcMotorEx motor;
        final DcMotorEx.Direction direction;
        double target;
        FlywheelTunerLogic.FResult f;
        FlywheelTunerLogic.PResult p;
        double fVolts;              // battery voltage while F was measured
        double pulseMs;             // calibrated brake pulse length
        boolean pulseCalibrated;

        Wheel(String name, DcMotorEx motor, DcMotorEx.Direction direction, double target) {
            this.name = name;
            this.motor = motor;
            this.direction = direction;
            this.target = target;
        }
    }

    private static class Abort extends RuntimeException {
    }

    private Wheel pollen;
    private VoltageSensor battery;
    private final ElapsedTime uiTimer = new ElapsedTime();
    private final List<String> history = new ArrayList<>();
    private String stage = "";
    private String live = "";
    private File csv;
    private int trialNumber = 0;

    @Override
    public void runOpMode() {
        telemetry = org.firstinspires.ftc.teamcode.subsystems.PanelsSupport.joined(telemetry);   // also show telemetry on Panels
        input.update(gamepad1, gamepad2); pad1 = input.pad1; pad2 = input.pad2;
        pollen = new Wheel("Pollen", hardwareMap.get(DcMotorEx.class, "pollenShooter"),
                DcMotorEx.Direction.REVERSE, TUNER_SPEED);
        battery = hardwareMap.voltageSensor.iterator().next();
        for (Wheel w : new Wheel[]{pollen}) {
            w.motor.setDirection(w.direction);
            w.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            w.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        while (!isStarted() && !isStopRequested()) {
            input.update(gamepad1, gamepad2); pad1 = input.pad1; pad2 = input.pad2;
            double step = 0;
            if (pad1.dpadUpWasPressed()) step = 25;
            if (pad1.dpadDownWasPressed()) step = -25;
            pollen.target += step;
            telemetry.addLine("Clear the area around the flywheels. No balls loaded.");
            telemetry.addData("Pollen target (D-pad up/down)", pollen.target);
            telemetry.addData("Battery", "%.2f V%s", battery.getVoltage(),
                    battery.getVoltage() < LOW_BATTERY_V ? "  LOW: use a fresh battery" : "");
            telemetry.addLine("Press Play to start. B stops it.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        List<Wheel> wheels = new ArrayList<>();
        wheels.add(pollen);

        try {
            for (Wheel w : wheels) {
                tune(w);
            }
        } catch (Abort a) {
            stopMotors();
            if (!opModeIsActive()) return;
            history.add("STOPPED with B");
        }
        stopMotors();

        // Also keep the answer in the robot log, so it can be found after the screen is gone
        for (Wheel w : wheels) {
            if (w.f != null && w.p != null) {
                RobotLog.ii("FlywheelTuner", String.format("RESULT %s: new PIDFCoefficients(%.1f, 0, 0, %.3f)", w.name, w.p.p, w.f.f));
            }
        }

        while (opModeIsActive()) {
            showResults(wheels);
            telemetry.update();
        }
    }

    private void tune(Wheel w) {
        stopMotors();
        csv = new File(AppUtil.FIRST_FOLDER, "flywheel_tuner_" + w.name.toLowerCase() + "_" + System.currentTimeMillis() + ".csv");
        append("trial,stage,p,f,target,t_ms,velocity");
        trialNumber = 0;
        HardwarePlant plant = new HardwarePlant(w);
        FlywheelTunerLogic.Listener log = line -> {
            history.add(w.name + ": " + line);
            while (history.size() > 8) history.remove(0);
            RobotLog.ii("FlywheelTuner", w.name + ": " + line);
            refresh();
        };

        stage = w.name + " - F";
        double startF = 12;
        w.f = FlywheelTunerLogic.tuneF(plant, w.target, startF, log);

        w.fVolts = plant.lastVolts;

        stage = w.name + " - kick calibration";
        plant.calibratePulse(w.f.f, w.target, log);
        w.pulseMs = plant.pulseMs;
        w.pulseCalibrated = plant.pulseCalibrated;

        stage = w.name + " - P";
        w.p = FlywheelTunerLogic.tuneP(plant, w.target, w.f.f, log);
        w.motor.setVelocity(0);
        w.motor.setPower(0);
    }

    /** Runs the real motor for the search logic. */
    private class HardwarePlant implements FlywheelTunerLogic.Plant {
        final Wheel w;
        double pulseMs = START_PULSE_MS;   // brake pulse length, set by calibratePulse()
        boolean pulseCalibrated = false;
        double lastVolts = 0;              // average battery voltage over the last steady-speed measurement

        HardwarePlant(Wheel w) {
            this.w = w;
        }

        /**
         * Finds the brake pulse length that dips the speed by DIP_FRACTION at P = CAL_P. Each attempt adjusts the
         * length from the dip it got (secant step once there are two points). Done when two attempts land within
         * CAL_TOL_PCT; the pulse is then the average of those. Otherwise the closest attempt is used.
         */
        void calibratePulse(double f, double target, FlywheelTunerLogic.Listener log) {
            double want = FlywheelTunerLogic.DIP_FRACTION * 100;
            double d = START_PULSE_MS;
            double prevD = -1, prevDip = -1;
            double bestD = d, bestErr = Double.MAX_VALUE;
            double hitSum = 0;
            int hits = 0;
            for (int i = 0; i < CAL_TRIES; i++) {
                pulseMs = d;
                FlywheelTunerLogic.Samples s = kickTest(CAL_P, f, target);
                double dip = FlywheelTunerLogic.analyze(s, target).dipPct;
                log.status(String.format("Kick calibration %d: pulse %.1f ms -> dip %.1f%% (aim %.1f%%)", i + 1, d, dip, want));
                double err = Math.abs(dip - want);
                if (err < bestErr) {
                    bestErr = err;
                    bestD = d;
                }
                if (err <= CAL_TOL_PCT) {
                    hitSum += d;
                    hits++;
                    if (hits >= 2) break;
                }
                double next;
                if (prevD > 0 && Math.abs(dip - prevDip) > 0.3 && Math.abs(d - prevD) > 0.5) {
                    next = d + (want - dip) * (d - prevD) / (dip - prevDip);
                } else {
                    next = d * want / Math.max(0.5, dip);
                }
                next = Math.max(d * 0.5, Math.min(d * 2, next));
                next = Math.max(MIN_PULSE_MS, Math.min(MAX_PULSE_MS, next));
                prevD = d;
                prevDip = dip;
                d = next;
            }
            pulseCalibrated = hits >= 2;
            pulseMs = pulseCalibrated ? hitSum / hits : bestD;
            log.status(String.format("Kick calibration %s: pulse %.1f ms", pulseCalibrated ? "done" : "NOT settled, using closest",
                    pulseMs));
        }

        private void setPidf(double p, double f) {
            w.motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 0, 0, f));
        }

        @Override
        public double steadyStateSpeed(double f, double target) {
            w.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            setPidf(0, f);
            w.motor.setVelocity(target);
            ElapsedTime t = new ElapsedTime();
            List<double[]> window = new ArrayList<>();   // {time, speed, battery volts}
            while (t.milliseconds() < MAX_SPIN_UP_MS) {
                check();
                double v = w.motor.getVelocity();
                live = String.format("%s speed %.0f / %.0f  F %.3f", w.name, v, target, f);
                window.add(new double[]{t.milliseconds(), v, battery.getVoltage()});
                while (window.get(0)[0] < t.milliseconds() - 600) window.remove(0);
                refresh();
                if (t.milliseconds() > 1800) {
                    double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE, sum = 0;
                    for (double[] s : window) {
                        lo = Math.min(lo, s[1]);
                        hi = Math.max(hi, s[1]);
                        sum += s[1];
                    }
                    if ((hi - lo) / target < 0.03) {
                        lastVolts = averageVolts(window);
                        return sum / window.size();
                    }
                }
            }
            double sum = 0;
            for (double[] s : window) sum += s[1];
            lastVolts = averageVolts(window);
            return sum / window.size();
        }

        private double averageVolts(List<double[]> window) {
            double sum = 0;
            for (double[] s : window) sum += s[2];
            return sum / Math.max(1, window.size());
        }

        @Override
        public FlywheelTunerLogic.Samples kickTest(double p, double f, double target) {
            trialNumber++;
            w.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            setPidf(p, f);
            w.motor.setVelocity(target);

            // Hold at the target until steady
            ElapsedTime t = new ElapsedTime();
            ElapsedTime inBand = new ElapsedTime();
            while (t.milliseconds() < MAX_SPIN_UP_MS) {
                check();
                double v = w.motor.getVelocity();
                live = String.format("%s holding %.0f / %.0f  P %.1f", w.name, v, target, p);
                if (Math.abs(v - target) > HOLD_BAND * target) inBand.reset();
                refresh();
                if (inBand.milliseconds() >= HOLD_MS) break;
            }

            // The "ball": brake for exactly pulseMs, the same for every test. No speed reads in this loop, so the
            // timing is not slowed down by hub reads.
            w.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            w.motor.setPower(KICK_POWER);
            ElapsedTime kick = new ElapsedTime();
            while (kick.milliseconds() < pulseMs) {
                check();
            }

            // Let go and record the recovery
            w.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            setPidf(p, f);
            w.motor.setVelocity(target);
            ElapsedTime rec = new ElapsedTime();
            List<Double> ts = new ArrayList<>();
            List<Double> vs = new ArrayList<>();
            while (rec.milliseconds() < FlywheelTunerLogic.RECORD_MS) {
                check();
                double ms = rec.milliseconds();
                double v = w.motor.getVelocity();
                ts.add(ms);
                vs.add(v);
                live = String.format("%s recovering %.0f / %.0f  P %.1f", w.name, v, target, p);
                refresh();
            }

            double[] tArr = new double[ts.size()];
            double[] vArr = new double[vs.size()];
            StringBuilder rows = new StringBuilder();
            for (int i = 0; i < tArr.length; i++) {
                tArr[i] = ts.get(i);
                vArr[i] = vs.get(i);
                rows.append(String.format("%d,%s,%.2f,%.3f,%.0f,%.1f,%.1f%n", trialNumber, stage, p, f, target, tArr[i], vArr[i]));
            }
            append(rows.toString());
            return new FlywheelTunerLogic.Samples(tArr, vArr);
        }
    }

    private void check() {
        input.update(gamepad1, gamepad2); pad1 = input.pad1; pad2 = input.pad2;
        if (!opModeIsActive() || pad1.b) throw new Abort();
    }

    private void stopMotors() {
        for (Wheel w : new Wheel[]{pollen}) {
            w.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            w.motor.setVelocity(0);
            w.motor.setPower(0);
        }
    }

    private void append(String text) {
        if (csv == null) return;
        try (FileWriter fw = new FileWriter(csv, true)) {
            fw.write(text);
            if (!text.endsWith("\n")) fw.write("\n");
        } catch (Exception ignored) {
        }
    }

    private void refresh() {
        if (uiTimer.milliseconds() < 200) return;
        uiTimer.reset();
        telemetry.addData("Stage", stage);
        telemetry.addData("Now", live);
        telemetry.addData("Battery", "%.2f V", battery.getVoltage());
        for (String line : history) telemetry.addLine(line);
        telemetry.addLine("B stops the tuner.");
        telemetry.update();
    }

    private void showResults(List<Wheel> wheels) {
        telemetry.addLine("DONE. Values to copy into Shooter.java:");
        for (Wheel w : wheels) {
            if (w.f == null) continue;
            telemetry.addLine("--- " + w.name + " (target " + (int) w.target + ") ---");
            if (w.p == null) {
                telemetry.addLine("F " + String.format("%.3f", w.f.f) + "  (P not finished)");
                continue;
            }
            telemetry.addLine(String.format("new PIDFCoefficients(%.1f, 0, 0, %.3f)", w.p.p, w.f.f));
            telemetry.addLine(String.format("Recovery %.0f ms (+/- %.0f)   overshoot %.2f%% (+/- %.2f)   dip %.1f%%",
                    w.p.meanSettleMs, w.p.settleSpreadMs, w.p.meanOvershootPct, w.p.overshootSpreadPct, w.p.meanDipPct));
            telemetry.addLine(String.format("F measured at %.2f V battery (about %.3f at %.1f V). Brake pulse %.1f ms.",
                    w.fVolts, w.f.f * w.fVolts / NOMINAL_V, NOMINAL_V, w.pulseMs));
            telemetry.addLine("Finalists (recovery is from the hit, mean +/- spread):");
            for (String line : w.p.finalistLines) telemetry.addLine("  " + line);
            if (w.p.topAboutEqual) {
                telemetry.addLine("The top P values are within each other's spread: about equal, any of them is fine.");
            }
            if (!w.pulseCalibrated) {
                telemetry.addLine("WARNING: the brake pulse calibration did not settle, so the test hit may not match a real ball.");
            }
            if (!w.f.converged) telemetry.addLine("WARNING: F did not settle on the target. Check the flywheel and rerun.");
            if (w.f.powerFraction > 0.85) {
                telemetry.addLine(String.format("WARNING: holding this speed takes %.0f%% power, so there is little room to recover. Lower the target.",
                        w.f.powerFraction * 100));
            }
            if (!w.p.withinOvershootLimit) telemetry.addLine("WARNING: overshoot is above the limit at every P tested.");
            if (Math.abs(w.p.meanDipPct - FlywheelTunerLogic.DIP_FRACTION * 100) > 2.5) {
                telemetry.addLine(String.format("WARNING: the test dip was %.1f%%, not the %.0f%% aimed for, so recovery times are not like real shots.",
                        w.p.meanDipPct, FlywheelTunerLogic.DIP_FRACTION * 100));
            }
        }
        if (csv != null) telemetry.addLine("Raw data (optional): " + csv.getName());
    }
}
