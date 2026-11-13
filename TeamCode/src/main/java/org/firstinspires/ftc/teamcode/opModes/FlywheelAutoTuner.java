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
 * Tunes F and P for the pollen and nectar flywheels at a target speed, with no balls.
 *
 * Init: Circle picks Pollen / Nectar / Both, D-pad up/down changes the target speed by 25.
 * Press Play and keep the area around the flywheels clear. B stops it at any time.
 *
 * F: runs with P = 0 and scales F until the speed hits the target.
 * P: holds the target, brakes the flywheel down 8%, lets go and measures how it recovers, for a range of P values.
 * The result is shown on the Driver Station and written to /sdcard/FIRST/flywheel_tuner_*.csv.
 */
@TeleOp(name = "Flywheel Auto Tuner", group = "Tuning")
public class FlywheelAutoTuner extends LinearOpMode {
    // Driver Station and Panels (laptop) gamepads combined: read pad1 / pad2 instead of gamepad1 / gamepad2
    private final org.firstinspires.ftc.teamcode.subsystems.LaptopInput input = new org.firstinspires.ftc.teamcode.subsystems.LaptopInput();
    private com.qualcomm.robotcore.hardware.Gamepad pad1;
    private com.qualcomm.robotcore.hardware.Gamepad pad2;
    static final double KICK_POWER = -0.5;       // braking power used to knock the speed down
    static final double KICK_MAX_MS = 600;       // give up braking after this long
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
    private Wheel nectar;
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
                DcMotorEx.Direction.REVERSE, Shooter.POLLEN_SPEED);
        nectar = new Wheel("Nectar", hardwareMap.get(DcMotorEx.class, "nectarShooter"),
                DcMotorEx.Direction.FORWARD, Shooter.NECTAR_SPEED);
        battery = hardwareMap.voltageSensor.iterator().next();
        for (Wheel w : new Wheel[]{pollen, nectar}) {
            w.motor.setDirection(w.direction);
            w.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            w.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        int selection = 0;   // 0 pollen, 1 nectar, 2 both
        String[] names = {"Pollen", "Nectar", "Both"};
        while (!isStarted() && !isStopRequested()) {
            input.update(gamepad1, gamepad2); pad1 = input.pad1; pad2 = input.pad2;
            if (pad1.circleWasPressed()) selection = (selection + 1) % 3;
            double step = 0;
            if (pad1.dpadUpWasPressed()) step = 25;
            if (pad1.dpadDownWasPressed()) step = -25;
            if (selection != 1) pollen.target += step;
            if (selection != 0) nectar.target += step;
            telemetry.addLine("Clear the area around the flywheels. No balls loaded.");
            telemetry.addData("Tuning (Circle)", names[selection]);
            telemetry.addData("Pollen target (D-pad up/down)", pollen.target);
            telemetry.addData("Nectar target", nectar.target);
            telemetry.addData("Battery", "%.2f V%s", battery.getVoltage(),
                    battery.getVoltage() < LOW_BATTERY_V ? "  LOW: use a fresh battery" : "");
            telemetry.addLine("Press Play to start. B stops it.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        List<Wheel> wheels = new ArrayList<>();
        if (selection != 1) wheels.add(pollen);
        if (selection != 0) wheels.add(nectar);

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

        stage = w.name + " - P";
        w.p = FlywheelTunerLogic.tuneP(plant, w.target, w.f.f, log);
        w.motor.setVelocity(0);
        w.motor.setPower(0);
    }

    /** Runs the real motor for the search logic. */
    private class HardwarePlant implements FlywheelTunerLogic.Plant {
        final Wheel w;

        HardwarePlant(Wheel w) {
            this.w = w;
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
            List<double[]> window = new ArrayList<>();   // {time, speed}
            while (t.milliseconds() < MAX_SPIN_UP_MS) {
                check();
                double v = w.motor.getVelocity();
                live = String.format("%s speed %.0f / %.0f  F %.3f", w.name, v, target, f);
                window.add(new double[]{t.milliseconds(), v});
                while (window.get(0)[0] < t.milliseconds() - 600) window.remove(0);
                refresh();
                if (t.milliseconds() > 1800) {
                    double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE, sum = 0;
                    for (double[] s : window) {
                        lo = Math.min(lo, s[1]);
                        hi = Math.max(hi, s[1]);
                        sum += s[1];
                    }
                    if ((hi - lo) / target < 0.03) return sum / window.size();
                }
            }
            double sum = 0;
            for (double[] s : window) sum += s[1];
            return sum / window.size();
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

            // Knock the speed down by brake power, like a ball would
            w.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            w.motor.setPower(KICK_POWER);
            ElapsedTime kick = new ElapsedTime();
            while (kick.milliseconds() < KICK_MAX_MS) {
                check();
                if (w.motor.getVelocity() <= target * (1 - FlywheelTunerLogic.DIP_FRACTION)) break;
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
        for (Wheel w : new Wheel[]{pollen, nectar}) {
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
            if (!w.f.converged) telemetry.addLine("WARNING: F did not settle on the target. Check the flywheel and rerun.");
            if (w.f.powerFraction > 0.85) {
                telemetry.addLine(String.format("WARNING: holding this speed takes %.0f%% power, so there is little room to recover. Lower the target.",
                        w.f.powerFraction * 100));
            }
            if (!w.p.withinOvershootLimit) telemetry.addLine("WARNING: overshoot is above the limit at every P tested.");
        }
        if (csv != null) telemetry.addLine("Raw data (optional): " + csv.getName());
    }
}
