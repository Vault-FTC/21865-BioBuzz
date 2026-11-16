package org.firstinspires.ftc.teamcode.tuning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The search for one flywheel's F and P. No hardware in here: the OpMode (or a test) supplies a Plant that
 * runs the motor.
 *
 * F: run with P = 0 at the target speed and scale F until the steady speed hits the target.
 * P: with F set, hit the wheel with the same brake pulse every time (the OpMode calibrates it to dip about
 * DIP_FRACTION), let go, and measure how long it takes to be back at speed. Try P values from low to high and keep the
 * one that is back fastest without overshooting or ringing.
 */
public final class FlywheelTunerLogic {
    private FlywheelTunerLogic() {}

    // Edit these to change what "good" means
    public static final double DIP_FRACTION = 0.06;        // the dip the brake pulse is calibrated to (6%: the average real shot dip measured by ShotLogger)
    public static final double BAND_PCT = 1.0;             // recovered = within this % of the target
    public static final double MAX_OVERSHOOT_PCT = 2.0;    // overshoot above target allowed
    public static final double SMOOTH_MS = 25;             // readings are averaged over +/- this before judging them
    public static final double RECORD_MS = 2500;           // how long each test is recorded after release
    public static final double P_MIN = 10;
    public static final double P_MAX = 1000;
    public static final double P_STEP = 1.6;               // coarse search multiplies P by this each trial
    public static final double OVERSHOOT_WEIGHT_MS = 150;  // each 1% of overshoot costs this much (ms)
    public static final double RINGING_WEIGHT_MS = 400;    // each extra crossing of the target costs this much (ms)
    public static final int FINAL_REPEATS = 4;             // final candidates are each tested this many times
    public static final int FINALISTS = 3;
    // A test whose dip is far outside this range of the aim (DIP_FRACTION) is treated as a glitch and run once more
    public static final double ODD_DIP_LOW = 0.4;
    public static final double ODD_DIP_HIGH = 2.5;

    public interface Plant {
        /** Runs closed loop with P = I = D = 0 and this F; returns the steady speed (ticks/s, positive). */
        double steadyStateSpeed(double f, double target);

        /** Holds the target with this P and F, knocks the speed down, lets go, records the recovery. */
        Samples kickTest(double p, double f, double target);
    }

    public interface Listener {
        void status(String line);
    }

    /** Speed readings after the speed was released, times in ms since release. */
    public static final class Samples {
        public final double[] tMs;
        public final double[] v;

        public Samples(double[] tMs, double[] v) {
            this.tMs = tMs;
            this.v = v;
        }
    }

    public static final class Metrics {
        public double dipPct;
        public double settleMs;       // time after the lowest point until the speed stays within the band
        public double recoverMs;      // time after the hit (release) until the speed stays within the band; what P is judged on
        public double overshootPct;
        public int crossings;         // times it crossed the target after the lowest point
        public boolean ringing;
        public boolean unsettled;     // still outside the band when recording ended
        public double cost;

        @Override
        public String toString() {
            return String.format("dip %.1f%%  back %.0f ms  over %.1f%%  cross %d%s", dipPct, recoverMs, overshootPct,
                    crossings, ringing ? "  RINGING" : (unsettled ? "  SLOW" : ""));
        }
    }

    public static final class Trial {
        public final double p;
        public final Metrics m;

        Trial(double p, Metrics m) {
            this.p = p;
            this.m = m;
        }
    }

    public static final class FResult {
        public double f;
        public double steadySpeed;
        public double powerFraction;   // motor power needed to hold the target (1.0 = full power)
        public boolean converged;
    }

    public static final class PResult {
        public double p;
        public double meanSettleMs;
        public double meanOvershootPct;
        public double meanDipPct;
        public double settleSpreadMs;     // std dev of settle time over the repeats: lower = more consistent shots
        public double overshootSpreadPct;
        public boolean withinOvershootLimit;
        public List<Trial> trials = new ArrayList<>();
        public double meanRecoverMs;      // from the hit, the number P was chosen on
        public double recoverSpreadMs;
        public double meanCost;
        public List<String> finalistLines = new ArrayList<>();   // every finalist's averages, best first
        public boolean topAboutEqual;     // the best and second best differ by less than their spread
    }

    // ---------- F ----------

    public static FResult tuneF(Plant plant, double target, double startF, Listener log) {
        FResult r = new FResult();
        double f = startF;
        for (int i = 0; i < 10; i++) {
            double v = plant.steadyStateSpeed(f, target);
            double err = (target - v) / target;
            log.status(String.format("F stage: F %.3f -> %.0f ticks/s (target %.0f, %+.1f%%)", f, v, target, -err * 100));
            r.f = f;
            r.steadySpeed = v;
            if (Math.abs(err) < 0.004) {
                r.converged = true;
                break;
            }
            double ratio = v > 1 ? target / v : 1.5;
            ratio = Math.max(0.5, Math.min(2.0, ratio));
            f = Math.max(0.5, Math.min(200, f * ratio));
        }
        r.powerFraction = r.f * target / 32767.0;
        return r;
    }

    // ---------- P ----------

    /** Moving average over +/- SMOOTH_MS, so reading noise isn't mistaken for dips, overshoot or ringing. */
    static double[] smooth(Samples s) {
        int n = s.v.length;
        double[] out = new double[n];
        for (int i = 0; i < n; i++) {
            double sum = 0;
            int count = 0;
            for (int j = i; j >= 0 && s.tMs[i] - s.tMs[j] <= SMOOTH_MS; j--) {
                sum += s.v[j];
                count++;
            }
            for (int j = i + 1; j < n && s.tMs[j] - s.tMs[i] <= SMOOTH_MS; j++) {
                sum += s.v[j];
                count++;
            }
            out[i] = sum / count;
        }
        return out;
    }

    public static Metrics analyze(Samples raw, double target) {
        Metrics m = new Metrics();
        double[] sm = smooth(raw);
        Samples s = new Samples(raw.tMs, sm);
        int n = s.v.length;

        // Reading noise: the part of the raw signal the smoothing removed, scaled down by the averaging it does
        double resid = 0;
        for (int i = 0; i < n; i++) resid += (raw.v[i] - sm[i]) * (raw.v[i] - sm[i]);
        double sigmaRaw = Math.sqrt(resid / Math.max(1, n));
        double perWindow = Math.max(1, (2 * SMOOTH_MS) / Math.max(1e-6, (raw.tMs[n - 1] - raw.tMs[0]) / Math.max(1, n - 1)));
        double sigmaPct = sigmaRaw / Math.sqrt(perWindow) / target * 100;
        double band = Math.max(BAND_PCT, 3 * sigmaPct);
        double deadband = Math.max(0.5, 3 * sigmaPct);
        if (n < 5) {
            m.unsettled = true;
            m.settleMs = RECORD_MS;
            m.cost = 1e6;
            return m;
        }
        int iMin = 0;
        for (int i = 1; i < n; i++) if (s.v[i] < s.v[iMin]) iMin = i;
        m.dipPct = (target - s.v[iMin]) / target * 100;

        double lastOutside = -1;
        double maxOver = 0;
        for (int i = iMin; i < n; i++) {
            double e = (s.v[i] - target) / target * 100;
            if (Math.abs(e) > band) lastOutside = s.tMs[i];
            if (e > maxOver) maxOver = e;
        }
        m.overshootPct = maxOver;
        m.unsettled = Math.abs((s.v[n - 1] - target) / target * 100) > band;
        m.settleMs = m.unsettled ? RECORD_MS : (lastOutside < 0 ? 0 : lastOutside - s.tMs[iMin]);
        m.recoverMs = m.unsettled ? RECORD_MS : (lastOutside < 0 ? 0 : lastOutside - s.tMs[0]);

        // Crossings of the target, with a small dead band so noise doesn't count
        int sign = 0;
        int crossings = 0;
        for (int i = iMin; i < n; i++) {
            double e = (s.v[i] - target) / target * 100;
            int now = e > deadband ? 1 : (e < -deadband ? -1 : 0);
            if (now != 0) {
                if (sign != 0 && now != sign) crossings++;
                sign = now;
            }
        }
        m.crossings = crossings;
        m.ringing = crossings > 6;

        // Judged from the hit, not from the lowest point: every P gets the same hit, so a P that stops the fall sooner
        // (a shallower dip) is rightly scored better.
        m.cost = m.recoverMs + OVERSHOOT_WEIGHT_MS * m.overshootPct + RINGING_WEIGHT_MS * Math.max(0, crossings - 2);
        if (m.overshootPct > MAX_OVERSHOOT_PCT) m.cost += 5000;
        if (m.ringing) m.cost += 5000;
        return m;
    }

    public static PResult tuneP(Plant plant, double target, double f, Listener log) {
        List<Trial> all = new ArrayList<>();

        // Coarse: P_MIN, P_MIN * step, ... Stop once it rings, since higher is only worse.
        double p = P_MIN;
        int ringingCount = 0;
        while (p <= P_MAX * 1.001) {
            Trial t = run(plant, target, f, p, all, log, "coarse");
            if (t.m.ringing) ringingCount++;
            if (ringingCount >= 2) break;
            p *= P_STEP;
        }

        // Refine: between the best coarse P and its neighbours
        Trial best = Collections.min(all, (a, b) -> Double.compare(a.m.cost, b.m.cost));
        double lo = P_MIN;
        double hi = P_MAX;
        for (Trial t : all) {
            if (t.p < best.p) lo = Math.max(lo, t.p);
        }
        for (Trial t : all) {
            if (t.p > best.p) {
                hi = t.p;
                break;
            }
        }
        if (lo < best.p) run(plant, target, f, Math.sqrt(lo * best.p), all, log, "refine");
        if (hi > best.p) run(plant, target, f, Math.sqrt(best.p * hi), all, log, "refine");

        // Final: the best few candidates, each repeated, judged on their average
        List<Trial> sorted = new ArrayList<>(all);
        Collections.sort(sorted, (a, b) -> Double.compare(a.m.cost, b.m.cost));
        List<Double> finalists = new ArrayList<>();
        for (Trial t : sorted) {
            if (finalists.size() >= FINALISTS) break;
            if (!finalists.contains(t.p)) finalists.add(t.p);
        }

        List<PResult> results = new ArrayList<>();
        for (double fp : finalists) {
            PResult r = new PResult();
            r.p = fp;
            double[] settle = new double[FINAL_REPEATS];
            double[] recover = new double[FINAL_REPEATS];
            double[] over = new double[FINAL_REPEATS];
            double[] cost = new double[FINAL_REPEATS];
            for (int i = 0; i < FINAL_REPEATS; i++) {
                Trial t = run(plant, target, f, fp, all, log, "final " + (i + 1) + "/" + FINAL_REPEATS);
                r.trials.add(t);
                settle[i] = t.m.settleMs;
                recover[i] = t.m.recoverMs;
                over[i] = t.m.overshootPct;
                cost[i] = t.m.cost;
                r.meanDipPct += t.m.dipPct / FINAL_REPEATS;
            }
            r.meanCost = mean(cost);
            r.meanSettleMs = mean(settle);
            r.meanRecoverMs = mean(recover);
            r.meanOvershootPct = mean(over);
            r.settleSpreadMs = std(settle);
            r.recoverSpreadMs = std(recover);
            r.overshootSpreadPct = std(over);
            r.withinOvershootLimit = r.meanOvershootPct <= MAX_OVERSHOOT_PCT;
            results.add(r);
        }
        Collections.sort(results, (a, b) -> Double.compare(a.meanCost, b.meanCost));
        PResult bestResult = results.get(0);
        for (PResult r : results) {
            bestResult.finalistLines.add(String.format("P %.1f: back in %.0f ms (+/- %.0f), over %.2f%%, dip %.1f%%",
                    r.p, r.meanRecoverMs, r.recoverSpreadMs, r.meanOvershootPct, r.meanDipPct));
        }
        if (results.size() > 1) {
            PResult second = results.get(1);
            bestResult.topAboutEqual = second.meanCost - bestResult.meanCost < bestResult.recoverSpreadMs + second.recoverSpreadMs;
        }
        bestResult.trials = all;
        return bestResult;
    }

    /** One test at this P. A test with a far-off dip is treated as a glitch and run once more. */
    private static Trial run(Plant plant, double target, double f, double p, List<Trial> all, Listener log, String stage) {
        double want = DIP_FRACTION * 100;
        Trial t = null;
        for (int attempt = 0; attempt < 2; attempt++) {
            Samples s = plant.kickTest(p, f, target);
            Metrics m = analyze(s, target);
            t = new Trial(p, m);
            boolean odd = m.dipPct < want * ODD_DIP_LOW || m.dipPct > want * ODD_DIP_HIGH;
            log.status(String.format("P stage (%s): P %.1f  %s%s", stage, p, m, odd && attempt == 0 ? "  odd dip, retrying" : ""));
            if (!odd) break;
        }
        all.add(t);
        return t;
    }

    static double mean(double[] a) {
        double s = 0;
        for (double x : a) s += x;
        return s / a.length;
    }

    static double std(double[] a) {
        double m = mean(a);
        double s = 0;
        for (double x : a) s += (x - m) * (x - m);
        return Math.sqrt(s / a.length);
    }
}
