package ro.chirea.drone;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Random;

/**
 * Offline stand-in for the aircraft so the ground station can be explored without
 * Firebase or hardware. Telemetry is synthetic: attitude follows the sticks through a
 * first-order lag, gas drifts slowly, Geiger counts are Poisson distributed.
 *
 * SPDX-License-Identifier: MIT
 */
final class DemoDrone {
    private static final Random RNG = new Random(42);
    private static double roll, pitch, yawRate;
    private static int cmdRoll = 50, cmdPitch = 50, cmdYaw = 50, cmdThr;
    private static String status = "STOPPED";
    private static final int[] ring = new int[60];
    private static int idx, tick;
    private static final long T0 = System.currentTimeMillis();

    private DemoDrone() { }

    static synchronized void command(Map<String, Object> c) {
        cmdRoll = (Integer) c.get("roll");
        cmdPitch = (Integer) c.get("pitch");
        cmdYaw = (Integer) c.get("yaw");
        cmdThr = (Integer) c.get("throttle");
        if ("STOPPED".equals(status) && cmdYaw <= 1 && cmdThr <= 1) status = "STARTING";
        else if ("STARTING".equals(status) && Math.abs(cmdYaw - 50) < 5 && cmdThr <= 1) status = "STARTED";
        else if ("STARTED".equals(status) && cmdYaw >= 99 && cmdThr <= 1) status = "STOPPED";
    }

    private static int poisson(double lambda) {
        double l = Math.exp(-lambda), p = 1;
        int k = 0;
        do { k++; p *= RNG.nextDouble(); } while (p > l);
        return k - 1;
    }

    static synchronized Map<String, Object> telemetry() {
        double t = (System.currentTimeMillis() - T0) / 1000.0;
        // angle mode: target angle = (stick - 50) * 0.656 deg  (492 us -> 32.8 deg)
        roll += 0.3 * ((cmdRoll - 50) * 0.656 - roll) + 0.2 * RNG.nextGaussian();
        pitch += 0.3 * ((cmdPitch - 50) * 0.656 - pitch) + 0.2 * RNG.nextGaussian();
        yawRate += 0.3 * ((cmdYaw - 50) * 3.28 - yawRate);
        if (++tick % 2 == 0) {                         // 2 polls per second -> 1 s Geiger gate
            idx = (idx + 1) % 60;
            ring[idx] = poisson(0.35 + 0.25 * Math.max(0, Math.sin(t / 25)));  // background + hot-spot
        }
        int counts = 0;
        for (int c : ring) counts += c;
        double cf = 0.0057;
        int gasAdc = (int) (170 + 280 * Math.pow(Math.max(0, Math.sin(t / 18)), 2) + 4 * RNG.nextGaussian());
        double rs = 1.0 * (5 - gasAdc * 5 / 1023.0) / (gasAdc * 5 / 1023.0);
        double ratio = rs / 0.51;                      // R0 = Rs(clean air)/9.83 for ADC 170
        double lpg = Math.pow(10, (Math.log10(ratio) - 0.21) / -0.47 + 2.3);
        double co = Math.pow(10, (Math.log10(ratio) - 0.72) / -0.34 + 2.3);
        double smoke = Math.pow(10, (Math.log10(ratio) - 0.53) / -0.44 + 2.3);

        Map<String, Object> m = new LinkedHashMap<>();
        m.put("status", status);
        m.put("failsafe", false);
        m.put("link_ok", true);
        m.put("battery_v", 12.4 - 0.002 * t);
        m.put("roll_deg", roll);
        m.put("pitch_deg", pitch);
        m.put("yaw_rate_dps", yawRate);
        m.put("gas_adc", (double) gasAdc);
        m.put("gas_lpg_ppm", lpg);
        m.put("gas_co_ppm", co);
        m.put("gas_smoke_ppm", smoke);
        m.put("radiation_cpm", (double) counts);
        m.put("radiation_usv_h", counts * cf);
        m.put("radiation_sigma_usv_h", Math.sqrt(Math.max(counts, 1)) * cf);
        m.put("geiger_tube", "SBM-20 (demo)");
        m.put("time", (double) System.currentTimeMillis());
        return m;
    }
}
