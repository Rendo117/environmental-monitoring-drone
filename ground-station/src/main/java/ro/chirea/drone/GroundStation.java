package ro.chirea.drone;

import javax.swing.*;
import javax.swing.border.EmptyBorder;
import javax.swing.border.TitledBorder;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Properties;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Ground station: control + monitoring application for the environmental drone.
 *
 * <pre>
 *  Tabs:  Connection | Control | Telemetry
 *  Command  -> PUT /drones/{id}/command   {roll,pitch,yaw,throttle (0..100), seq, mode}
 *  Telemetry<- GET /drones/{id}/telemetry/latest  (2 Hz)
 *  Keep-alive: the last applied command is re-sent every 200 ms with a new seq,
 *  so the on-board watchdog can tell "operator holds the sticks" from "link lost".
 *  Keyboard (when enabled):  W/S pitch, A/D yaw, Up/Down throttle, Left/Right roll,
 *                            Space centre sticks, Esc motor stop.
 * </pre>
 *
 * Revision of the 2022 IntelliJ-GUI-designer application: the UI is built in code
 * (no .form file needed), credentials are no longer hard-coded ("user"/"drone"),
 * and the duplicated per-key file/batch/Python logic is a single sendCommand().
 *
 * SPDX-License-Identifier: MIT
 */
public class GroundStation extends JFrame {

    // ---- state ---------------------------------------------------------------
    private final Properties cfg = new Properties();
    private FirebaseClient fb;
    private boolean demo;
    private volatile boolean connected;
    private final ExecutorService io = Executors.newSingleThreadExecutor();
    private final ExecutorService tio = Executors.newSingleThreadExecutor();
    private long seq;
    private final int[] applied = {50, 50, 50, 0};          // roll, pitch, yaw, throttle
    private String droneId = "drone1";
    private PrintWriter csv;

    // ---- widgets -------------------------------------------------------------
    private final JTextField urlField = new JTextField(34);
    private final JTextField keyField = new JTextField(34);
    private final JTextField userField = new JTextField(22);
    private final JPasswordField passField = new JPasswordField(22);
    private final JTextField idField = new JTextField("drone1", 10);
    private final JCheckBox demoBox = new JCheckBox("Offline demo (no Firebase, simulated drone)");
    private final JLabel connLabel = new JLabel("● disconnected");
    private final JTextArea log = new JTextArea(10, 60);

    private final JSpinner rollSp = spinner(50), pitchSp = spinner(50), yawSp = spinner(50), thrSp = spinner(0);
    private final JCheckBox continuous = new JCheckBox("Continuous sender (5 Hz)");
    private final JCheckBox keyboard = new JCheckBox("Keyboard control");
    private final JLabel sentLabel = new JLabel(" ");

    private final Map<String, JLabel> tele = new LinkedHashMap<>();
    private final Chart radChart = new Chart("Dose rate  [µSv/h]", new Color(0xC2410C));
    private final Chart gasChart = new Chart("LPG equivalent  [ppm]", new Color(0x0F766E));

    public GroundStation() {
        super("Environmental Drone — Ground Station");
        loadConfig();
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        JTabbedPane tabs = new JTabbedPane();
        tabs.add("Connection", connectionTab());
        tabs.add("Control", controlTab());
        tabs.add("Telemetry", telemetryTab());
        setContentPane(tabs);
        bindKeys(tabs);
        pack();
        setMinimumSize(new Dimension(820, 560));
        setLocationRelativeTo(null);

        new Timer(200, e -> heartbeat()).start();          // keep-alive / continuous sender
        new Timer(500, e -> pollTelemetry()).start();
    }

    // =========================================================================
    // UI
    // =========================================================================
    private static JSpinner spinner(int v) { return new JSpinner(new SpinnerNumberModel(v, 0, 100, 1)); }

    private JPanel connectionTab() {
        JPanel p = new JPanel(new GridBagLayout());
        p.setBorder(new EmptyBorder(14, 14, 14, 14));
        GridBagConstraints c = new GridBagConstraints();
        c.insets = new Insets(4, 4, 4, 4);
        c.anchor = GridBagConstraints.WEST;
        String[] labels = {"Database URL", "Web API key", "E-mail", "Password", "Drone id"};
        JComponent[] fields = {urlField, keyField, userField, passField, idField};
        for (int r = 0; r < labels.length; r++) {
            c.gridx = 0; c.gridy = r; p.add(new JLabel(labels[r]), c);
            c.gridx = 1; p.add(fields[r], c);
        }
        JButton connect = new JButton("Connect");
        connect.addActionListener(e -> connect());
        c.gridx = 1; c.gridy = labels.length; p.add(demoBox, c);
        c.gridy++;
        JPanel row = new JPanel(new FlowLayout(FlowLayout.LEFT, 0, 0));
        row.add(connect); row.add(Box.createHorizontalStrut(12)); row.add(connLabel);
        p.add(row, c);
        c.gridx = 0; c.gridy++; c.gridwidth = 2; c.fill = GridBagConstraints.BOTH; c.weightx = c.weighty = 1;
        log.setEditable(false);
        log.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
        p.add(new JScrollPane(log), c);
        return p;
    }

    private JPanel controlTab() {
        JPanel p = new JPanel(new BorderLayout(10, 10));
        p.setBorder(new EmptyBorder(14, 14, 14, 14));

        JPanel sticks = new JPanel(new GridLayout(4, 2, 8, 8));
        sticks.setBorder(new TitledBorder("Stick values (0–100, centre 50 → 1500 µs)"));
        sticks.add(new JLabel("Roll  (←/→)")); sticks.add(rollSp);
        sticks.add(new JLabel("Pitch (W/S)")); sticks.add(pitchSp);
        sticks.add(new JLabel("Yaw   (A/D)")); sticks.add(yawSp);
        sticks.add(new JLabel("Throttle (↑/↓)")); sticks.add(thrSp);

        JButton send = new JButton("Send");
        send.addActionListener(e -> applyFromSpinners());
        JButton arm = new JButton("ARM");
        arm.addActionListener(e -> armSequence());
        JButton disarm = new JButton("DISARM");
        disarm.addActionListener(e -> disarmSequence());
        JButton stop = new JButton("MOTOR STOP");
        stop.setForeground(new Color(0xB91C1C));
        stop.setToolTipText("Throttle 0 and level sticks. In flight this makes the aircraft fall!");
        stop.addActionListener(e -> motorStop());

        JPanel buttons = new JPanel(new GridLayout(0, 1, 6, 6));
        buttons.setBorder(new TitledBorder("Commands"));
        for (JComponent b : new JComponent[]{send, continuous, keyboard, arm, disarm, stop}) buttons.add(b);

        JTextArea help = new JTextArea(
                "Arming (Mode-2 convention used by the firmware):\n" +
                "  1. throttle 0 + yaw 0  → STARTING\n" +
                "  2. yaw back to 50       → STARTED (motors idle 1100 µs)\n" +
                "Disarm: throttle 0 + yaw 100.\n" +
                "If no command reaches the drone for 1 s, the Raspberry Pi stops the pulses and the\n" +
                "flight controller levels the aircraft and ramps the throttle down (fail-safe).\n\n" +
                "Keyboard: W/S pitch · A/D yaw · ←/→ roll · ↑/↓ throttle · Space centre · Esc stop");
        help.setEditable(false);
        help.setOpaque(false);
        help.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));

        p.add(sticks, BorderLayout.CENTER);
        p.add(buttons, BorderLayout.EAST);
        JPanel south = new JPanel(new BorderLayout());
        south.add(sentLabel, BorderLayout.NORTH);
        south.add(help, BorderLayout.CENTER);
        p.add(south, BorderLayout.SOUTH);
        return p;
    }

    private JPanel telemetryTab() {
        JPanel p = new JPanel(new BorderLayout(10, 10));
        p.setBorder(new EmptyBorder(14, 14, 14, 14));
        JPanel grid = new JPanel(new GridLayout(0, 4, 10, 6));
        String[] keys = {"Status", "Fail-safe", "Link", "Battery",
                "Roll", "Pitch", "Yaw rate", "Data age",
                "Radiation", "Dose rate", "LPG", "CO", "Smoke", "Gas ADC", "Geiger tube", "Frames"};
        for (String k : keys) {
            JPanel cell = new JPanel(new BorderLayout());
            JLabel name = new JLabel(k);
            name.setForeground(Color.GRAY);
            JLabel v = new JLabel("—");
            v.setFont(v.getFont().deriveFont(Font.BOLD, 15f));
            cell.add(name, BorderLayout.NORTH);
            cell.add(v, BorderLayout.CENTER);
            tele.put(k, v);
            grid.add(cell);
        }
        JPanel charts = new JPanel(new GridLayout(1, 2, 10, 10));
        charts.add(radChart);
        charts.add(gasChart);
        JButton export = new JButton("Record CSV…");
        export.addActionListener(e -> toggleCsv(export));
        JPanel top = new JPanel(new BorderLayout());
        top.add(grid, BorderLayout.CENTER);
        JPanel r = new JPanel(new FlowLayout(FlowLayout.RIGHT));
        r.add(export);
        top.add(r, BorderLayout.SOUTH);
        p.add(top, BorderLayout.NORTH);
        p.add(charts, BorderLayout.CENTER);
        return p;
    }

    private void bindKeys(JComponent root) {
        InputMap im = root.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
        ActionMap am = root.getActionMap();
        Object[][] keys = {
                {KeyEvent.VK_W, pitchSp, +1}, {KeyEvent.VK_S, pitchSp, -1},
                {KeyEvent.VK_D, yawSp, +1}, {KeyEvent.VK_A, yawSp, -1},
                {KeyEvent.VK_RIGHT, rollSp, +1}, {KeyEvent.VK_LEFT, rollSp, -1},
                {KeyEvent.VK_UP, thrSp, +1}, {KeyEvent.VK_DOWN, thrSp, -1}};
        for (Object[] k : keys) {
            String name = "k" + k[0];
            im.put(KeyStroke.getKeyStroke((Integer) k[0], 0), name);
            am.put(name, new AbstractAction() {
                public void actionPerformed(ActionEvent e) {
                    if (!keyboard.isSelected()) return;
                    JSpinner sp = (JSpinner) k[1];
                    int v = Math.max(0, Math.min(100, (Integer) sp.getValue() + (Integer) k[2]));
                    sp.setValue(v);
                    applyFromSpinners();
                }
            });
        }
        im.put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "centre");
        am.put("centre", new AbstractAction() {
            public void actionPerformed(ActionEvent e) {
                if (!keyboard.isSelected()) return;
                rollSp.setValue(50); pitchSp.setValue(50); yawSp.setValue(50);
                applyFromSpinners();
            }
        });
        im.put(KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE, 0), "stop");
        am.put("stop", new AbstractAction() {
            public void actionPerformed(ActionEvent e) { if (keyboard.isSelected()) motorStop(); }
        });
    }

    // =========================================================================
    // Logic
    // =========================================================================
    private void loadConfig() {
        Path p = Path.of("config.properties");
        if (Files.exists(p)) {
            try (InputStream in = new FileInputStream(p.toFile())) { cfg.load(in); } catch (IOException ignored) { }
        }
        urlField.setText(cfg.getProperty("database_url", "https://YOUR-PROJECT-default-rtdb.europe-west1.firebasedatabase.app"));
        keyField.setText(cfg.getProperty("api_key", ""));
        userField.setText(cfg.getProperty("email", ""));
        idField.setText(cfg.getProperty("drone_id", "drone1"));
    }

    private void log(String s) {
        SwingUtilities.invokeLater(() -> {
            log.append(LocalDateTime.now().format(DateTimeFormatter.ofPattern("HH:mm:ss.SSS")) + "  " + s + "\n");
            log.setCaretPosition(log.getDocument().getLength());
        });
    }

    private void connect() {
        demo = demoBox.isSelected();
        droneId = idField.getText().trim();
        if (demo) {
            connected = true;
            setConn(true, "demo mode");
            log("Offline demo: commands are not transmitted, telemetry is simulated.");
            return;
        }
        String email = userField.getText().trim();
        String pass = new String(passField.getPassword());
        fb = new FirebaseClient(urlField.getText().trim(), keyField.getText().trim());
        log("Signing in as " + email + " …");
        io.submit(() -> {
            try {
                fb.signIn(email, pass);
                connected = true;
                setConn(true, "connected");
                log("Authenticated. Path /drones/" + droneId);
            } catch (Exception ex) {
                connected = false;
                setConn(false, "disconnected");
                log("Connection failed: " + ex.getMessage());
            }
        });
    }

    private void setConn(boolean ok, String text) {
        SwingUtilities.invokeLater(() -> {
            connLabel.setText("● " + text);
            connLabel.setForeground(ok ? new Color(0x15803D) : Color.DARK_GRAY);
        });
    }

    private void applyFromSpinners() {
        synchronized (applied) {
            applied[0] = (Integer) rollSp.getValue();
            applied[1] = (Integer) pitchSp.getValue();
            applied[2] = (Integer) yawSp.getValue();
            applied[3] = (Integer) thrSp.getValue();
        }
        sendCommand("manual");
    }

    private void setApplied(int roll, int pitch, int yaw, int thr) {
        rollSp.setValue(roll); pitchSp.setValue(pitch); yawSp.setValue(yaw); thrSp.setValue(thr);
        applyFromSpinners();
    }

    private void armSequence() {
        log("ARM: throttle 0 + yaw 0, then yaw 50");
        setApplied(50, 50, 0, 0);
        new Timer(1200, e -> { setApplied(50, 50, 50, 0); ((Timer) e.getSource()).stop(); }).start();
    }

    private void disarmSequence() {
        log("DISARM: throttle 0 + yaw 100");
        setApplied(50, 50, 100, 0);
        new Timer(1200, e -> { setApplied(50, 50, 50, 0); ((Timer) e.getSource()).stop(); }).start();
    }

    private void motorStop() {
        log("MOTOR STOP");
        setApplied(50, 50, 50, 0);
    }

    /** Called every 200 ms: continuous mode follows the spinners; otherwise keep-alive. */
    private void heartbeat() {
        if (!connected) return;
        if (continuous.isSelected()) {
            synchronized (applied) {
                applied[0] = (Integer) rollSp.getValue();
                applied[1] = (Integer) pitchSp.getValue();
                applied[2] = (Integer) yawSp.getValue();
                applied[3] = (Integer) thrSp.getValue();
            }
        }
        sendCommand(continuous.isSelected() ? "continuous" : "keepalive");
    }

    private void sendCommand(String mode) {
        if (!connected) { log("Not connected — command not sent"); return; }
        final Map<String, Object> cmd = new LinkedHashMap<>();
        synchronized (applied) {
            cmd.put("roll", applied[0]);
            cmd.put("pitch", applied[1]);
            cmd.put("yaw", applied[2]);
            cmd.put("throttle", applied[3]);
        }
        cmd.put("seq", ++seq);
        cmd.put("mode", mode);
        cmd.put("sentAt", Map.of(".sv", "timestamp"));
        sentLabel.setText(String.format("Applied → roll %s  pitch %s  yaw %s  throttle %s   (seq %d, %s)",
                cmd.get("roll"), cmd.get("pitch"), cmd.get("yaw"), cmd.get("throttle"), seq, mode));
        if (!"keepalive".equals(mode)) log("cmd " + Json.write(cmd));
        if (demo) { DemoDrone.command(cmd); return; }
        io.submit(() -> {
            try { fb.put("drones/" + droneId + "/command", cmd); }
            catch (Exception ex) { log("send failed: " + ex.getMessage()); }
        });
    }

    private void pollTelemetry() {
        if (!connected) return;
        if (demo) { showTelemetry(DemoDrone.telemetry()); return; }
        tio.submit(() -> {
            try {
                Object o = fb.get("drones/" + droneId + "/telemetry/latest");
                if (o instanceof Map) SwingUtilities.invokeLater(() -> showTelemetry((Map<?, ?>) o));
            } catch (Exception ex) { log("telemetry: " + ex.getMessage()); }
        });
    }

    private int frames;

    private void showTelemetry(Map<?, ?> t) {
        frames++;
        double usv = Json.num(t, "radiation_usv_h", Double.NaN);
        double sig = Json.num(t, "radiation_sigma_usv_h", Double.NaN);
        double lpg = Json.num(t, "gas_lpg_ppm", Double.NaN);
        double ts = Json.num(t, "time", 0);
        set("Status", String.valueOf(t.get("status")));
        set("Fail-safe", Boolean.TRUE.equals(t.get("failsafe")) ? "ACTIVE" : "no");
        set("Link", Boolean.FALSE.equals(t.get("link_ok")) ? "LOST" : "ok");
        set("Battery", String.format("%.2f V", Json.num(t, "battery_v", Double.NaN)));
        set("Roll", String.format("%+.1f°", Json.num(t, "roll_deg", Double.NaN)));
        set("Pitch", String.format("%+.1f°", Json.num(t, "pitch_deg", Double.NaN)));
        set("Yaw rate", String.format("%+.1f °/s", Json.num(t, "yaw_rate_dps", Double.NaN)));
        set("Data age", ts > 0 ? String.format("%.1f s", (System.currentTimeMillis() - ts) / 1000.0) : "—");
        set("Radiation", String.format("%.0f CPM", Json.num(t, "radiation_cpm", Double.NaN)));
        set("Dose rate", String.format("%.3f ± %.3f µSv/h", usv, sig));
        set("LPG", String.format("%.0f ppm", lpg));
        set("CO", String.format("%.0f ppm", Json.num(t, "gas_co_ppm", Double.NaN)));
        set("Smoke", String.format("%.0f ppm", Json.num(t, "gas_smoke_ppm", Double.NaN)));
        set("Gas ADC", String.format("%.0f", Json.num(t, "gas_adc", Double.NaN)));
        set("Geiger tube", String.valueOf(t.get("geiger_tube")));
        set("Frames", String.valueOf(frames));
        radChart.add(usv);
        gasChart.add(lpg);
        if (csv != null) {
            csv.printf("%s,%s,%.2f,%.1f,%.1f,%.1f,%.4f,%.4f,%.1f%n", LocalDateTime.now(), t.get("status"),
                    Json.num(t, "battery_v", 0), Json.num(t, "roll_deg", 0), Json.num(t, "pitch_deg", 0),
                    Json.num(t, "radiation_cpm", 0), usv, sig, lpg);
            csv.flush();
        }
    }

    private void set(String k, String v) { tele.get(k).setText(v); }

    private void toggleCsv(JButton b) {
        if (csv != null) { csv.close(); csv = null; b.setText("Record CSV…"); log("CSV closed"); return; }
        JFileChooser fc = new JFileChooser();
        fc.setSelectedFile(new java.io.File("measurements.csv"));
        if (fc.showSaveDialog(this) != JFileChooser.APPROVE_OPTION) return;
        try {
            csv = new PrintWriter(new FileWriter(fc.getSelectedFile()));
            csv.println("time,status,battery_v,roll_deg,pitch_deg,cpm,usv_h,sigma_usv_h,lpg_ppm");
            b.setText("Stop recording");
            log("Recording to " + fc.getSelectedFile());
        } catch (IOException ex) { log("CSV: " + ex.getMessage()); }
    }

    // =========================================================================
    /** Minimal rolling line chart (last 120 samples = 60 s). */
    static class Chart extends JPanel {
        private final Deque<Double> v = new ArrayDeque<>();
        private final String title;
        private final Color color;

        Chart(String title, Color color) {
            this.title = title; this.color = color;
            setPreferredSize(new Dimension(360, 180));
            setBackground(Color.WHITE);
            setBorder(BorderFactory.createLineBorder(new Color(0xDDDDDD)));
        }

        void add(double x) {
            if (Double.isNaN(x) || Double.isInfinite(x)) return;
            v.addLast(x);
            while (v.size() > 120) v.removeFirst();
            repaint();
        }

        @Override protected void paintComponent(Graphics g0) {
            super.paintComponent(g0);
            Graphics2D g = (Graphics2D) g0;
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g.setColor(Color.DARK_GRAY);
            g.drawString(title, 10, 18);
            if (v.size() < 2) return;
            double lo = v.stream().mapToDouble(d -> d).min().orElse(0);
            double hi = v.stream().mapToDouble(d -> d).max().orElse(1);
            if (hi - lo < 1e-9) { hi += 1; lo -= 1; }
            int w = getWidth() - 20, h = getHeight() - 40, x0 = 10, y0 = 28;
            g.setColor(new Color(0x888888));
            g.drawString(String.format("%.3g", hi), x0, y0 + 10);
            g.drawString(String.format("%.3g", lo), x0, y0 + h);
            g.setColor(color);
            g.setStroke(new BasicStroke(2f));
            int i = 0, px = 0, py = 0;
            for (double d : v) {
                int x = x0 + i * w / 119;
                int y = y0 + (int) ((hi - d) / (hi - lo) * h);
                if (i > 0) g.drawLine(px, py, x, y);
                px = x; py = y; i++;
            }
        }
    }

    public static void main(String[] args) {
        try { UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName()); } catch (Exception ignored) { }
        SwingUtilities.invokeLater(() -> new GroundStation().setVisible(true));
    }
}
