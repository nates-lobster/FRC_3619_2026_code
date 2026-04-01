package frc.robot.utils;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;

public class MacroRecorder {
    private static final String BASE_PATH = "/home/lvuser/";
    private String currentFileName = "recorded_macro.csv";
    
    public static class Step {
        public final double timestamp;
        public final double left;
        public final double right;

        public Step(double timestamp, double left, double right) {
            this.timestamp = timestamp;
            this.left = left;
            this.right = right;
        }

        @Override
        public String toString() {
            return String.format("%.3f,%.3f,%.3f", timestamp, left, right);
        }
    }

    private final List<Step> recordedSteps = new ArrayList<>();
    private final Timer timer = new Timer();
    private boolean isRecording = false;

    public void startRecording(String name) {
        this.currentFileName = name.endsWith(".csv") ? name : name + ".csv";
        recordedSteps.clear();
        timer.restart();
        isRecording = true;
        System.out.println("[Macro] Recording started for: " + currentFileName);
    }

    public void record(double left, double right) {
        if (!isRecording) return;
        recordedSteps.add(new Step(timer.get(), left, right));
    }

    public void stopRecording() {
        isRecording = false;
        timer.stop();
        saveToFile();
        System.out.println("[Macro] Recording stopped. Saved " + recordedSteps.size() + " steps to " + currentFileName);
    }

    public boolean isRecording() {
        return isRecording;
    }

    private void saveToFile() {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(BASE_PATH + currentFileName))) {
            for (Step step : recordedSteps) {
                writer.write(step.toString());
                writer.newLine();
            }
        } catch (IOException e) {
            System.err.println("[Macro] Failed to save macro: " + e.getMessage());
        }
    }

    public List<Step> loadFromFile(String name) {
        String fileName = name.endsWith(".csv") ? name : name + ".csv";
        List<Step> steps = new ArrayList<>();
        File file = new File(BASE_PATH + fileName);
        if (!file.exists()) {
            System.err.println("[Macro] No recorded macro found at " + file.getAbsolutePath());
            return steps;
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(",");
                if (parts.length == 3) {
                    steps.add(new Step(
                        Double.parseDouble(parts[0]),
                        Double.parseDouble(parts[1]),
                        Double.parseDouble(parts[2])
                    ));
                }
            }
        } catch (IOException | NumberFormatException e) {
            System.err.println("[Macro] Failed to load macro: " + e.getMessage());
        }
        return steps;
    }
}
