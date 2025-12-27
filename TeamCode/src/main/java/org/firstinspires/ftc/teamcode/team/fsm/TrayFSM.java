package org.firstinspires.ftc.teamcode.team.fsm;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * State machine to automate the tray intake.
 * Usage:
 * - Construct with your hardware objects (tray servo, two intake CRServos, color sensor, telemetry).
 * - Call startAutoIntake() to begin the state machine. Periodically call update() from loop().
 * - Telemetry will show raw RGB (0-255) and detected slot states.
 */
public class TrayFSM {
    public enum SlotState { EMPTY, PURPLE, GREEN, UNKNOWN }
    private enum State { IDLE, POSITION_TO_SLOT, INTAKE_WAIT, CHECK_SLOT, DONE }

    private final Servo trayServo;
    private final CRServo rubberBands;      // CRServo for rubber band intake
    private final CRServo intakeRoller;     // CRServo for intake roller
    private final NormalizedColorSensor colorSensor;
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    // Slot servo target positions (configurable). Default values are placeholders and should be tuned.
    private final double[] slotPositions = new double[]{0.0, 0.5, 1.0};
    private double intakeRubberPower = 1.0;
    private double intakeRollerPower = 1.0;

    private final SlotState[] slots = new SlotState[3];
    private State state = State.IDLE;
    private int currentSlotIndex = 0; // 0..2

    // Timing/timeouts (in seconds)
    private double intakeTimeout = 2.0; // how long to run intake while waiting for ball
    private double settleTime = 0.3;    // how long to wait after moving servo before checking

    // Color thresholds (0-255)
    private int presenceThreshold = 30; // brightness threshold to decide something is present
    private int purpleThreshold = 40; // combined red+blue threshold to consider purple
    private int greenThreshold = 40;  // green threshold to consider green

    private double stateStartTime = 0.0;

    public TrayFSM(Servo trayServo, CRServo rubberBands, CRServo intakeRoller, NormalizedColorSensor colorSensor, Telemetry telemetry) {
        this.trayServo = trayServo;
        this.rubberBands = rubberBands;
        this.intakeRoller = intakeRoller;
        this.colorSensor = colorSensor;
        this.telemetry = telemetry;
        for (int i = 0; i < 3; i++) slots[i] = SlotState.EMPTY;
    }

    // Optional configuration methods
    public void setSlotServoPositions(double pos0, double pos1, double pos2) {
        slotPositions[0] = pos0;
        slotPositions[1] = pos1;
        slotPositions[2] = pos2;
    }

    public void setIntakeTimeout(double seconds) { this.intakeTimeout = seconds; }
    public void setSettleTime(double seconds) { this.settleTime = seconds; }
    public void setPresenceThreshold(int v) { this.presenceThreshold = v; }
    public void setPurpleThreshold(int v) { this.purpleThreshold = v; }
    public void setGreenThreshold(int v) { this.greenThreshold = v; }
    public void setIntakePowers(double rubberPower, double rollerPower) { this.intakeRubberPower = rubberPower; this.intakeRollerPower = rollerPower; }

    // Start the automatic intake process. This will clear previous slot data and begin at slot 0.
    public void startAutoIntake() {
        for (int i = 0; i < 3; i++) slots[i] = SlotState.EMPTY;
        currentSlotIndex = findNextEmptySlot(0);
        if (currentSlotIndex < 0) {
            state = State.DONE;
            return;
        }
        state = State.POSITION_TO_SLOT;
        stateStartTime = timer.seconds();
        moveToSlot(currentSlotIndex);
    }

    // Call this periodically (e.g., opmode loop)
    public void update() {
        // Read RGB scaled to 0-255
        int[] rgb = readRGB255();
        // Add telemetry for raw rgb
        telemetry.addData("RGB", String.format("%d / %d / %d", rgb[0], rgb[1], rgb[2]));

        switch (state) {
            case IDLE:
                // nothing
                break;

            case POSITION_TO_SLOT:
                // Wait a short settle time after moving servo
                if (timer.seconds() - stateStartTime >= settleTime) {
                    // Start intake to try to capture a ball
                    rubberBands.setPower(intakeRubberPower);
                    intakeRoller.setPower(intakeRollerPower);
                    state = State.INTAKE_WAIT;
                    stateStartTime = timer.seconds();
                    telemetry.addData("State", String.format("INTAKE_WAIT slot=%d", currentSlotIndex + 1));
                }
                break;

            case INTAKE_WAIT:
                // while intake running, monitor sensor to see if a ball has arrived
                SlotState detected = classifyCurrentColor();
                if (detected != SlotState.EMPTY) {
                    // Ball detected — stop intake, record, move to next slot
                    rubberBands.setPower(0.0);
                    intakeRoller.setPower(0.0);
                    slots[currentSlotIndex] = detected;
                    telemetry.addData(String.format("Slot %d", currentSlotIndex + 1), slots[currentSlotIndex].toString());
                    // Move to next empty slot
                    int next = findNextEmptySlot(currentSlotIndex + 1);
                    if (next < 0) {
                        state = State.DONE;
                    } else {
                        currentSlotIndex = next;
                        moveToSlot(currentSlotIndex);
                        state = State.POSITION_TO_SLOT;
                        stateStartTime = timer.seconds();
                    }
                } else if (timer.seconds() - stateStartTime >= intakeTimeout) {
                    // Timeout, assume no ball arrived. Stop intake and mark EMPTY, go to next slot
                    rubberBands.setPower(0.0);
                    intakeRoller.setPower(0.0);
                    slots[currentSlotIndex] = SlotState.EMPTY;
                    int next = findNextEmptySlot(currentSlotIndex + 1);
                    if (next < 0) {
                        state = State.DONE;
                    } else {
                        currentSlotIndex = next;
                        moveToSlot(currentSlotIndex);
                        state = State.POSITION_TO_SLOT;
                        stateStartTime = timer.seconds();
                    }
                } else {
                    telemetry.addData("IntakeWaiting", String.format("slot=%d time=%.2f", currentSlotIndex + 1, timer.seconds() - stateStartTime));
                }
                break;

            case CHECK_SLOT:
                // Not used in this simple flow (kept for extensibility)
                state = State.DONE;
                break;

            case DONE:
                telemetry.addData("TrayFSM", "DONE");
                // leave motors off
                rubberBands.setPower(0.0);
                intakeRoller.setPower(0.0);
                break;
        }

        // Output slot summary
        for (int i = 0; i < 3; i++) {
            telemetry.addData(String.format("Slot %d", i + 1), slots[i].toString());
        }

        telemetry.update();
    }

    // Move servo to the given slot index (0..2)
    private void moveToSlot(int slotIndex) {
        double pos = slotPositions[Math.max(0, Math.min(2, slotIndex))];
        trayServo.setPosition(pos);
    }

    // Returns next index >= start that is considered empty in the internal slots array; -1 if none
    private int findNextEmptySlot(int start) {
        for (int i = start; i < 3; i++) {
            if (slots[i] == SlotState.EMPTY) return i;
        }
        return -1;
    }

    // Read color sensor and return RGB scaled to 0..255
    private int[] readRGB255() {
        // NormalizedColorSensor returns floats 0..1. Convert to 0..255 and clamp.
        com.qualcomm.robotcore.hardware.NormalizedRGBA colors = colorSensor.getNormalizedColors();
        int r = normTo255(colors.red);
        int g = normTo255(colors.green);
        int b = normTo255(colors.blue);
        return new int[]{r, g, b};
    }

    private int normTo255(float v) {
        if (Float.isNaN(v)) return 0;
        float clamped = Math.max(0f, Math.min(1f, v));
        return (int) (clamped * 255f + 0.5f);
    }

    // Classify current color reading into EMPTY / PURPLE / GREEN / UNKNOWN
    private SlotState classifyCurrentColor() {
        int[] rgb = readRGB255();
        int r = rgb[0], g = rgb[1], b = rgb[2];
        int brightness = (r + g + b) / 3;
        // If very dark, treat as empty
        if (brightness < presenceThreshold) return SlotState.EMPTY;
        // Detect green: green component dominant and above threshold
        if (g >= r && g >= b && g >= greenThreshold) return SlotState.GREEN;
        // Detect purple: red and blue are both strong compared to green
        int redBlueAvg = (r + b) / 2;
        if (redBlueAvg >= purpleThreshold && redBlueAvg > g) return SlotState.PURPLE;
        // Fallback unknown
        return SlotState.UNKNOWN;
    }

    // Public getters
    public SlotState[] getSlots() {
        return slots.clone();
    }

    public boolean isDone() { return state == State.DONE; }

    public void stop() {
        rubberBands.setPower(0);
        intakeRoller.setPower(0);
        state = State.IDLE;
    }

}
