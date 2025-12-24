package org.firstinspires.ftc.teamcode.team.fsm;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class ShootPatternFSM {

    private final DarienOpModeFSM opMode;
    private final ShootArtifactFSM shootArtifactFSM;

    public ShootPatternFSM(DarienOpModeFSM opMode) {
        this.opMode = opMode;
        this.shootArtifactFSM = opMode.shootArtifactFSM;
    }

    private ArrayList<AprilTagDetection> aprilTagDetections;
    private int nbStep = 0;
    private int nbMotifIndex = 0;
    private double nbLastActionTime = 0;
    private boolean nbShootingActive = false;
    private double shootPower = 0;
    private boolean shotStarted = false;

    public void startShootPattern(ArrayList<AprilTagDetection> detections, double currentTime, double shootingPower) {
        aprilTagDetections = detections;
        nbStep = 0;
        nbMotifIndex = 0;
        nbLastActionTime = currentTime;
        nbShootingActive = true;
        shootPower = shootingPower;
        shootArtifactFSM.setEjectionMotorsControlledByPattern(true);
        shootArtifactFSM.shotGun(shootPower);
    }

    public void updateShootPattern(double currentTime) {
        if (!nbShootingActive || aprilTagDetections == null || aprilTagDetections.isEmpty()) return;

        AprilTagDetection detection = aprilTagDetections.get(0); // Only use first detection for motif
        int[] motif = null;
        // Motif order: 1=TRAY_POS_1_SCORE, 2=TRAY_POS_2_SCORE, 3=TRAY_POS_3_SCORE
        switch (detection.id) {
            case 21:
                motif = new int[]{2, 3, 1}; break; // GPP
            case 22:
                motif = new int[]{1, 2, 3}; break; // PGP
            case 23:
            default:
                motif = new int[]{1, 3, 2}; break; // PPG
        }

        if (nbMotifIndex >= motif.length) {
            nbShootingActive = false;
            shootArtifactFSM.shotGunStop(); // Stop motors
            shootArtifactFSM.setEjectionMotorsControlledByPattern(false);
            return;
        }

        switch (nbStep) {
            case 0: // Move tray
                double targetPos = (motif[nbMotifIndex] == 1) ? DarienOpModeFSM.TRAY_POS_1_SCORE :
                        (motif[nbMotifIndex] == 2) ? DarienOpModeFSM.TRAY_POS_2_SCORE :
                                DarienOpModeFSM.TRAY_POS_3_SCORE;
                //opMode.servoIncremental(opMode.TrayServo, targetPos, opMode.currentTrayPosition, 1, 4);
                //opMode.currentTrayPosition = targetPos;
                opMode.setTrayPosition(targetPos);
                nbLastActionTime = currentTime;
                nbStep = 1;
                shotStarted = false; // Reset for next shot
                break;
            case 1: // Wait for tray move
                if (currentTime - nbLastActionTime >= 1.0) {
                    nbLastActionTime = currentTime;
                    nbStep = 2;
                }
                break;
            case 2: // Shoot and wait for completion
                if (!shotStarted) {
                    shootArtifactFSM.startShooting(shootPower);
                    shotStarted = true;
                    nbLastActionTime = currentTime;
                }
                shootArtifactFSM.updateShooting();
                if (shootArtifactFSM.shootingDone() || currentTime - nbLastActionTime >= 2.0) {
                    shootArtifactFSM.resetShooting();
                    nbMotifIndex++;
                    nbStep = 0;
                }
                break;
        }
    }

    public boolean isShootPatternDone() {
        return !nbShootingActive;
    }

}
