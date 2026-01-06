package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;

@TeleOp(name = "TeleopFSM", group = "DriverControl")
@Config
@Configurable
public class TeleOpFSM extends DarienOpModeFSM {

    // INSTANCES
    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance

    // TUNING CONSTANTS
    public static double INTAKE_TIME = 1;
    public static double SHOT_TIMEOUT = 2.0; // seconds

    // VARIABLES
    private double shotStartTime;
    private boolean shotStarted = false;
    private boolean isReadingAprilTag = false;
    private double tripleShotStartTime;
    private boolean tripleShotStarted = false;

    // Track previous bumper state for edge detection
    private boolean prevRightBumper = false;

    //private clamp test
    private static double clampT(double v, double min, double max) {
        if (Double.isNaN(v)) return min;               // defensive: treat NaN as min
        if (min > max) {                               // tolerate inverted bounds
            double t = min; min = max; max = t;
        }
        return Math.max(min, Math.min(max, v));
    }
    /**
     *
     * @param angle
     * @return normalized angle to (-pi, pi]
     */
    private double normalizeRadians(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
    @Override
    public void initControls() {
        super.initControls();
        follower = Constants.createFollower(hardwareMap);

        // Initialize rubber bands to off
        rubberBands.setPower(0);
        //TrayServo.setPosition(currentTrayPosition); // Prevent movement at init
    }

    @Override
    public void runOpMode() throws InterruptedException {
        float gain = 2;
        initControls();
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();


        waitForStart();
        if (isStopRequested()) return;
        //Start
        follower.startTeleopDrive(true);
        follower.update();

        while (this.opModeIsActive() && !isStopRequested()) {

            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            // -----------------
            // GAMEPAD1 CONTROLS
            // -----------------

            //RubberBands + IntakeRoller CONTROLS:
            if (gamepad1.y) {
                intakeRoller.setPower(INTAKE_INTAKE_ROLLER_POWER);
                rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                topIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
            } else if (gamepad1.a) {
                intakeRoller.setPower(-OUTPUT_INTAKE_ROLLER_POWER);
                rubberBands.setPower(-OUTPUT_RUBBER_BANDS_POWER);
                topIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);
            } else if (gamepad1.x) {
                intakeRoller.setPower(0);
                rubberBands.setPower(0);
                topIntake.setPower(0);
            }

            // Toggle auto-intake on right bumper press (edge triggered)
            if (gamepad1.right_bumper && !prevRightBumper) {
                // toggle the TrayFSM instance (from DarienOpModeFSM)
                trayFSM.toggleAutoIntake();
            }
            prevRightBumper = gamepad1.right_bumper;

            // Show current auto-intake status on telemetry
            telemetry.addData("AutoIntakeRunning", trayFSM != null && trayFSM.isAutoIntakeRunning());

            // Update trayFSM state machine each loop so it can run when toggled on
            trayFSM.update();
            if (!trayFSM.isAutoIntakeRunning()) {
                    shotgunFSM.toPowerUp();
            }


            // -----------------
            // GAMEPAD2 CONTROLS
            // -----------------

            if (!shotStarted && !tripleShotStarted) {
                // -----------------
                // MANUAL CONTROLS: only allowed when not running macros
                // -----------------

                //CONTROL: POINT TURRET TO GOAL
                // ALIGN TO BLUE GOAL
                // TODO: Add controls for aligning to blue goal

                // ALIGN TO RED GOAL
                if (gamepad2.a && !isReadingAprilTag) {
                    //point robot at red goal if gamepad1 right trigger is pressed
                    tagFSM.start(getRuntime());
                    isReadingAprilTag = true;

                } else if (isReadingAprilTag) {
                    tagFSM.update(getRuntime(), true, telemetry);
                    telemetry.addLine("Reading...");

                    if (tagFSM.isDone()) {
                        telemetry.addLine("DONE READING!");
                        isReadingAprilTag = false;
                        aprilTagDetections = tagFSM.getDetections();
                        //aprilTagDetections.removeIf(tag -> tag.id != 24);
                        aprilTagDetections.removeIf(tag -> tag.id == 20 || tag.id == 21 || tag.id == 22 || tag.id == 23);
                        if (!aprilTagDetections.isEmpty()) {
                            telemetry.addLine("FOUND APRILTAG!");
                            tagFSM.telemetryAprilTag(telemetry);
                            // Rotate the turret only if an apriltag is detected and it's the red goal apriltag id
                            AprilTagDetection detection = aprilTagDetections.get(0);
                            if (detection.id == 24) {
                                telemetry.addLine("ALIGNING TO GOAL...");
                                double currentHeading = 0;
                                double relativeHeading = detection.ftcPose.bearing;
                                double targetHeading = normalizeRadians(currentHeading + relativeHeading);
                                currentTurretPosition = clampT(targetHeading, TURRET_ROTATION_MAX_LEFT, TURRET_ROTATION_MAX_RIGHT);

                                turretServo.setPosition(currentTurretPosition);
                            } // end detection.id == 24
                        } // end detection is empty
                    } // end tagFSM is done

                } // end Red Goal April Tag

                //CONTROL: EJECTION MOTORS
                if (!trayFSM.isAutoIntakeRunning()) {
                    if (gamepad2.right_stick_y > 0.05) {
                        // close shot
                        shotgunFSM.toPowerUp();
                    } else if (gamepad2.right_stick_y < -0.05) {
                        // far shot
                        shotgunFSM.toPowerUpFar();
                    }
                } else {
                    shotgunFSM.toOff();
                }

                /*
                if (gamepad2.right_trigger > 0.05 && gamepad2.right_stick_y >= -0.05) {
                    ejectionMotor.setPower(getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP));
                } else if (gamepad2.right_trigger > 0.05 && gamepad2.right_stick_y < -0.05) {
                    ejectionMotor.setPower(getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP_FAR));
                } else if (gamepad2.left_trigger > 0.05) {
                    ejectionMotor.setPower(-getVoltageAdjustedMotorPower(SHOT_GUN_POWER_DOWN));
                } else {
                    ejectionMotor.setPower(0);
                }

                 */

                //CONTROL: ELEVATOR
                if (gamepad2.left_bumper) {
                    Elevator.setPosition(ELEVATOR_POS_UP);
                } else {
                    Elevator.setPosition(ELEVATOR_POS_DOWN);
                }

                // CONTROL: ROTATING TRAY
                if (gamepad2.dpad_left) {
                    setTrayPosition(TRAY_POS_1_INTAKE);
                    //servoIncremental(TrayServo, TRAY_POS_1_INTAKE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_1_INTAKE;
                } else if (gamepad2.x) {
                    setTrayPosition(TRAY_POS_1_SCORE);
                    //servoIncremental(TrayServo, TRAY_POS_1_SCORE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_1_SCORE;
                } else if (gamepad2.dpad_up) {
                    setTrayPosition(TRAY_POS_2_INTAKE);
                    //servoIncremental(TrayServo, TRAY_POS_2_INTAKE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_2_INTAKE;
                } else if (gamepad2.y) {
                    setTrayPosition(TRAY_POS_2_SCORE);
                    //servoIncremental(TrayServo, TRAY_POS_2_SCORE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_2_SCORE;
                } else if (gamepad2.dpad_right) {
                    setTrayPosition(TRAY_POS_3_INTAKE);
                    //servoIncremental(TrayServo, TRAY_POS_3_INTAKE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_3_INTAKE;
                } else if (gamepad2.b && !gamepad2.start) {
                    setTrayPosition(TRAY_POS_3_SCORE);
                    //servoIncremental(TrayServo, TRAY_POS_3_SCORE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_3_SCORE;
                }

                /*
                // CONTROL: ROTATING TRAY USING FSM
                if (gamepad2.dpad_left && gamepad2.back) {
                    setTrayPosition(TRAY_POS_1_INTAKE);
                } else if (gamepad2.x && gamepad2.back) {
                    setTrayPosition(TRAY_POS_1_SCORE);
                } else if (gamepad2.dpad_up && gamepad2.back) {
                    setTrayPosition(TRAY_POS_2_INTAKE);
                } else if (gamepad2.y && gamepad2.back) {
                    setTrayPosition(TRAY_POS_2_SCORE);
                } else if (gamepad2.dpad_right && gamepad2.back) {
                    setTrayPosition(TRAY_POS_3_INTAKE);
                } else if (gamepad2.b && gamepad2.back) {
                    setTrayPosition(TRAY_POS_3_SCORE);
                }

                 */

                // -----------------
                // IMPORTANT: ALWAYS PUT MACRO CONTROLS AFTER MANUAL CONTROLS
                // -----------------

                //CONTROL: START SHOTGUN MACRO USING FSM
                if (gamepad2.dpad_down && gamepad2.right_trigger > 0.05) {
                    // TODO: pre-spin up the shotgun before starting the shooting FSM
                    if (gamepad2.right_stick_y < -0.05) {
                        shootArtifactFSM.startShooting(SHOT_GUN_POWER_UP_FAR);
                    } else {
                        shootArtifactFSM.startShooting(SHOT_GUN_POWER_UP);
                    }
                    shotStartTime = getRuntime();
                    shotStarted = true;
                }

                // CONTROL: START TRIPLE SHOT MACRO USING FSM

                // Edge-triggered start: press right bumper to start triple shoot
                else if (gamepad2.right_bumper && !tripleShotStarted) {
                    // ONLY START IF IN MANUAL CONTROL MODE
                    double power = (gamepad2.right_stick_y < -0.05) ? SHOT_GUN_POWER_UP_FAR : SHOT_GUN_POWER_UP;
                    shootTripleFSM.startShootTriple(getRuntime(), power); // start the 1,2,3 sequence
                    tripleShotStartTime = getRuntime();
                    tripleShotStarted = true;
                }

                telemetry.addData("shotStarted", shotStarted);
                telemetry.addData("tripleShotStarted", tripleShotStarted);

            } //manual controls
            else {
                // -----------------
                // MACRO CONTROLS
                // -----------------

                //CONTROL: SHOTGUN MACRO
                if (shotStarted)  {
                    // ONLY UPDATE IF IN MACRO CONTROL MODE
                    shootArtifactFSM.updateShooting();
                    if (shootArtifactFSM.shootingDone() || getRuntime() - shotStartTime >= SHOT_TIMEOUT) {
                        shootArtifactFSM.resetShooting();
                        shotStarted = false;
                    }
                }

                // CONTROL: TRIPLE SHOT MACRO
                else if (tripleShotStarted) {
                    // ONLY UPDATE IF IN MACRO CONTROL MODE
                    shootTripleFSM.updateShootTriple(getRuntime());
                    if (shootTripleFSM.isDone() || getRuntime() - tripleShotStartTime >= 10) {
                        tripleShotStarted = false;
                    }
                }

                /*
                telemetry.addData("shotStarted", shotStarted);
                telemetry.addData("shootingStage", shootArtifactFSM.getShootingStage());
                telemetry.addData("shootingDone", shootArtifactFSM.shootingDone());
                telemetry.addData("Elevator Pos", Elevator.getPosition());
                telemetry.addData("Ejection L", ejectionMotorLeft.getPower());
                telemetry.addData("Ejection R", ejectionMotorRight.getPower());

                 */

            } //macro controls

            //turret rotation
            if (gamepad2.left_stick_x <=-0.05) {    //turn turret clockwise
                //updating the current turret position to be in range of the min and max
                currentTurretPosition = clampT(currentTurretPosition + TURRET_ROTATION_INCREMENT, TURRET_ROTATION_MAX_LEFT, TURRET_ROTATION_MAX_RIGHT);
                //sets turret position
                turretServo.setPosition(currentTurretPosition);
            }
            else if (gamepad2.left_stick_x >= 0.05) {   //turn turret counterclockwise
                //updating the current turret position to be in range of the min and max
                currentTurretPosition = clampT(currentTurretPosition - TURRET_ROTATION_INCREMENT, TURRET_ROTATION_MAX_LEFT, TURRET_ROTATION_MAX_RIGHT);
                //sets turret position
                turretServo.setPosition(currentTurretPosition);
            }
            /*    telemetry.addData("TurretPos", "%.3f", currentTurretPosition);
                telemetry.addData("Turret Min/Max/Inc", "%.3f / %.3f / %.3f", TURRET_ROTATION_MIN, TURRET_ROTATION_MAX, TURRET_ROTATION_INCREMENT); */

            telemetry.update();
        } //while opModeIsActive
    } //runOpMode
} //TeleOpFSM class