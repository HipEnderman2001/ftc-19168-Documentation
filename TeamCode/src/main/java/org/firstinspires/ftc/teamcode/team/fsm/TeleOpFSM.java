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
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

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
    private boolean prevRightBumper1 = false;
    //private boolean prevRightBumper2 = false;
    private boolean prevBackButton = false;
    private ShotgunPowerLevel shotgunPowerLatch = ShotgunPowerLevel.LOW;

    // AUTOMATIC TURRET CONTROLS BASED ON CAMERA APRILTAG DETECTION
    AprilTagDetection detection;
    double yaw; // Stores detection.ftcPose.yaw
    double currentHeadingDeg;
    double relativeHeadingDeg; // Camera-relative bearing to AprilTag (degrees)
    double targetServoPos = Double.NaN; // Convert heading → servo position
    double rawBearingDeg; // Stores detection.ftcPose.bearing;

    // cameraOffsetX < 0 if camera is mounted on the LEFT
   // public static double cameraOffsetX = 0.105; // in centimeter, positive is right, negative is left
    double correctedBearingRad;
    double correctedBearingDeg;
    boolean isCalculatingTurretTargetPosition = false;

    int targetGoalTagId;

    // PIDF Tuning values for ejection motor
    /*
    public static double NEW_P = 0.1;
    public static double NEW_I = 0.1;
    public static double NEW_D = 0;
    public static double NEW_F = 0;

     */

    //private clamp test
    private static double clampT(double v, double min, double max) {
        if (Double.isNaN(v)) return min;               // defensive: treat NaN as min
        if (min > max) {                               // tolerate inverted bounds
            double t = min; min = max; max = t;
        }
        return Math.max(min, Math.min(max, v));
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

        //PIDFCoefficients pidfOrig = ejectionMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Change coefficients using methods included with DcMotorEx class.
        //PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        //ejectionMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        // Re-read coefficients and verify change.
        //PIDFCoefficients pidfModified = ejectionMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);


        while (this.opModeIsActive() && !isStopRequested()) {

            // -----------------
            // ALWAYS RUN
            // -----------------
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            leftIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
            rightIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);

            // -----------------
            // GAMEPAD1 CONTROLS
            // -----------------

            //RubberBands + topIntake CONTROLS:
            if (gamepad1.y) {
                // Intake mode
                rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                topIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                //leftIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                //rightIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);
            } else if (gamepad1.a) {
                // Eject mode
                rubberBands.setPower(-INTAKE_RUBBER_BANDS_POWER);
                topIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);
                //leftIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                //rightIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);
            } else if (gamepad1.x) {
                rubberBands.setPower(0);
                topIntake.setPower(0);
                //leftIntake.setPower(0);
                //rightIntake.setPower(0);
            }

            // Toggle auto-intake on right bumper press (edge triggered)
            if (gamepad1.right_bumper && !prevRightBumper1) {
                // toggle the TrayFSM instance (from DarienOpModeFSM)
                trayFSM.toggleAutoIntake();
            }
            prevRightBumper1 = gamepad1.right_bumper;

            // Toggle auto-intake on back button press
            if (gamepad2.back && !prevBackButton) {
                // toggle the TrayFSM instance (from DarienOpModeFSM)
                trayFSM.toggleAutoIntake();
            }
            prevBackButton = gamepad2.back;

            // Show current auto-intake status on telemetry
            telemetry.addData("AutoIntakeRunning", trayFSM != null && trayFSM.isAutoIntakeRunning());

            // Update trayFSM state machine each loop so it can run when toggled on
            trayFSM.update();
            if (!trayFSM.isAutoIntakeRunning() && shotgunPowerLatch != ShotgunPowerLevel.OFF) {
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
                if (gamepad2.b && !isReadingAprilTag) {
                    // ALIGN TO RED GOAL
                    tagFSM.start(getRuntime());
                    isReadingAprilTag = true;
                    targetGoalTagId = APRILTAG_ID_GOAL_RED;
                    telemetry.addLine("ALIGN TURRET TO RED!");
                } else if (gamepad2.x && !isReadingAprilTag) {
                    // ALIGN TO BLUE GOAL
                    tagFSM.start(getRuntime());
                    isReadingAprilTag = true;
                    targetGoalTagId = APRILTAG_ID_GOAL_BLUE;
                    telemetry.addLine("ALIGN TURRET TO BLUE!");
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
                            detection = aprilTagDetections.get(0);
                            if (detection.id == targetGoalTagId) {
                                telemetry.addLine("ALIGNING TO GOAL...");
                                yaw = detection.ftcPose.yaw; // TODO: REMOVE LATER SINCE IT'S ONLY FOR TELEMETRY

                                // Current turret heading (degrees)
                                currentHeadingDeg = turretFSM.getTurretHeading(); // TODO: REMOVE LATER SINCE IT'S ONLY FOR TELEMETRY

                                // Camera-relative bearing to AprilTag (degrees)
                                rawBearingDeg = detection.ftcPose.bearing;

                                if (!isCalculatingTurretTargetPosition) {
                                    isCalculatingTurretTargetPosition = true;
                                    targetServoPos = currentTurretPosition + RATIO_BETWEEN_TURRET_GEARS * rawBearingDeg / FIVE_ROTATION_SERVO_SPAN_DEG;
                                    //targetServoPos = Range.clip(targetServoPos, TURRET_ROTATION_MAX_LEFT, TURRET_ROTATION_MAX_RIGHT);
                                }
                            } // end detection.id == 20 or 24
                        } // end detection is empty
                    } // end tagFSM is done
                    isCalculatingTurretTargetPosition = false;

                } // end Red Goal April Tag

                telemetry.addData("yaw", yaw);
                telemetry.addData("Raw Bearing Deg (alpha)", rawBearingDeg);
                telemetry.addData("currentHeadingDeg (C0)", currentHeadingDeg);
                telemetry.addData("currentTurretPosition", currentTurretPosition);
                telemetry.addData("targetServoPos", targetServoPos);

                //CONTROL: ELEVATOR
                if (gamepad2.left_bumper) {
                    Elevator.setPosition(ELEVATOR_POS_UP);
                } else {
                    Elevator.setPosition(ELEVATOR_POS_DOWN);
                }
                // CONTROL: ROTATING TRAY USING FSM
                if (gamepad2.dpad_left) {
                    setTrayPosition(TRAY_POS_1_SCORE);
                } else if (gamepad2.dpad_up) {
                    setTrayPosition(TRAY_POS_2_SCORE);
                } else if (gamepad2.dpad_right) {
                    setTrayPosition(TRAY_POS_3_SCORE);
                }

                // -----------------
                // IMPORTANT: ALWAYS PUT MACRO CONTROLS AFTER MANUAL CONTROLS
                // -----------------

                //CONTROL: START SHOTGUN MACRO USING FSM
                if (gamepad2.dpad_down && gamepad2.right_stick_y < -0.05) {
                    shootArtifactFSM.startShooting(SHOT_GUN_POWER_UP_FAR);
                    shotStartTime = getRuntime();
                    shotStarted = true;
                } else if (gamepad2.dpad_down) {
                    shootArtifactFSM.startShooting(SHOT_GUN_POWER_UP);
                    shotStartTime = getRuntime();
                    shotStarted = true;
                }

                // CONTROL: START TRIPLE SHOT MACRO USING FSM
                // Toggle auto-intake on right bumper press (edge triggered)
                /*
                if (gamepad1.right_bumper && !prevRightBumper1) {
                    // toggle the TrayFSM instance (from DarienOpModeFSM)
                    ShootTripleFSM.toggle();
                }
                prevRightBumper2 = gamepad2.right_bumper;

                 */


                // Edge-triggered start: press right bumper to start triple shoot
                else if (gamepad2.right_bumper && gamepad2.right_stick_y < -0.05) {
                    shootTripleFSM.startShootTriple(getRuntime(), SHOT_GUN_POWER_UP_FAR);
                    tripleShotStartTime = getRuntime();
                    tripleShotStarted = true;
                } else if (gamepad2.right_bumper) {
                    shootTripleFSM.startShootTriple(getRuntime(), SHOT_GUN_POWER_UP);
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
            else if (gamepad2.leftStickButtonWasPressed() && !Double.isNaN(targetServoPos)) {
                currentTurretPosition = targetServoPos;
                turretServo.setPosition(targetServoPos);
            }
            /*    telemetry.addData("TurretPos", "%.3f", currentTurretPosition);
                telemetry.addData("Turret Min/Max/Inc", "%.3f / %.3f / %.3f", TURRET_ROTATION_MIN, TURRET_ROTATION_MAX, TURRET_ROTATION_INCREMENT); */

            //CONTROL: EJECTION MOTORS
            if (!trayFSM.isAutoIntakeRunning()) {
                //Latch control
                if (gamepad2.right_stick_y < -.05) {
                    shotgunPowerLatch = ShotgunPowerLevel.HIGH;
                } else if (gamepad2.right_stick_y > 0.05) {
                    shotgunPowerLatch = ShotgunPowerLevel.LOW;
                } else if (gamepad2.rightStickButtonWasPressed()) {
                    shotgunPowerLatch = ShotgunPowerLevel.OFF;
                }
                switch (shotgunPowerLatch) {
                    case OFF:
                        shotgunFSM.toOff();
                        telemetry.addData("Requested ShotGun RPM", 0);
                        break;
                    case HIGH:
                        shotgunFSM.toPowerUpFar(SHOT_GUN_POWER_UP_FAR_RPM_TELEOP);
                        telemetry.addData("Requested ShotGun RPM", SHOT_GUN_POWER_UP_FAR_RPM_TELEOP);
                        break;
                    default:
                    case LOW:
                        shotgunFSM.toPowerUp();
                        telemetry.addData("Requested ShotGun RPM", SHOT_GUN_POWER_UP_RPM);
                        break;
                }
                /*
                if (shotgunPowerLatch == DarienOpModeFSM.shotgunPowerLevel.HIGH) {
                    shotgunFSM.toPowerUpFar();
                    telemetry.addData("Requested ShotGun RPM", SHOT_GUN_POWER_UP_FAR_RPM);
                } else if (shotgunPowerLatch == DarienOpModeFSM.shotgunPowerLevel.OFF){
                    shotgunFSM.toOff();
                    telemetry.addData("Requested ShotGun RPM", 0);
                } else {
                    shotgunFSM.toPowerUp();
                    telemetry.addData("Requested ShotGun RPM", SHOT_GUN_POWER_UP_RPM);
                }

                 */

                /*
                if (gamepad2.right_stick_y > 0.05) {
                    // close shot
                    shotgunFSM.toPowerUp();
                    telemetry.addData("Requested ShotGun RPM", SHOT_GUN_POWER_UP_RPM);
                    isHighPower = false;
                } else if (gamepad2.right_stick_y < -0.05 && isHighPower) {
                    // far shot
                    shotgunFSM.toPowerUpFar();
                    telemetry.addData("Requested ShotGun RPM", SHOT_GUN_POWER_UP_FAR_RPM);
                    isHighPower = true;
                }

                 */
            } else {
                shotgunFSM.toOff();
            }
            telemetry.addData("Actual ShotGun RPM", ejectionMotor.getVelocity() * 60 / TICKS_PER_ROTATION); // convert from ticks per second to RPM
            telemetry.addData("ejectionMotor power", ejectionMotor.getPower());
            telemetry.addData("Actual ShotGun TPS", ejectionMotor.getVelocity()); // convert from ticks per second to RPM

            /*
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                    pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);

             */

            telemetry.update();
        } //while opModeIsActive
    } //runOpMode
} //TeleOpFSM class