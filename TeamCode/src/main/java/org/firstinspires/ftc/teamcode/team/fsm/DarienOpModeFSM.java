package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

/**
 * Base OpMode for Pedro pathing and state machine logic.
 * Extend this class for autonomous OpModes using Pedro pathing.
 */
@Config
@Configurable
public abstract class DarienOpModeFSM extends LinearOpMode {

    // Pedro pathing/state machine FSMs (declare as needed)
    // public PathFollowerFSM pathFollowerFSM;
    public AprilTagDetectionFSM tagFSM;
    public ShootPatternFSM shootPatternFSM;
    public ShootArtifactFSM shootArtifactFSM;

    // AprilTag
    public ArrayList<AprilTagDetection> aprilTagDetections;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal = null;

    // telemetry
    public TelemetryPacket tp;
    public FtcDashboard dash;

    // HARDWARE DEVICES
    //public DcMotor omniMotor0, omniMotor1, omniMotor2, omniMotor3;
    public Servo TrayServo, Elevator, Feeder, IntakeServo;
    public CRServo rubberBands;
    public DcMotor ejectionMotorRight, ejectionMotorLeft;
    public NormalizedColorSensor intakeColorSensor;

    // HARDWARE FIXED CONSTANTS
    public static final double encoderResolution = 537.7; //no change unless we change motors
    public static final double wheelDiameter = 3.75; // inches
    public static final double constMult = (wheelDiameter * (Math.PI));
    public static final double inchesToEncoder = encoderResolution / constMult;
    public static final double PI = 3.1416;

    // HARDWARE TUNING CONSTANTS
    public static final double INTAKE_DISTANCE = 5;
    public static final double INTAKE_SERVO_POS_UP = 0.75;
    public static final double INTAKE_SERVO_POS_DOWN = 0.21;
    public static double INTAKE_SERVO_DURATION_RAISE = 1; // seconds
    public static double COLOR_SENSOR_TIMEOUT = 2; // seconds
    public static double TRAY_SERVO_DURATION_ROTATE = 1.5; // seconds
    public static double TRAY_POS_1_INTAKE = 0.275;
    public static double TRAY_POS_2_INTAKE = 0.205;
    public static double TRAY_POS_3_INTAKE = 0.350;
    public static double TRAY_POS_1_SCORE = 0.385;
    public static double TRAY_POS_2_SCORE = 0.310;
    public static double TRAY_POS_3_SCORE = 0.240;
    public static final double ELEVATOR_POS_UP = 0.83;
    public static final double ELEVATOR_POS_DOWN = 0.45;
    public static final double FEEDER_POS_UP = .9;
    public static double FEEDER_POS_DOWN = .55;
    public static double SHOT_GUN_POWER_UP = 0.32; // tuned to 6000 rpm motor at a distance of 39 inches from the front of the robot to the goal wall.
    public static double SHOT_GUN_POWER_UP_FAR = 0.40; // tuned to 6000 rpm motor
    public static double SHOT_GUN_POWER_DOWN = 0.2; // tuned to 6000 rpm motor
    public static final double TIMEOUT_APRILTAG_DETECTION = 3;
    public static double INTAKE_RUBBER_BANDS_POWER = -0.7;
    public static double OUTPUT_RUBBER_BANDS_POWER = 0.2;

    public double IntakeServoPosition = 0;
    public double currentTrayPosition;
    //public double targetTrayPosition = 0.0;
    public double startTimeIntakeColorSensor;
    public boolean intakeLifted = false;
    public double intakeLiftStartTime = 0;

    // Abstract method for child classes to implement
    @Override
    public abstract void runOpMode() throws InterruptedException;

    public ServoIncrementalFSM intakeServoFSM;
    //public ServoIncrementalFSM trayServoFSM;

    public void initControls() {

        // INITIALIZE SENSORS

        // Initialize 2 Deadwheel odometry
        // configure2DeadWheel();

        //TELEMETRY
        // TODO: Put a flag to turn on/off ftc dashboard. We don't want that to run during matches.
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();

        // INITIALIZE SERVOS
        TrayServo = hardwareMap.get(Servo.class, "Tray");
        Elevator = hardwareMap.get(Servo.class, "Elevator");
        IntakeServo = hardwareMap.get(Servo.class, "intakeServo");
        Feeder = hardwareMap.get(Servo.class, "Feeder");
        rubberBands = hardwareMap.get(CRServo.class, "rubberBands");

        // INITIALIZE SENSORS
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");

        // INITIALIZE MOTORS
        //ejectionMotorRight = initializeMotor("ejectionMotorRight");
        ejectionMotorRight = hardwareMap.get(DcMotor.class, "ejectionMotorRight");
        ejectionMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //ejectionMotorLeft = initializeMotor("ejectionMotorLeft");
        ejectionMotorLeft = hardwareMap.get(DcMotor.class, "ejectionMotorLeft");
        ejectionMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        initAprilTag();

        // INSTANTIATE THE STATE MACHINES
        tagFSM = new AprilTagDetectionFSM(aprilTag, TIMEOUT_APRILTAG_DETECTION);
        shootArtifactFSM = new ShootArtifactFSM(this);
        shootPatternFSM = new ShootPatternFSM(this);

        intakeServoFSM = new ServoIncrementalFSM(IntakeServo);
        //trayServoFSM = new ServoIncrementalFSM(TrayServo);
        //currentTrayPosition = TRAY_POS_1_SCORE; // set a default tray position

        startTimeIntakeColorSensor = getRuntime();

        telemetry.addLine("FTC 19168 Robot Initialization Done!");
        telemetry.update();
    }

    /**
     * Set the tray position and update the currentTrayPosition variable.
     *
     * @param position The desired position for the tray servo.
     * @param duration The duration over which to move the tray servo to the desired position.
     */
    public void setTrayPosition(double position, double duration) {
        TrayServo.setPosition(position);
        //servoIncremental(TrayServo, position, currentTrayPosition, duration, 4);
        currentTrayPosition = position;
        /*
        if (!trayServoFSM.isRunning()) {
            targetTrayPosition = position;
            trayServoFSM.start(position, currentTrayPosition, duration, getRuntime());
        }
         */
    }

    /**
     * Set the tray position with a default duration in seconds.
     *
     * @param position The desired position for the tray servo.
     */
    public void setTrayPosition(double position) {
        setTrayPosition(position, TRAY_SERVO_DURATION_ROTATE);
    }

    public void runIntakeLifterWithColorSensor() {
        if (intakeServoFSM.isRunning()) {
            intakeServoFSM.update(getRuntime());
        } else {
            if (intakeColorSensor instanceof DistanceSensor) {
                if (((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM) <= INTAKE_DISTANCE && (getRuntime() - startTimeIntakeColorSensor) >= COLOR_SENSOR_TIMEOUT) {
                    startTimeIntakeColorSensor = getRuntime();
                    intakeServoFSM.start(INTAKE_SERVO_POS_UP, INTAKE_SERVO_POS_DOWN, INTAKE_SERVO_DURATION_RAISE, getRuntime());
                } else {
                    IntakeServo.setPosition(INTAKE_SERVO_POS_DOWN);
                }
            }
        }
    }

    public DcMotor initializeMotor(String name) {
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    public void servoIncremental(Servo servo, double endPos, double startPos, double endDuration, double divisor) {
        //calculate how many increments it will take to reach to position in the target time
        double currentPos;
        double startTime = getRuntime();
        double currentTime = startTime;

        while (currentTime - startTime < endDuration) {
            if (endPos > startPos) {
                // rotate tray clockwise
                currentPos = ((endPos - startPos) / (endDuration - (currentTime - startTime))) * (currentTime - startTime) + startPos;
            } else {
                // rotate tray counterclockwise
                currentPos = ((startPos - endPos) / (endDuration - (currentTime - startTime))) * (currentTime - startTime) + endPos;
            }
            servo.setPosition(currentPos / divisor);

            //telemetry.addData("currentPos:", currentPos);
            //telemetry.addData("currentTime:", currentTime);
            //telemetry.update();
            //tp.put("currentServo", currentPos);
            //tp.put("currentTime", currentTime);

            //dash.sendTelemetryPacket(tp);

            if (currentPos >= endPos) {
                return;
            }
            currentTime = getRuntime();
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

    }

    public double getVoltageAdjustedMotorPower(double power) {
        double nominalVoltage = 13.0; // Typical full battery voltage for FTC
        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double scale = nominalVoltage / currentVoltage;
        return power * scale;
    }
}
