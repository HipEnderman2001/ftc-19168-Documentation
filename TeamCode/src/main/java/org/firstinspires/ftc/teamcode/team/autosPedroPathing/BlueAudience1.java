package org.firstinspires.ftc.teamcode.team.autosPedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

@Autonomous(name = "BlueAudienceSidePedro", group = "Pedro:Blues", preselectTeleOp = "TeleopFSM")
@Configurable
public class BlueAudience1 extends DarienOpModeFSM {
    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance
    private int pathState;                      // State machine state
    private Paths paths;                        // Paths
    private Timer pathTimer, opmodeTimer;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- ROBOT + HARDWARE INIT (from DarienOpModeFSM) ---
        initControls(); // sets up TrayServo, Elevator, Feeder, motors, AprilTag, etc.

        // --- PEDRO + TIMERS INIT ---
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Starting pose – same as your OpMode version
        follower.setStartingPose(new Pose(56, 9, Math.toRadians(90)));

        // Build all the paths once
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        telemetry.addLine("BlueGoalSidePedro: READY");
        telemetry.update();

        // --- WAIT FOR START ---
        waitForStart();
        if (isStopRequested()) return;

        opmodeTimer.resetTimer();
        setPathState(0);

        // Set the initial tray position immediately.
        setTrayPosition(TRAY_POS_1_SCORE);

        // --- MAIN AUTONOMOUS LOOP ---
        while (opModeIsActive() && !isStopRequested()) {

            // Pedro follower must be updated every loop
            follower.update();

            // Drive the state machine
            pathState = autonomousPathUpdate();

            runIntakeLifterWithColorSensor();

            /*
            // Update tray servo FSM if running
            if (trayServoFSM.isRunning()) {
                trayServoFSM.update(getRuntime());
                if (!trayServoFSM.isRunning()) {
                    // Update current tray position when done
                    currentTrayPosition = targetTrayPosition;
                }
            }

             */

            // Panels/driver telemetry
            panelsTelemetry.addData("Tray Curr", currentTrayPosition);
            //panelsTelemetry.addData("Tray Targ", targetTrayPosition);
            panelsTelemetry.addData("Path State", pathState);
            panelsTelemetry.addData("X", follower.getPose().getX());
            panelsTelemetry.addData("Y", follower.getPose().getY());
            panelsTelemetry.addData("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);

            telemetry.update();
        }
    }


    public static class Paths {

        public PathChain ShootingPosition;
        public PathChain IntakePosition;
        public PathChain Intake1;
        public PathChain Intake2;
        public PathChain Intake3;
        public PathChain ShootingPosition2;
        public PathChain Parking;

        public Paths(Follower follower) {
            ShootingPosition = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 9.000), new Pose(56.000, 18.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112))
                    .build();

            IntakePosition = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 18.000), new Pose(42.000, 35.750))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180))
                    .build();

            Intake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 35.750), new Pose(34.000, 35.750))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Intake2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(34.000, 35.750), new Pose(29.000, 35.750))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Intake3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(29.000, 35.750), new Pose(24.000, 35.750))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ShootingPosition2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.000, 35.750),
                                    new Pose(45.000, 29.000),
                                    new Pose(56.000, 18.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113.5))
                    .build();

            Parking = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 18.000), new Pose(56.000, 29.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113.5), Math.toRadians(90))
                    .build();
        }
    }


//67

    public int autonomousPathUpdate() {
        telemetry.addData("PathState", pathState);
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());

        switch (pathState) {
            case 0:
                //start reading april tags
                telemetry.addLine("Case " + pathState + ": Wait for Camera");

                // Set the initial tray position
                setTrayPosition(TRAY_POS_1_SCORE);
                tagFSM.start(getRuntime());
                follower.setMaxPower(.8); //normal speed
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {

                    telemetry.addLine("Case " + pathState + ": exiting");

                    setPathState(pathState + 1);
                }
                break;

            case 1:
                //once april tags done reading, move to shooting position 1
                telemetry.addLine("Case " + pathState + ":");

                tagFSM.update(getRuntime(), true, telemetry);

                if ((tagFSM.isDone()) || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    aprilTagDetections = tagFSM.getDetections();
                    aprilTagDetections.removeIf(tag -> tag.id == 20 || tag.id == 24);
                    follower.followPath(paths.ShootingPosition);

                    setPathState(pathState + 1);
                }
                break;

            case 2:
                //move to shooting position 1
                telemetry.addLine("Case " + pathState + ": wait for Path 1...");

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {

                    setPathState(pathState + 1);
                }
                break;

            case 3:
                //once at shooting position 1, shoot artifacts set 1
                telemetry.addLine("Case " + pathState + ": start shooting...");

                shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), SHOT_GUN_POWER_UP_FAR);

                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(pathState + 1);
                }
                break;

            case 4:
                //once artifacts set 1 shot, move to intake position
                telemetry.addLine("Case " + pathState + ": Update shooting...");

                shootPatternFSM.updateShootPattern(getRuntime());

                if (shootPatternFSM.isShootPatternDone() || pathTimer.getElapsedTimeSeconds() > 10.0) {

                    rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                    setTrayPosition(TRAY_POS_2_INTAKE);
                    follower.followPath(paths.IntakePosition, true);
                    setPathState(pathState + 1);
                }
                break;

            case 5:
                //when in position, go to intake position 1
                telemetry.addLine("Case " + pathState + ": Going to intake position 1");

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1.5) {

                    follower.setMaxPower(0.175); //slow down for pickup
                    follower.followPath(paths.Intake1, true);
                    setPathState(pathState + 1);
                }
                break;

            case 6:
                //when ball 1 intaken, move to intake position 2
                telemetry.addLine("Case " + pathState + ": Intaking ball 1g");

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.3) {

                    setTrayPosition(TRAY_POS_3_INTAKE);
                    follower.followPath(paths.Intake2, true);
                    setPathState(pathState + 1);
                }
                break;

            case 7:
                //move to intake position 3
                telemetry.addLine("Case " + pathState + ": Intaking ball 3p");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.3) {

                    setTrayPosition(TRAY_POS_1_INTAKE);
                    follower.followPath(paths.Intake3, true);
                    setPathState(pathState + 1);
                }
                break;

            case 8:
                //once ball 3p intaken, move to shooting position 2
                telemetry.addLine("Case " + pathState + ": Move to shoot position 2");

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.3) {

                    follower.setMaxPower(.8); //reset to normal speed
                    shootArtifactFSM.shotGun(SHOT_GUN_POWER_UP_FAR);
                    follower.followPath(paths.ShootingPosition2, true);
                    setPathState(pathState + 1);
                }
                break;

            case 9:
                //once at shooting position 2, shoot artifacts set 2
                telemetry.addLine("Case " + pathState + ": Wait for ShootingPosition, then shoot artifact");


                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.0) {
                    shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), SHOT_GUN_POWER_UP_FAR);
                    setPathState(pathState + 1);
                }
                break;

            case 10:
                //once artifacts set 2 shot, move to parking
                telemetry.addLine("Case " + pathState + ": Update shooting");

                shootPatternFSM.updateShootPattern(getRuntime());

                if (shootPatternFSM.isShootPatternDone() || pathTimer.getElapsedTimeSeconds() > 10.0) {
                    rubberBands.setPower(0); //stop intake
                    follower.followPath(paths.Parking, true);
                    setPathState(-1);
                }
                break;


            default:
                // -1 or any undefined state: do nothing, stay idle
                telemetry.addLine("Idle state (pathState = " + pathState + ")");
                break;
        }

        return pathState;
    }

    /**
     * Sets the path state and resets its timer.
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}