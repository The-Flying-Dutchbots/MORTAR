package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSys;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSys;
import org.firstinspires.ftc.teamcode.subsystems.VisionSys;

@Autonomous(name = "PedroRedFar", group = "Autonomous")
@Configurable // Panels
public class PedroRedFarAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 1; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    ShooterSys shooterSys = new ShooterSys();
    IntakeSys intakeSys = new IntakeSys();
    VisionSys visionSys = new VisionSys();
    ElapsedTime utilityTimer = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(81.4, 8.7, Math.toRadians(-90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        intakeSys.init(hardwareMap);
        shooterSys.init(hardwareMap);
        visionSys.init(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("shooterRPM",shooterSys.getShooterRPM());
        panelsTelemetry.update(telemetry);

    }

    public static class Paths {

        public PathChain ShootPreloads;
        public PathChain IntakeRightSetup;
        public PathChain IntakeRightLoad;
        public PathChain RightToScore;
        public PathChain IntakeWallSetup;
        public PathChain IntakeWallLoad;
        public PathChain WallToShoot;
        public PathChain LEAVE;
        public PathChain LEAVEFR;

        public Paths(Follower follower) {
            ShootPreloads = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(81.400, 8.721), new Pose(83.725, 13.373))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-120))
                    .build();

            IntakeRightSetup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(83.725, 13.373),
                                    new Pose(83.144, 24.420),
                                    new Pose(99.230, 35.855)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-120), Math.toRadians(0))
                    .build();

            IntakeRightLoad = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(99.230, 35.855), new Pose(133.147, 35.661))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            RightToScore = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.147, 35.661),
                                    new Pose(98.649, 34.304),
                                    new Pose(84.307, 15.311)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-116))
                    .build();

            IntakeWallSetup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(84.307, 15.311),
                                    new Pose(104.269, 27.908),
                                    new Pose(134.371, 24.614)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-116), Math.toRadians(-90))
                    .build();

            IntakeWallLoad = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(134.371, 24.614), new Pose(133.790, 11.047))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                    .build();

            WallToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.790, 11.047),
                                    new Pose(119.580, 20.156),
                                    new Pose(84.501, 15.311)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-115))
                    .build();

            LEAVE = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(84.501, 15.311), new Pose(86.633, 37.987))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-115))
                    .build();

            LEAVEFR = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.633, 37.987), new Pose(93.610, 42.832))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 1: // move to preload shooting position
                shooterSys.startShooting();
                shooterSys.setHoodState(ShooterSys.HoodState.FAR);
                utilityTimer.reset();
                follower.followPath(paths.ShootPreloads);
                pathState++;
                break;

            case 2: // start intake to shoot preloads
                shooterSys.startShooting();
                if (!follower.isBusy() && utilityTimer.seconds() > 2.0) {
                    utilityTimer.reset();
                    intakeSys.shootStart();
                    pathState++;
                }
                break;
            case 3: // wait to shoot balls then go to the left intake position
                shooterSys.startShooting();
                intakeSys.shootStart();
                if (utilityTimer.seconds() > 4.5) {
                    follower.followPath(paths.IntakeRightSetup);
                    intakeSys.intakeStop();
                    shooterSys.stopShooting();
                    pathState++;
                }
                break;
            case 4: // begin intake
                intakeSys.intakeStart();
                if(!follower.isBusy()){
                    follower.followPath(paths.IntakeRightLoad);
                    follower.setMaxPower(0.4);
                    pathState++;
                }
                break;
            case 5:
                intakeSys.intakeStart();
                if(!follower.isBusy()){

                    follower.followPath(paths.RightToScore);
                    utilityTimer.reset();
                    follower.setMaxPower(1);
                    pathState++;
                    shooterSys.startShooting();
                }
                break;
            case 6:
                intakeSys.intakeStop();
                shooterSys.startShooting();
                if(!follower.isBusy() && utilityTimer.seconds() >3.0){
                    utilityTimer.reset();
                    intakeSys.shootStart();
                    shooterSys.startShooting();
                    pathState++;
                }
                break;
            case 7:
                shooterSys.startShooting();
                if(utilityTimer.seconds() > 4.5){
                    intakeSys.intakeStop();
                    shooterSys.stopShooting();
                    follower.followPath(paths.IntakeWallSetup);
                    pathState++;
                }
                break;
            case 8: // begin intake
                intakeSys.intakeStart();
                if(!follower.isBusy()){
                    follower.followPath(paths.IntakeWallLoad);
                    utilityTimer.reset();
                    follower.setMaxPower(0.6);
                    pathState++;
                }
                break;
            case 9:
                intakeSys.intakeStart();
                if(!follower.isBusy() || utilityTimer.seconds() > 2.3){
                    intakeSys.intakeStop();
                    shooterSys.startShooting();
                    follower.followPath(paths.WallToShoot);
                    follower.setMaxPower(1);
                    utilityTimer.reset();
                    pathState++;
                }
                break;
            case 10:
                intakeSys.intakeStart();
                shooterSys.startShooting();
                if(!follower.isBusy() && utilityTimer.seconds() >3.0){
                    utilityTimer.reset();
                    pathState++;
                }
                break;
            case 11:
                intakeSys.shootStart();
                if(utilityTimer.seconds() >4.2){
                    intakeSys.intakeStop();
                    shooterSys.stopShooting();
                    pathState++;
                }
                break;
            case 12:
                follower.followPath(paths.LEAVE);
                shooterSys.setHoodState(ShooterSys.HoodState.STOWED);
                pathState++;
                break;
            case 13:
                if(!follower.isBusy()){
                    follower.followPath(paths.LEAVEFR);
                    pathState++;
                }
                break;

        }
        shooterSys.updateHood();

        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}