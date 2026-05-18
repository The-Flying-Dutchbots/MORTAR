package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSys;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSys;
import org.firstinspires.ftc.teamcode.subsystems.VisionSys;



@Autonomous(name = "PedroRedCloseAutoGOLDEN", group = "Autonomous")
@Configurable // Panels
public class PedroRedCloseGOLDEN extends OpMode {

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
        follower.setStartingPose(new Pose(125.5, 120.7, Math.toRadians(-144)));

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
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain ScorePreloadsClose;
        public PathChain IntakeLeftSetup;
        public PathChain IntakeLeftLoad;
        public PathChain LeftToScore;
        public PathChain IntakeMiddleSetup;
        public PathChain IntakeMiddleLoad;
        public PathChain MiddleToScore;
        public PathChain LEAVE;
        public PathChain LEAVEFR;

        public Paths(Follower follower) {
            ScorePreloadsClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.588, 120.743), new Pose(109.991, 109.215))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-144), Math.toRadians(-135))
                    .build();

            IntakeLeftSetup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(112.991, 112.215),
                                    new Pose(97.873, 105.626),
                                    new Pose(97.098, 84.501)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(0))
                    .build();

            IntakeLeftLoad = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(97.098, 84.501), new Pose(128.108, 83.144))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            LeftToScore = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(128.108, 83.144), new Pose(83.338, 82.950))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-144)) //make smaller if miss right
                    .build();

            IntakeMiddleSetup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(83.338, 82.950),
                                    new Pose(87.795, 67.445),
                                    new Pose(95.160, 60.081)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-142), Math.toRadians(0))
                    .build();

            IntakeMiddleLoad = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.160, 60.081), new Pose(129.464, 58.530))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            MiddleToScore = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(129.464, 58.530),
                                    new Pose(96.323, 69.384),
                                    new Pose(82.950, 82.563)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-142))
                    .build();

            LEAVE = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(82.950, 82.563), new Pose(99.991, 109.215))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-142), Math.toRadians(0))
                    .build();

            LEAVEFR = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(99.991, 109.215), new Pose(90.439, 109.855))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }





    public int autonomousPathUpdate() {
        switch (pathState) {
            case 1: // move to preload shooting position
                shooterSys.startShooting();
                follower.followPath(paths.ScorePreloadsClose);
                pathState++;
                break;

            case 2: // start intake to shoot preloads
                if (!follower.isBusy()) {
                    utilityTimer.reset();
                    intakeSys.shootStart();
                    pathState++;
                }
                break;
            case 3: // wait to shoot balls then go to the left intake position
                shooterSys.startShooting();
                intakeSys.shootStart();
                if (utilityTimer.seconds() > 5.0) {
                    follower.followPath(paths.IntakeLeftSetup);
                    shooterSys.setHoodState(ShooterSys.HoodState.MID);
                    intakeSys.intakeStop();
                    shooterSys.stopShooting();
                    pathState++;
                }
                break;
            case 4: // begin intake
                intakeSys.intakeStart();
                if(!follower.isBusy()){
                    follower.followPath(paths.IntakeLeftLoad);
                    follower.setMaxPower(0.4);
                    pathState++;
                }
                break;
            case 5:
                intakeSys.intakeStart();
                if(!follower.isBusy()){

                    follower.followPath(paths.LeftToScore);
                    follower.setMaxPower(1);
                    pathState++;
                }
                break;
            case 6:
                intakeSys.intakeStart();
                shooterSys.startShooting();
                if(!follower.isBusy()){
                    utilityTimer.reset();
                    intakeSys.shootStart();
                    pathState++;
                }
                break;
            case 7:
                shooterSys.startShooting();
                if(utilityTimer.seconds() > 5.0){
                    intakeSys.intakeStop();
                    shooterSys.stopShooting();
                    follower.followPath(paths.IntakeMiddleSetup);
                    pathState =12;
                }
                break;
            case 8: // begin intake
                intakeSys.intakeStart();
                if(!follower.isBusy()){
                    follower.followPath(paths.IntakeMiddleLoad);
                    follower.setMaxPower(0.4);
                    pathState++;
                }
                break;
            case 9:
                intakeSys.intakeStart();
                if(!follower.isBusy()){
                    intakeSys.intakeStop();
                    shooterSys.startShooting();
                    follower.followPath(paths.MiddleToScore);
                    follower.setMaxPower(1);
                    pathState++;
                }
                break;
            case 10:
                intakeSys.intakeStart();
                shooterSys.startShooting();
                if(!follower.isBusy()){
                    utilityTimer.reset();
                    pathState++;
                }
                break;
            case 11:
                intakeSys.shootStart();
                if(utilityTimer.seconds() >5.0){
                    intakeSys.intakeStop();
                    shooterSys.stopShooting();
                    pathState++;
                }
                break;
            case 12:
                follower.followPath(paths.LEAVE);
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

