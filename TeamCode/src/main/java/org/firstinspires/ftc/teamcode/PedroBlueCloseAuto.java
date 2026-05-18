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



@Autonomous(name = "PedroBlueCloseAuto", group = "Autonomous")
@Configurable // Panels
public class PedroBlueCloseAuto extends OpMode {

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
        follower.setStartingPose(new Pose(18.4, 120.9, Math.toRadians(-37)));


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
                            new BezierLine(new Pose(19.412, 119.937), new Pose(34.397, 108.828))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(-45))
                    .build();

            IntakeLeftSetup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(31.397, 111.828),
                                    new Pose(47.483, 110.859),
                                    new Pose(47.289, 84.307)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(180))
                    .build();

            IntakeLeftLoad = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.289, 84.307), new Pose(15.699, 83.532))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            LeftToScore = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.699, 83.532), new Pose(61.499, 82.532))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-39))
                    .build();

            IntakeMiddleSetup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(61.499, 82.532),
                                    new Pose(55.042, 66.283),
                                    new Pose(47.289, 60.662)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-39), Math.toRadians(180))
                    .build();

            IntakeMiddleLoad = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.289, 60.662), new Pose(14.536, 58.336))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            MiddleToScore = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(14.536, 58.336),
                                    new Pose(52.522, 59.499),
                                    new Pose(61.499, 82.532)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-42))
                    .build();

            LEAVE = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(61.499, 82.532),
                                    new Pose(57.755, 44.188),
                                    new Pose(46.708, 35.273)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-42), Math.toRadians(180))
                    .build();

            LEAVEFR = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(46.708, 35.273), new Pose(55.429, 35.273))
                    )
                    .setTangentHeadingInterpolation()
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
                    pathState++;
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
                break;

        }
        shooterSys.updateHood();

        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}

