package org.firstinspires.ftc.teamcode.Geronimo;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Four High Specimens Auto Route", preselectTeleOp = "Geronimo 1 Manual Control")
// @Disabled
public class FourSpecimensAutoAkshay extends LinearOpMode {
    Geronimo control = new Geronimo(true, false,this);
    MecanumDrive_Geronimo drive;
    Pose2d startPose;

    // Trajectories
    Action DeliverStartingSpecimen;
    Action DriveToSamplesandDeliver1;
    Action DriveToSamplesandDeliver2;
    Action DriveToSamplesandDeliver3;
    Action DrivetoDeck1;
    Action DriveToSubmersible1;
    Action DrivetoDeck2;
    Action DriveToSubmersible2;
    Action DrivetoDeck3;
    Action DriveToSubmersible3;
    Action ParkinDeck;

    // LIMELIGHT ACTION BELOW
    Action LimelightAction;

    VelConstraint speedUpVelocityConstraint;
    AccelConstraint speedUpAccelerationConstraint;
    VelConstraint normalVelocityConstraint;
    AccelConstraint normalAccelerationConstraint;
    VelConstraint slowDownVelocityConstraint;
    AccelConstraint slowDownAccelerationConstraint;
    VelConstraint intakeVelocityConstraint;

    public void runOpMode(){
        startPose = new Pose2d(12,-64, Math.toRadians(270));

        // Define some custom constraints
        speedUpVelocityConstraint = new TranslationalVelConstraint(60.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        normalVelocityConstraint = new TranslationalVelConstraint(50.0);
        normalAccelerationConstraint = new ProfileAccelConstraint(-35.0, 50.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(30);
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);
        intakeVelocityConstraint = new TranslationalVelConstraint(15);

        // Initialize the Robot
        drive = new MecanumDrive_Geronimo(hardwareMap, startPose);
        control.Init(hardwareMap);
        // If your hardware has a "limelight" device configured, you may want:
        // control.InitLimelight(hardwareMap);

        control.AutoStartPosition();
        telemetry.update();
        control.imuOffsetInDegrees = 270;

        while(!isStarted()){
            telemetry.update();
        }
        resetRuntime();
        Geronimo.autoTimeLeft = 0.0;
        control.SetClawPosition(Geronimo.CLAW_MAX_POS);

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************

        DeliverStartingSpecimen = drive.actionBuilder(startPose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0,-39), Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(0,-30), Math.toRadians(270), intakeVelocityConstraint)
                .build();

        DriveToSamplesandDeliver1 = drive.actionBuilder(new Pose2d(0, -30, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(36, -40),Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(36,-16), Math.toRadians(270), slowDownVelocityConstraint)
                .splineToConstantHeading(new Vector2d(46,-16), Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .lineToYConstantHeading(-54)
                .build();

        DriveToSamplesandDeliver2 = drive.actionBuilder(new Pose2d(46,-54, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(46, -17), Math.toRadians(270), slowDownVelocityConstraint)
                .splineToConstantHeading(new Vector2d(56,-17), Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .lineToYConstantHeading(-54)
                .build();

        DrivetoDeck1 = drive.actionBuilder(new Pose2d(56,-54,Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(36,-54,Math.toRadians(270)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(36, -63),Math.toRadians(270), intakeVelocityConstraint)
                .build();

        DriveToSubmersible1 = drive.actionBuilder(new Pose2d(36,-63, Math.toRadians(270)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(2,-39),Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(2,-30), Math.toRadians(270), intakeVelocityConstraint)
                .build();

        DrivetoDeck2 = drive.actionBuilder(new Pose2d(2,-30,Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(36,-54,Math.toRadians(270)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(36,-63), Math.toRadians(270), intakeVelocityConstraint)
                .build();

        DriveToSubmersible2 = drive.actionBuilder(new Pose2d(36,-63,Math.toRadians(270)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(3,-39),Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(3,-30), Math.toRadians(270), intakeVelocityConstraint)
                .build();

        DrivetoDeck3 = drive.actionBuilder(new Pose2d(3,-30,Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(36,-54,Math.toRadians(270)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(36,-63), Math.toRadians(270), intakeVelocityConstraint)
                .build();

        DriveToSubmersible3 = drive.actionBuilder(new Pose2d(36,-63,Math.toRadians(90)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(4,-39),Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(4,-30), Math.toRadians(270), intakeVelocityConstraint)
                .build();

        ParkinDeck = drive.actionBuilder(new Pose2d(4,-30,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(4,-35), Math.toRadians(270), intakeVelocityConstraint)
                .splineToLinearHeading(new Pose2d(36,-58,Math.toRadians(90)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .build();

        // LIMELIGHT ACTION BELOW:
        LimelightAction = limelightAction();

        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        Actions.runBlocking(
                new SequentialAction(
                        // 1) Deliver pre-loaded specimen
                        new ParallelAction(DeliverStartingSpecimen,
                                new SequentialAction(
                                        new SleepAction(0.3),
                                        deliverSpecimenHigh()
                                )
                        ),
                        finishdeliverSpecimenHigh(),
                        new SleepAction(.5),
                        releaseSpecimen(),
                        new SleepAction(.3),

                        // 2) Gather floor samples
                        new ParallelAction(DriveToSamplesandDeliver1,
                                new SequentialAction(
                                        new SleepAction(.3),
                                        slidestozero(),
                                        rotatorarmstozero(),
                                        grabSpecimenfromwall()
                                )
                        ),
                        DriveToSamplesandDeliver2,

                        // 3) Drive to the wall, prepare to grab
                        new ParallelAction(DrivetoDeck1, grabSpecimenfromwall()),
                        grabSpecimen(),
                        new SleepAction(.3),
                        liftSpecimenoffWall(),
                        new SleepAction(.5),

                        // 4) Deliver to high bar
                        new ParallelAction(DriveToSubmersible1, deliverSpecimenHigh()),
                        finishdeliverSpecimenHigh(),
                        new SleepAction(.5),
                        releaseSpecimen(),
                        new SleepAction(.3),

                        // 5) 2nd specimen
                        new ParallelAction(DrivetoDeck2,
                                new SequentialAction(
                                        slidestozero(),
                                        rotatorarmstozero(),
                                        stowPosition()
                                )
                        ),
                        grabSpecimenfromwall(),
                        grabSpecimen(),
                        new SleepAction(.3),
                        liftSpecimenoffWall(),
                        new SleepAction(.5),
                        new ParallelAction(DriveToSubmersible2, deliverSpecimenHigh()),
                        finishdeliverSpecimenHigh(),
                        new SleepAction(.5),
                        releaseSpecimen(),
                        new SleepAction(.3),

                        // 6) 3rd specimen
                        new ParallelAction(DrivetoDeck3,
                                new SequentialAction(
                                        slidestozero(),
                                        rotatorarmstozero(),
                                        stowPosition()
                                )
                        ),
                        grabSpecimenfromwall(),
                        grabSpecimen(),
                        new SleepAction(.3),
                        liftSpecimenoffWall(),
                        new SleepAction(.5),
                        new ParallelAction(DriveToSubmersible3, deliverSpecimenHigh()),
                        finishdeliverSpecimenHigh(),
                        new SleepAction(.5),
                        releaseSpecimen(),
                        new SleepAction(.3),

                        // 7) Here is where you can optionally run the LimelightAction
                        LimelightAction,

                        // 8) Finally, park in the deck
                        new ParallelAction(
                                ParkinDeck,
                                new SequentialAction(
                                        slidestozero(),
                                        rotatorarmstozero(),
                                        stowPosition()
                                )
                        )
                        // new SleepAction(5)
                )
        );
        drive.updatePoseEstimate();

        Geronimo.autoTimeLeft = 30 - getRuntime();
        telemetry.addData("Time left", Geronimo.autoTimeLeft);
        telemetry.update();
    }

    // ------------------------------------------------------------------
    // LIMELIGHT ACTION IMPLEMENTATION
    // ------------------------------------------------------------------
    /**
     *  Return an Action that, upon being run, uses the Limelight-based strafe offset
     *  to move sideways (via a short Road Runner trajectory).
     */
    public Action limelightAction() {
        return new LimelightAction();
    }

    public class LimelightAction implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // 1) Query how many inches we need to strafe (from Limelight's vantage)
                double offsetInches = control.GetStrafeOffsetInInches("block");
                telemetry.addLine("offset" + offsetInches);

                // 2) Build a short strafe trajectory from the robot’s current pose
                Pose2d currentPose = drive.pose;
                Action strafeAction = drive.actionBuilder(currentPose)
                        // We'll strafe in the Y direction by offsetInches
                        .strafeToConstantHeading(new Vector2d(
                               0,
                               0 + offsetInches
                        ))
                        .build();

                // 3) Run that action to completion
                Actions.runBlocking(strafeAction);

                // 4) Done with the limelight strafe
                initialized = true;
            }

            // Returning false signals we are finished (never gets re-called)
            return false;
        }
    }
    // ------------------------------------------------------------------

    public Action grabSpecimenfromwall (){return new GrabSpecimenFromWall();}
    public class GrabSpecimenFromWall implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SpecimenPickupFromWallServoPosition();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;
        }
    }

    public Action deliverLowSpecimen (){return new DeliverLowSpecimen();}
    public class DeliverLowSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SpecimenDeliverLow();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;
        }
    }

    public Action releaseSpecimen (){return new ReleaseSpecimen();}
    public class ReleaseSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SetClawPosition(Geronimo.CLAW_MIN_POS);
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;
        }
    }

    public Action stowPosition (){return new StowPosition();}
    public class StowPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.RemoveFromWallServoPosition();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;
        }
    }

    public Action slidestozero (){return new SlidesToZero();}
    public class SlidesToZero implements Action {
        private boolean initialized = false;
        double timeout = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SetSlideToPosition(0);
                initialized = true;
                timeout = control.opMode.getRuntime() + 0.5;
            }
            boolean returnValue = true;
            if (control.opMode.getRuntime()>=timeout || control.GetSlidesLimitSwitchPressed()) {
                returnValue = false;
            }
            packet.put("lock purple pixel", 0);
            return returnValue;
        }
    }

    public Action rotatorarmstozero (){return new RotatorArmsToZero();}
    public class RotatorArmsToZero implements Action {
        private boolean initialized = false;
        double timeout = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SetSlideRotatorArmToPosition(0);
                initialized = true;
                timeout = control.opMode.getRuntime() + 0.5;
            }
            boolean returnValue = true;
            if (control.opMode.getRuntime()>=timeout || control.GetSlideRotatorArmLimitSwitchPressed()) {
                returnValue = false;
            }
            packet.put("lock purple pixel", 0);
            return returnValue;
        }
    }

    public Action grabSpecimen (){return new GrabSpecimen();}
    public class GrabSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SetClawPosition(Geronimo.CLAW_MAX_POS);
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;
        }
    }

    public Action liftSpecimenoffWall (){return new LiftSpecimenOffWall();}
    public class LiftSpecimenOffWall implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.RemoveFromWallServoPosition();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;
        }
    }

    public Action deliverSpecimenHigh (){return new DeliverSpecimenHigh();}
    public class DeliverSpecimenHigh implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SpecimenDeliverHighChamberAlternate();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;
        }
    }

    public Action finishdeliverSpecimenHigh (){return new FinishDeliverSpecimenHigh();}
    public class FinishDeliverSpecimenHigh implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SpecimenDeliverHighChamberFinishingMove();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;
        }
    }
}
