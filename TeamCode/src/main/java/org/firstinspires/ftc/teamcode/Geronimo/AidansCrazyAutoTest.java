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
//import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// TODO -- quit naming programs after yourself and make it have an intuitive name
@Autonomous(name="Aidan'sCrazyAutoTest", preselectTeleOp =
        "Geronimo_Manual_Control")
// @Disabled
public class AidansCrazyAutoTest extends LinearOpMode {
    Geronimo control = new Geronimo(true, false,this);
    MecanumDrive_Geronimo drive;
    Pose2d startPose;

    // Trajectories
    Action DeliverStartingSpecimen;
    Action DriveToSamplesandDeliver1;
    Action DriveToSamplesandDeliver2;
    Action DriveToSamplesandDeliver3;
    Action IntakeDrive1;
    Action DriveToSubmersible1;
    Action DriveToSubmersible2;
    Action DriveToSubmersible3;
    Action ParkinDeck;

    VelConstraint speedUpVelocityConstraint;
    AccelConstraint speedUpAccelerationConstraint;
    VelConstraint slowDownVelocityConstraint;
    AccelConstraint slowDownAccelerationConstraint;
    VelConstraint intakeVelocityConstraint;

    public void runOpMode(){
        startPose = new Pose2d(12,-64, Math.toRadians(270));

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(60.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(30);
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);
        intakeVelocityConstraint = new TranslationalVelConstraint(15);

        /* Initialize the Robot */
        drive = new MecanumDrive_Geronimo(hardwareMap, startPose);
        control.Init(hardwareMap);
        // TODO -- need to adjust this starting position to keep the specimen out of the wall
        control.AutoStartPosition();
        telemetry.update();
        control.imuOffsetInDegrees = 270; // Math.toDegrees(startPose.heading.toDouble());

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
                //.splineToConstantHeading(new Vector2d(4,-39), Math.toRadians(-45))
                //.strafeToLinearHeading(new Vector2d(0,-39), Math.toRadians(270))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0,-39), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0,-30), Math.toRadians(270))
                .build();

        DriveToSamplesandDeliver1 = drive.actionBuilder(new Pose2d(0, -30, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(23,-48), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(36,-48), Math.toRadians(270))
                //.splineToConstantHeading(new Vector2d(36,-12), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(36,-12), Math.toRadians(270))
                //.strafeToLinearHeading(new Vector2d(44,-54),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(44,-17), Math.toRadians(270))
                .lineToYConstantHeading(-54)
                .build();

        DriveToSamplesandDeliver2 = drive.actionBuilder(new Pose2d(44,-54, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(44, -17), Math.toRadians(270), slowDownVelocityConstraint)
                .splineToConstantHeading(new Vector2d(54,-17), Math.toRadians(270))
                .lineToYConstantHeading(-54)
                .build();

        DriveToSamplesandDeliver3 = drive.actionBuilder(new Pose2d(54,-54,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(54, -12), Math.toRadians(270), slowDownVelocityConstraint)
                //.strafeToLinearHeading(new Vector2d(60.75,-12), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(60.75,-17), Math.toRadians(270))
                //.strafeToLinearHeading(new Vector2d(60.75,-54), Math.toRadians(270))
                .lineToYConstantHeading(-54)
                //.strafeToLinearHeading(new Vector2d(48,-54),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(44,-46),Math.toRadians(270))
                .build();

        IntakeDrive1 = drive.actionBuilder(new Pose2d(44,-46,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(44, -63),Math.toRadians(270), intakeVelocityConstraint)
                .build();

        DriveToSubmersible1 = drive.actionBuilder(new Pose2d(44,-63, Math.toRadians(270)))
                //.strafeToLinearHeading(new Vector2d(48, -54), Math.toRadians(270))
                //.strafeToLinearHeading(new Vector2d(2, -54), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(2, -39), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(2,-30), Math.toRadians(270))
                .build();
        /*
        DriveToSubmersible2 = drive.actionBuilder(new Pose2d(7,-35,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(48,-54), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(7,-35), Math.toRadians(90)) //May change to 270 heading once delivery is clarified
                .build();
        DriveToSubmersible3 = drive.actionBuilder(new Pose2d(7,-35,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(48,-54), Math.toRadians(270))
                 .strafeToLinearHeading(new Vector2d(7,-35), Math.toRadians(90))
                .build();

       */
         ParkinDeck = drive.actionBuilder(new Pose2d(2,-30,Math.toRadians(270)))
                 //Pose 2D 50,-54, 270
                 .splineTo(new Vector2d(49,-45),Math.toRadians(90))
                 .strafeToLinearHeading(new Vector2d(49,-58), Math.toRadians(90))
                // .turnTo(Math.toRadians(90))
                        .build();

        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        Actions.runBlocking(
                new SequentialAction(
                        // deliver pre-loaded specimen
                        new ParallelAction(DeliverStartingSpecimen,
                                deliverSpecimenHigh()),
                        finishdeliverSpecimenHigh(),
                        new SleepAction(.5),
                        releaseSpecimen(),
                        new SleepAction(.3),

                        /* TODO
                        Need to replace grabSpecimenfromWall in this parallel action with an embedded sequential action within the parallel action
                            - New Action -- Set Slides to zero -- don't return "false" in the action until GetSlidesLimitSwitchPressed()
                            - New Action -- Set Rotator Arm to zero -- don't return false in the action until GetSlideRotatorArmLimitSwitchPressed
                            - Change the grabSpecimenfromWall() action to call SpecimenPickupFromWallServoPosition() instead of SpecimenPickupFromWall()
                         */
                        // Gather the 3 floor samples into the observation area
                        new ParallelAction(DriveToSamplesandDeliver1, grabSpecimenfromwall()),
                        DriveToSamplesandDeliver2,
                        DriveToSamplesandDeliver3,

                        // Drive to the wall and prepare to grab a specimen
                        new ParallelAction(IntakeDrive1,
                                grabSpecimenfromwall()),

                        // TODO -- need to test how small we can make these sleep actions, these servos are pretty fast this year
                        // grab the specimen off of the wall
                        grabSpecimen(),
                        new SleepAction(.3),
                        liftSpecimenoffWall(),
                        new SleepAction(.5),

                        // Deliver the specimen to the High bar
                        new ParallelAction(DriveToSubmersible1, deliverSpecimenHigh()),
                        finishdeliverSpecimenHigh(),
                        new SleepAction(.5),
                        releaseSpecimen(),
                        new SleepAction(.3),

                        // TODO -- need to convert stowPosition into an embedded sequential action within the parallel action like was done above
                        // Drive to the parking position
                        new ParallelAction(ParkinDeck,
                                stowPosition())
                        //new SleepAction(5))

        ));
        drive.updatePoseEstimate();

        Geronimo.autoTimeLeft = 30-getRuntime();
        telemetry.addData("Time left", Geronimo.autoTimeLeft);
        telemetry.update();

    }

    public Action grabSpecimenfromwall (){return new GrabSpecimenFromWall();}
    public class GrabSpecimenFromWall implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
               // control.limelightHasTarget();
                control.SpecimenPickupFromWall();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
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
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action releaseSpecimen (){return new ReleaseSpecimen();}
    public class ReleaseSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SetClawPosition((Geronimo.CLAW_MIN_POS));
                control.SpecimenPickupFromWall();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action stowPosition (){return new StowPosition();}
    public class StowPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.RemoveFromWall();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
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
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action liftSpecimenoffWall (){return new LiftSpecimenOffWall();}
    public class LiftSpecimenOffWall implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.RemoveFromWall();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
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
            return false;  // returning true means not done, and will be called again.  False means action is completely done
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
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
}

