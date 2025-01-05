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


@Autonomous(name="High Five Baskets Auto Route", preselectTeleOp =
        "Geronimo 1 Manual Control")
// @Disabled
public class HighFiveBasketsAutoRoute extends LinearOpMode {
    Geronimo control = new Geronimo(true, false,this);
    MecanumDrive_Geronimo drive;
    Pose2d startPose;

    // Trajectories
    Action DeliverStartingSample;
    Action DriveToSample1;
    Action DeliverSample1;
    Action DriveToSample2;
    Action DeliverSample2;
    Action AdjustForDrivers;

    VelConstraint speedUpVelocityConstraint;
    AccelConstraint speedUpAccelerationConstraint;
    VelConstraint normalVelocityConstraint;
    AccelConstraint normalAccelerationConstraint;
    VelConstraint slowDownVelocityConstraint;
    AccelConstraint slowDownAccelerationConstraint;
    VelConstraint intakeVelocityConstraint;
    VelConstraint humanPlayerVelocityConstraint;

    public void runOpMode(){
        startPose = new Pose2d(- 12,-64, Math.toRadians(90));

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(60.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        normalVelocityConstraint = new TranslationalVelConstraint(50.0);
        normalAccelerationConstraint = new ProfileAccelConstraint(-35.0, 50.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(30);
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);
        intakeVelocityConstraint = new TranslationalVelConstraint(15);
        humanPlayerVelocityConstraint = new TranslationalVelConstraint(7);

        /* Initialize the Robot */
        drive = new MecanumDrive_Geronimo(hardwareMap, startPose);
        control.Init(hardwareMap);
        //  -- need to adjust this starting position to keep the specimen out of the wall Check
        control.Stow();
        telemetry.update();
        control.imuOffsetInDegrees = 90; // Math.toDegrees(startPose.heading.toDouble());

        // Wait for start to be pressed
        while(!isStarted()){
            telemetry.update();
        }

        // Started
        resetRuntime();
        Geronimo.autoTimeLeft = 0.0;
        control.SetUrchinServoPosition(1);
        sleep(100);

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************

        DeliverStartingSample = drive.actionBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-36, -36),Math.toRadians(90))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-59,-59 ), Math.toRadians(45))
                .build();
        DriveToSample1 = drive.actionBuilder(new Pose2d(-59,-59,Math.toRadians(45)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-48,-47), Math.toRadians(90))
                .build();
        DeliverSample1 = drive.actionBuilder(new Pose2d(-48,-47,Math.toRadians(90)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-59,-59), Math.toRadians(45))
                .build();
        DriveToSample2 = drive.actionBuilder(new Pose2d(-59,-59,Math.toRadians(45)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-57,-47), Math.toRadians(45))
                .build();
        DeliverSample2 = drive.actionBuilder(new Pose2d(-57,-47,Math.toRadians(90)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-59,-59), Math.toRadians(45))
                .build();

        AdjustForDrivers = drive.actionBuilder(new Pose2d(-59,-59,Math.toRadians(45)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-42,-24), Math.toRadians(90))
                .build();
        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        Actions.runBlocking(
                new SequentialAction(
                        // Drive to basket and deliver preloaded sample
                        new ParallelAction(DeliverStartingSample, basketHigh()),
                        // Raise slides to high basket height
                        finishBasketHigh_SlidesPosition(),
                        new SleepAction(3.0),
                        // Rotate urchin to align above basket
                        finishBasketHigh_UrchinDeliverPosition(),
                        new SleepAction(0.4),
                        // Release the sample from the urchin
                        openUrchin(),
                        new SleepAction(0.4),
                        // Rotate urchin back away from the basket
                        finishBasketHigh_UrchinSafeToLowerPosition(),
                        new SleepAction(0.4),
                        // Rotate arms a little away from basket and lower slides to zero
                        finishBasketHigh_ArmSafeToLowerPosition(),
                        slidesToZero(), new SleepAction(1),
                        stowPosition(),
                        rotatorArmsToZero(),

                        // Drive to Sample 1
                        new ParallelAction(DriveToSample1, sampleUrchinFloorPickup_SlidePosition(), openUrchin()),
                        sampleUrchinFloorPickup_UrchinReadyPosition(),
                        new SleepAction(0.4),
                        sampleUrchinFloorPickupFinishingMove_UrchinGrabPosition(),
                        new SleepAction(0.1),
                        closeUrchin(),
                        new SleepAction(0.4),
                        sampleUrchinFloorPickup_UrchinReadyPosition(),
                        new SleepAction(0.1),
                        stowPosition(),
                        new SleepAction(0.2),
                        // *** Deliver Sample 1 ***
                        new ParallelAction(DeliverSample1, basketHigh()),
                        finishBasketHigh_SlidesPosition(),
                        new SleepAction(3.0),
                        // Rotate urchin to align above basket
                        finishBasketHigh_UrchinDeliverPosition(),
                        new SleepAction(0.4),
                        // Release the sample from the urchin
                        openUrchin(),
                        new SleepAction(0.4),
                        // Rotate urchin back away from the basket
                        finishBasketHigh_UrchinSafeToLowerPosition(),
                        new SleepAction(0.4),
                        // Rotate arms a little away from basket and lower slides to zero
                        finishBasketHigh_ArmSafeToLowerPosition(),
                        slidesToZero(), new SleepAction(1),
                        stowPosition(),
                        rotatorArmsToZero(),

                        // Drive to Sample 2
                        new ParallelAction(DriveToSample2, sampleUrchinFloorPickup_SlidePosition(), openUrchin()),
                        sampleUrchinFloorPickup_UrchinReadyPosition(),
                        new SleepAction(0.4),
                        sampleUrchinFloorPickupFinishingMove_UrchinGrabPosition(),
                        new SleepAction(0.1),
                        closeUrchin(),
                        new SleepAction(0.4),
                        sampleUrchinFloorPickup_UrchinReadyPosition(),
                        new SleepAction(0.1),
                        stowPosition(),
                        new SleepAction(0.2),
                        // *** Deliver Sample 2 ***
                        new ParallelAction(DeliverSample2, basketHigh()),
                        finishBasketHigh_SlidesPosition(),
                        new SleepAction(3.0),
                        // Rotate urchin to align above basket
                        finishBasketHigh_UrchinDeliverPosition(),
                        new SleepAction(0.4),
                        // Release the sample from the urchin
                        openUrchin(),
                        new SleepAction(0.4),
                        // Rotate urchin back away from the basket
                        finishBasketHigh_UrchinSafeToLowerPosition(),
                        new SleepAction(0.4),
                        // Rotate arms a little away from basket and lower slides to zero
                        finishBasketHigh_ArmSafeToLowerPosition(),
                        slidesToZero(), new SleepAction(1),
                        stowPosition(),
                        rotatorArmsToZero(),

                        // Park for Drivers
                        new ParallelAction(AdjustForDrivers, sampleUrchinFloorPickup_SlidePosition(), openUrchin())
                        // position the urchin to be ready to intake
                         /*,
                        new SleepAction(0.2),
                        // Lower the urchin to be closer to the floor
                        sampleUrchinFloorPickupFinishingMove_UrchinGrabPosition(),
                        new SleepAction(0.1),
                        // Close the urchin to grab the specimen
                        closeUrchin(),
                        new SleepAction(0.3),
                        // Raise the Urchin back up to ready position to assess if succeeded
                        sampleUrchinFloorPickup_UrchinReadyPosition()
                        */
                ));

        drive.updatePoseEstimate();

        Geronimo.autoTimeLeft = 30-getRuntime();
        telemetry.addData("Time left", Geronimo.autoTimeLeft);
        telemetry.update();

    }

    public Action basketHigh (){return new BasketHigh();}
    public class BasketHigh implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
               // control.limelightHasTarget();
                control.BasketHigh();
                initialized = true;
            }
            packet.put("basketHigh", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }

    public Action finishBasketHigh_SlidesPosition (){return new FinishBasketHigh_SlidesPosition();}
    public class FinishBasketHigh_SlidesPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.BasketHighFinishingMove_SlidesPosition();
                initialized = true;
            }
            packet.put("finishBasketHigh_SlidesPosition", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action finishBasketHigh_UrchinDeliverPosition (){return new FinishBasketHigh_UrchinDeliverPosition();}
    public class FinishBasketHigh_UrchinDeliverPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.BasketHighFinishingMove_UrchinDeliverPosition();
                initialized = true;
            }
            packet.put("finishBasketHigh_UrchinDeliverPosition", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action finishBasketHigh_UrchinSafeToLowerPosition (){return new FinishBasketHigh_UrchinSafeToLowerPosition();}
    public class FinishBasketHigh_UrchinSafeToLowerPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.BasketHighFinishingMove_UrchinSafeToLowerPosition();
                initialized = true;
            }
            packet.put("finishBasketHigh_UrchinSafeToLowerPosition", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action finishBasketHigh_ArmSafeToLowerPosition (){return new FinishBasketHigh_ArmSafeToLowerPosition();}
    public class FinishBasketHigh_ArmSafeToLowerPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.BasketHighFinishingMove_ArmSafeToLowerPosition();
                initialized = true;
            }
            packet.put("finishBasketHigh_ArmSafeToLowerPosition", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
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
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action openUrchin (){return new OpenUrchin();}
    public class OpenUrchin implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SetUrchinServoPosition(0);
                //control.SpecimenPickupFromWallServoPosition();
                initialized = true;
            }
            packet.put("openUrchin", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action closeUrchin (){return new CloseUrchin();}
    public class CloseUrchin implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SetUrchinServoPosition(1);
                //control.SpecimenPickupFromWallServoPosition();
                initialized = true;
            }
            packet.put("closeUrchin", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action slidesToZero(){return new SlidesToZero();}
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
            if (control.opMode.getRuntime()>=timeout || control.GetSlidesLimitSwitchPressed())
            {
                returnValue = false;
            }
            packet.put("slidesToZero", 0);
            return returnValue ;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action rotatorArmsToZero(){return new RotatorArmsToZero();}
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
            if (control.opMode.getRuntime()>=timeout || control.GetSlideRotatorArmLimitSwitchPressed())
            {
                returnValue = false;
            }
            packet.put("rotatorArmsToZero", 0);
            return returnValue ;

              // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action sampleUrchinFloorPickup_SlidePosition (){return new SampleUrchinFloorPickup_SlidePosition();}
    public class SampleUrchinFloorPickup_SlidePosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SampleUrchinFloorPickup_SlidePosition();
                initialized = true;
            }
            packet.put("SampleUrchinFloorPickup_SlidePosition", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action sampleUrchinFloorPickup_UrchinReadyPosition (){return new SampleUrchinFloorPickup_UrchinReadyPosition();}
    public class SampleUrchinFloorPickup_UrchinReadyPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SampleUrchinFloorPickup_UrchinReadyPosition();
                initialized = true;
            }
            packet.put("SampleUrchinFloorPickup_UrchinReadyPosition", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action sampleUrchinFloorPickupFinishingMove_UrchinGrabPosition (){return new SampleUrchinFloorPickupFinishingMove_UrchinGrabPosition();}
    public class SampleUrchinFloorPickupFinishingMove_UrchinGrabPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SampleUrchinFloorPickup_UrchinReadyPosition();
                initialized = true;
            }
            packet.put("SampleUrchinFloorPickupFinishingMove_UrchinGrabPosition", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }

}

