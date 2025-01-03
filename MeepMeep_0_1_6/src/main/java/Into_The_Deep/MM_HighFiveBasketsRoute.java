package Into_The_Deep;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MM_HighFiveBasketsRoute {
    static Action DeliverStartingSample;
    static Action DriveToSample1;
    static Action DeliverSample1;
    static Action DriveToSample2;
    static Action DeliverSample2;
    static Action DriveToSample3;
    static Action DeliverSample3;

    static VelConstraint speedUpVelocityConstraint;
    static AccelConstraint speedUpAccelerationConstraint;
    static VelConstraint normalVelocityConstraint;
    static AccelConstraint normalAccelerationConstraint;
    static VelConstraint slowDownVelocityConstraint;
    static AccelConstraint slowDownAccelerationConstraint;
    static VelConstraint intakeVelocityConstraint;
    static VelConstraint humanPlayerVelocityConstraint;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, Math.PI*.6, Math.PI*.8, 15)
                .build();

        Pose2d StartPose = new Pose2d(-12, -64, Math.toRadians(90));

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(60.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        normalVelocityConstraint = new TranslationalVelConstraint(50.0);
        normalAccelerationConstraint = new ProfileAccelConstraint(-35.0, 50.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(30);
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);
        intakeVelocityConstraint = new TranslationalVelConstraint(15);
        humanPlayerVelocityConstraint = new TranslationalVelConstraint(7);

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************

        DeliverStartingSample = myBot.getDrive().actionBuilder(StartPose)
                .splineToConstantHeading(new Vector2d(-36, -36),Math.toRadians(90))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-57,-57), Math.toRadians(45))
                .build();

        DriveToSample1 = myBot.getDrive().actionBuilder(new Pose2d(-57, -57, Math.toRadians(45)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-48,-47), Math.toRadians(90))
                .build();

        DeliverSample1 = myBot.getDrive().actionBuilder(new Pose2d(-48,-47, Math.toRadians(90)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-59,-59), Math.toRadians(45))
                .build();

        DriveToSample2 = myBot.getDrive().actionBuilder(new Pose2d(-59, -59, Math.toRadians(45)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-57,-43), Math.toRadians(90))
                .build();
        DeliverSample2 = myBot.getDrive().actionBuilder(new Pose2d(-57,-43, Math.toRadians(90)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-57,-57), Math.toRadians(45))
                .build();

        DriveToSample3 = myBot.getDrive().actionBuilder(new Pose2d(-57, -57, Math.toRadians(45)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-55,-40), Math.toRadians(135))
                .build();
        DeliverSample3 = myBot.getDrive().actionBuilder(new Pose2d(-55,-40, Math.toRadians(135)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-57,-57), Math.toRadians(45))
                .build();

        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        myBot.runAction(
                new SequentialAction(
                        //Drive to submersible and pick up sample
                        DeliverStartingSample,
                        // Put sample in the basket and safely Lower the slides and arms before moving
                        new SleepAction(1.0),

                        DriveToSample1,
                        // Pick up Sample from floor
                        new SleepAction(1.5),
                        // Deliver Sample to High Backet
                        DeliverSample1,
                        // Put sample in the basket and safely Lower the slides and arms before moving
                        new SleepAction(1.0),

                        DriveToSample2,
                        // Pick up Sample from floor
                        new SleepAction(1.5),
                        // Deliver Sample to High Backet
                        DeliverSample2,
                        // Put sample in the basket and safely Lower the slides and arms before moving
                        new SleepAction(6.0),

                        DriveToSample3,
                        // Pick up Sample from floor
                        new SleepAction(1.5),
                        // Deliver Sample to High Backet
                        DeliverSample3,
                        // Put sample in the basket and safely Lower the slides and arms before moving
                        new SleepAction(6.0)

                        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
