package Into_The_Deep;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.acmerobotics.roadrunner.SleepAction;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Action;

public class ShreyAutoTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

       // static Pose2d startPose;
    // Testing from home


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();



        Pose2d StartPose = new Pose2d(30, -60, 90);
        Vector2d VectorTwo = new Vector2d(5, -30);
        //myBot.runAction(myBot.getDrive().actionBuilder(StartPose)
                           //  .splineToLinearHeading(new Pose2d(36,48,36.6), Math.toRadians(270))
        Action DriveToSubmersible1 = myBot.getDrive().actionBuilder(StartPose)
                                .strafeToLinearHeading(VectorTwo, Math.toRadians(90))
                                .build();

        Action DropOff1 = myBot.getDrive().actionBuilder(new Pose2d(5,-30,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(60, -58), Math.toRadians(270))
                .build();

        Action DriveToSubmersible2 = myBot.getDrive().actionBuilder(new Pose2d(60,-58,Math.toRadians(270)))
                .strafeToLinearHeading(VectorTwo, Math.toRadians(90))
                .build();

        Action DropOff2 = myBot.getDrive().actionBuilder(new Pose2d(5,-30,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(60, -58), Math.toRadians(270))
                .build();

        Action PickSpeicmen = myBot.getDrive().actionBuilder(new Pose2d(60,-58,Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(40, -58,Math.toRadians(90)),Math.toRadians(90))
                .build();

        Action DriveToSubmersible3 = myBot.getDrive().actionBuilder(new Pose2d(40,-58,Math.toRadians(90)))
                .strafeToLinearHeading(VectorTwo, Math.toRadians(90))
                .build();

        Action SpecimenDrop = myBot.getDrive().actionBuilder(new Pose2d(40,-58,Math.toRadians(90)))
               .strafeToLinearHeading(VectorTwo, Math.toRadians(90))
                .build();

        myBot.runAction(new SequentialAction(
                //Drive to submersible and pick up sample
                DriveToSubmersible1,
                new SleepAction(1),
                //Drop submersible
                DropOff1,
                new SleepAction(1),
                //drive to submersible and pickup sample
                DriveToSubmersible2,
                new SleepAction(1),
                //drop sample and pick up specimen
                DropOff2,
                new SleepAction(1),
                PickSpeicmen,
                new SleepAction(1),
                //drop specimen and pick up sample
                SpecimenDrop//,
                //new SleepAction(1)
                ));



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
