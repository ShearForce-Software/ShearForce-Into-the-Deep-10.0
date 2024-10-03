package Into_The_Deep;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AkshaySFirst_Test {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

       // static Pose2d startPose;
    // Testing from home


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        Pose2d startPose = new Pose2d(-36, 0, Math.toRadians(0));


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, 60, Math.toRadians(0)))
                .lineToX(48)
                //.turn(Math.toRadians(90))
                //.lineToY(48)
                //.turn(Math.toRadians(90))
                //.lineToX(0)
                //.turn(Math.toRadians(90))
                //.lineToY(0)
                //.turn(Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    //Define action 1(moving along X and turning 90 degrees)
    public static void runAction1(RoadRunnerBotEntity myBot){
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(48)
                .turn(Math.toRadians(90))
                .build());
    }


}
