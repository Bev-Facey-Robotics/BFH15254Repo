package FtcRobotController.MeepMeepTesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingFr {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-16,-61, 90))
                .setTangent(0)
                //Line up with first yellow

                .splineToConstantHeading(new Vector2d(-34,-25.6), 0)
                //First Yellow
                .turnTo(Math.PI)
                .lineToXLinearHeading(-47, Math.PI)
                //Intake Program

                //Basket Drive (FIX)
                .lineToXConstantHeading(-43)
                .splineToLinearHeading(new Pose2d(-45,-45,8*Math.PI/6),0)
                .strafeToConstantHeading(new Vector2d(-60,-60))
                //Outake Program

                //Second Yellow
                .strafeToConstantHeading(new Vector2d(-50,-45))
                .splineTo(new Vector2d(-45,-25.6), 180)
                .turnTo(Math.PI)
                .lineToXLinearHeading(-58.3, Math.PI)

                //Intake Program

                //Basket Drive
                .lineToXConstantHeading(-50)
                .splineToLinearHeading(new Pose2d(-45,-45,8*Math.PI/6),0)
                .strafeToConstantHeading(new Vector2d(-60,-60))
                //Outake Program

                //Third Yellow
                .strafeToConstantHeading(new Vector2d(-45,-45))
                .splineTo(new Vector2d(-55,-25.6), 0)
                .turnTo(Math.PI)
                .lineToXLinearHeading(-67.3, Math.PI)

                //Intake Program

                //Basket Drive
                .lineToXConstantHeading(-65)
                .splineToLinearHeading(new Pose2d(-45,-45,8*Math.PI/6),0)
                .strafeToConstantHeading(new Vector2d(-60,-60))
                //Outake Program

                //Parking Run









                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


