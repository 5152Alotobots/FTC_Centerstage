package org.firstinspires.ftc.teamcode.subsystem.visionportal.apriltag.pose;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class AprilTagFieldLayout
{

    //
    // COORDINATE SYSTEM IN CENTIMETERS!
    // X = Left/Right from center of driver station
    // Y = Forward/Backward from driver station
    // Z = Up/Down from driver station
    //
    //     +-------------+      Z
    //     |             |      |
    //     |     TAG     |      |
    //     |      X      |      |
    //     |             |      |
    //     +_____________+      |
    //                          |
    //   X----------------------\
    //                           \
    //                            \
    //                             \
    //                              \
    //                               Y
    //
    //


    //                                                  X    Y   Z
    //                      BLUE DRIVER STATION CENTER (0,-182.88,0)
    //                  |                      *                        |
    //                  |                                               |
    //                  |                                               |
    //                  |                                               | R
    //                L |                                               | I
    //                E |                                               | G
    //                F |                                               | H
    //        X   Y Z T |                                               | T     X   Y Z
    //   (-182.88,0,0)  | *                    *   X Y Z              * |   (182.88,0,0)
    //                W |            FIELD CENTER (0,0,0)               | W
    //                A |                                               | A
    //                L |                                               | L
    //                L |                                               | L
    //                  |                                               |
    //                  |                                               |
    //                  |                                               |
    //                  |                      *        X    Y   Z      |
    //                       RED DRIVER STATION CENTER (0,182.88,0)

    public static AprilTagLibrary getCenterStageCentimetersLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(153.04f,105.18f,10.16f), DistanceUnit.CM,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(153.04f,89.94f,10.16f), DistanceUnit.CM,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(153.04f,74.70f,10.16f), DistanceUnit.CM,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(153.04f,-74.70f,10.16f), DistanceUnit.CM,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(153.04f,-89.94f,10.16f), DistanceUnit.CM,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(153.04f,-41.41f,10.16f), DistanceUnit.CM,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-178.44f,-103.19f,13.97f), DistanceUnit.CM,
                        new Quaternion(0.7071f,0,0,-7.071f,0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-178.44f,-89.28f,10.16f), DistanceUnit.CM,
                        new Quaternion(0.7071f,0,0,-7.071f,0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-178.44f,89.28f,10.16f), DistanceUnit.CM,
                        new Quaternion(0.7071f,0,0,-7.071f,0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-178.44f,103.19f,13.97f), DistanceUnit.CM,
                        new Quaternion(0.7071f,0,0,-7.071f,0))
                .build();
    }
}
