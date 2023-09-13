package org.firstinspires.ftc.teamcode.subsystem.tensorflow;

public class SubSys_Tensorflow_Constants
{
    public class ProcessorOptions
    {
        public static final int MAX_RECOGNITIONS = 16; // Max number of recognised objects on screen
        public static final boolean USE_OBJECT_TRACKER = true; // Use TFlite object tracker
        public static final float MAX_BOX_OVERLAP = 0.2f; // How much the boxes can overlap
        public static final float MIN_BOX_SIZE = 16; // Minimum size of detection recognized by the system

        public class Advanced
        {
            public static final boolean TENSORFLOW_2 = false; // Is the detector tf2
            public static final boolean IS_QUANTIZED = false; // Is the model quantized
        }
    }
    // Put these in main class BECAUSE JAVA 8 IS STUPID WTF DYM I CAN'T PUT A STATIC STRING LIST IN A SUBCLASS
    public static final String ASSET_NAME = "Test"; // Name of the asset bundled into the build
    public static final String[] MODEL_LABELS = new String[]{"A", "B", "C"}; // Labels in a String list
}
