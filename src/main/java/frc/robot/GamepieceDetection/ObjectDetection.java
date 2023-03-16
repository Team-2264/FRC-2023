package frc.robot.GamepieceDetection;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTableInstance;

public class ObjectDetection {
    public float pitch;
    public float yaw;
    //public float distance;
    //public float area;
    public ObjectType type;

    public static Optional<ObjectDetection> getBestDetection() {
        var table = NetworkTableInstance.getDefault().getTable("ObjectVision");

        boolean object_detected = table.getEntry("detected").getBoolean(false);
        var pitch = table.getEntry("pitch").getFloat(Float.NaN);
        var yaw = table.getEntry("yaw").getFloat(Float.NaN);

        if(pitch == Double.NaN || yaw == Double.NaN || !object_detected) {
            return Optional.empty();
        }

        var detection = new ObjectDetection();

        detection.pitch = pitch;
        detection.yaw = yaw;
        detection.type = ObjectType.Cone;

        return Optional.of(detection);
    }
    
    // Code for detecting multiple objects at the same time (currently doesn't work with current python code)
        /*public static ObjectDetection[] getDetections() {
            var table = NetworkTableInstance.getDefault().getTable("ObjectVision");
            float[] empty_float = {};
            boolean[] empty_bool = {};

            float[] pitch_array = table.getEntry("pitch").getFloatArray(empty_float);
            float[] yaw_array = table.getEntry("yaw").getFloatArray(empty_float);
            float[] distance_array = table.getEntry("distance").getFloatArray(empty_float);
            float[] area_array = table.getEntry("area").getFloatArray(empty_float);
            boolean[] is_cone_array = table.getEntry("is_cone").getBooleanArray(empty_bool);

            var len = pitch_array.length;
            ObjectDetection[] detections = new ObjectDetection[len];

            for(int i = 0; i < len; i++) {
                var detection = new ObjectDetection();
                detection.pitch = pitch_array[i];
                detection.yaw = yaw_array[i];
                detection.distance = distance_array[i];
                detection.area = area_array[i];

                if(is_cone_array[i]) {
                    detection.type = ObjectType.Cone;
                } else {
                    detection.type = ObjectType.Cube;
                }
            }

            return detections;
        }

        public static Optional<ObjectDetection> getBestDetection() {
            var table = NetworkTableInstance.getDefault().getTable("ObjectVision");
            float[] empty_float = {};
            boolean[] empty_bool = {};

            float[] pitch_array = table.getEntry("pitch").getFloatArray(empty_float);
            float[] yaw_array = table.getEntry("yaw").getFloatArray(empty_float);
            float[] distance_array = table.getEntry("distance").getFloatArray(empty_float);
            float[] area_array = table.getEntry("area").getFloatArray(empty_float);
            boolean[] is_cone_array = table.getEntry("is_cone").getBooleanArray(empty_bool);

            var len = pitch_array.length;

            var largest_area = 0.0;
            Integer largest_area_index = null;


            for(int i = 0; i < len; i++) {
                if(area_array[i] > largest_area) {
                    largest_area = area_array[i];
                    largest_area_index = i;
                }
            }

            if(largest_area_index == null) {
                return Optional.empty();
            }

            var detection = new ObjectDetection();
            detection.pitch = pitch_array[largest_area_index];
            detection.yaw = yaw_array[largest_area_index];
            detection.distance = distance_array[largest_area_index];
            detection.area = area_array[largest_area_index];

            if(is_cone_array[largest_area_index]) {
                detection.type = ObjectType.Cone;
            } else {
                detection.type = ObjectType.Cube;
            }

            return Optional.of(detection);
        }*/

}