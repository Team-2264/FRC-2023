package frc.lib;

import java.util.HashMap;

import frc.robot.enums.AutoPosition;

public class AutonomousPositions {

    public static final HashMap<AutoPosition, String> POSITION_MAP = new HashMap<AutoPosition, String>();

    public AutonomousPositions() {
        // Position Map
        for (AutoPosition position : AutoPosition.values()) {
            POSITION_MAP.put(position, position.toString());
        }
    }

    public HashMap<AutoPosition, String> getPositionMap() {
        return POSITION_MAP;
    }
}
