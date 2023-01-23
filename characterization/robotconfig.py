{

    "rightControllerTypes": ["TalonFX", "TalonFX"],
    "leftControllerTypes": ["TalonFX", "TalonFX"],
    # Note: The first id in the list of ports should be the one with an encoder
    # Ports for the left-side motors
    "leftMotorPorts": [50, 56],
    # Ports for the right-side motors
    "rightMotorPorts": [52, 54],
    # Inversions for the left-side motors
    "leftMotorsInverted": [False, False],
    # Inversions for the right side motors
    "rightMotorsInverted": [False, False],

    # Wheel diameter (in units of your choice - will dictate units of analysis)
    "wheelDiameter": 4,
    # If your robot has only one encoder, set all right encoder fields to `None`
    # Encoder edges-per-revolution (*NOT* cycles per revolution!)
    # This value should be the edges per revolution *of the wheels*, and so
    # should take into account gearing between the encoder and the wheels
    "encoderEPR": 13824, # 6.75:1 gearing so 6.75*2048

    # Whether the left encoder is inverted
    "leftEncoderInverted": False,
    # Whether the right encoder is inverted:
    "rightEncoderInverted": False,

    "gyroType": "Pigeon2",
    "gyroPort": "15",
}


