from enum import Enum

class CMD(Enum):
    PING = 0
    SEND_TWO_INTS = 1
    SEND_THREE_FLOATS = 2
    ECHO = 3
    DANCE = 4
    SET_VEL = 5
    GET_TIME_MILLIS = 6
    GET_TEMP_5s = 7
    GET_TEMP_5s_RAPID = 8
    TEST_TOF = 9
    TIMED_TOF = 10
    TIMED_FFT = 11
    TIMED_PID_DATA = 12
    TOF_PID = 13
    UPDATE_KP = 14
    STEP = 15
    KF_PID = 16
    STUNT = 17,
    ANGSPEED = 18,
    LOCALIZATION = 19,
    L12 = 20
    
    