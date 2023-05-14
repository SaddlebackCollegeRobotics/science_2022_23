from enum import IntEnum

class PS2000(IntEnum):
    '''
    PicoScope 2000 Series (A API) enumerations
    '''
    CHANNEL_A = 0
    CHANNEL_B = 1
    ENABLED = 1
    DC = 1
    RANGE_2V = 7
    ANALOGUE_OFFSET = 0
    ADC_1024_THRESHOLD = 64
    RISING = 0
    DELAY = 0
    AUTO_TRIGGER = 1000
    MAX_SAMPLES = 2000      # pre + post trigger samples
    TIME_BASE = 8
    
