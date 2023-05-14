'''
Author: Jasper Doan
Date:   05/12/2023
Desc:   This file contains the class definition PicoScope 2000 series. The
            purpose is to read off voltage data from the oscilloscope, and
            plot them for analysis from the Science subteam.

Notes:  Is this perfect? No. Is it good enough? Yes.
        Why did did you have to bring me back into Robotics Renee? I was
        happy with my life, and you ruined it. 🥰 I hate you 🥰 

Config:
    Channel A:                              Channel B:
        handle = chandle                        handle = chandle
        channel = PS2000_CHANNEL_A = 0          channel = PS2000_CHANNEL_B = 1
        enabled = 1                             enabled = 1
        coupling type = PS2000_DC = 1           coupling type = PS2000_DC = 1
        range = PS2000_2V = 7                   range = PS2000_2V = 7
        analogue offset = 0 V                   analogue offset = 0 V
'''

import ctypes
import numpy as np
import matplotlib.pyplot as plt

from .pico_config import PS2000
from picosdk.ps2000 import ps2000 as ps
from picosdk.functions import adc2mV, assert_pico2000_ok


class Oscilloscope:
    '''
    This class is a wrapper for the PicoScope 2000 series API.
        It is designed to be used with a context manager.
    '''

    def __init__(self):
        self.status = {}

        self.chandle = None
        self.timeInterval = None
        self.timeUnits = None
        self.over_sample = None
        self.max_sample_ret = None
        self.time_indisposed_ms = None

        self.status["openUnit"] = ps.ps2000_open_unit()
        assert_pico2000_ok(self.status["openUnit"])


    def capture(self) -> None:
        '''
        This function is the main function that will be called to capture
            data from the oscilloscope.
        '''
        self.chandle = self.open_unit()

        self.set_channel_A()
        self.set_channel_B()

        self.set_trigger()

        self.get_timebase()

        self.block_capture()

        # self.plot(*self.get_data())




    def open_unit(self) -> ctypes.c_int16:
        '''
        Open 2000 series PicoScope
        Returns handle to chandle for use in future API functions
        '''
        return ctypes.c_int16(self.status["openUnit"])
    

    def set_channel_A(self) -> None:
        '''
        Set up channel A
            handle          = chandle
            channel         = PS2000_CHANNEL_A = 0
            enabled         = 1
            coupling type   = PS2000_DC = 1
            range           = PS2000_2V = 7
            analogue offset = 0 V
        '''
        self.status["setChA"] = ps.ps2000_set_channel(self.chandle,
                                                      PS2000.CHANNEL_A,
                                                      PS2000.ENABLED,
                                                      PS2000.DC,
                                                      PS2000.RANGE_2V)
        assert_pico2000_ok(self.status["setChA"])
    

    def set_channel_B(self) -> None:
        '''
        Set up channel B
            handle          = chandle
            channel         = PS2000_CHANNEL_B = 1
            enabled         = 1
            coupling type   = PS2000_DC = 1
            range           = PS2000_2V = 7
            analogue offset = 0 V
        '''
        self.status["setChB"] = ps.ps2000_set_channel(self.chandle,
                                                      PS2000.CHANNEL_B,
                                                      PS2000.ENABLED,
                                                      PS2000.DC,
                                                      PS2000.RANGE_2V)
        assert_pico2000_ok(self.status["setChB"])
    

    def set_trigger(self) -> None:
        '''
        Set up trigger
            handle      = chandle
            source      = PS2000_CHANNEL_A = 0
            threshold   = 1024 ADC counts
            direction   = PS2000_RISING = 2
            delay       = 0 s
            autoTrigger = 1000 ms
        '''
        self.status["setTrigger"] = ps.ps2000_set_trigger(self.chandle,
                                                          PS2000.CHANNEL_A,
                                                          PS2000.ADC_1024_THRESHOLD,
                                                          PS2000.RISING,
                                                          PS2000.DELAY,
                                                          PS2000.AUTO_TRIGGER)
        assert_pico2000_ok(self.status["setTrigger"])
    

    def get_timebase(self) -> None:
        '''
        Gets timebase
            handle              = chandle
            timebase            = 8 = timebase
            no_of_samples       = maxSamples = 2000
            oversample          = 1
            ptr to timeInterval = ctypes.byref(timeInterval)
            ptr to timeUnits    = ctypes.byref(timeUnits)
            ptr to maxSamples   = ctypes.byref(maxSamples) 
        '''
        self.timeInterval = ctypes.c_int32()
        self.timeUnits = ctypes.c_int32()
        self.over_sample = ctypes.c_int16(1)
        self.max_sample_ret = ctypes.c_int32()

        self.status["getTimebase"] = ps.ps2000_get_timebase(self.chandle,
                                                            PS2000.TIME_BASE,
                                                            PS2000.MAX_SAMPLES,
                                                            ctypes.byref(self.timeInterval),
                                                            ctypes.byref(self.timeUnits),
                                                            self.over_sample,
                                                            ctypes.byref(self.max_sample_ret))
        assert_pico2000_ok(self.status["getTimebase"])
    

    def block_capture(self) -> None:
        '''
        Captures a block of data and stores it in the driver buffer
            handle          = chandle
            no_of_samples   = maxSamples = 2000
            timebase        = 8 = timebase
            oversample      = 1
            ptr to t_ind_ms = ctypes.byref(time_indisposed_ms)
        '''
        self.time_indisposed_ms = ctypes.c_int32()
        self.status["runBlock"] = ps.ps2000_run_block(self.chandle,
                                                      PS2000.MAX_SAMPLES,
                                                      PS2000.TIME_BASE,
                                                      self.over_sample,
                                                      ctypes.byref(self.time_indisposed_ms))
        assert_pico2000_ok(self.status["runBlock"])
    

    def get_data(self):
        '''
        Gets data from the driver buffer
            handle              = chandle
            start_index         = startIndex = 0
            no_of_samples       = maxSamples = 2000
            downsample_ratio    = downSampleRatio = 1
            downsample_ratio_m  = ctypes.byref(downSampleRatioMode)
            ptr to overflow     = ctypes.byref(overflow)
        '''
        # Check for data collection to finish using ps5000aIsReady
        ready = ctypes.c_int16(0)
        check = ctypes.c_int16(0)
        while ready.value == check.value:
            self.status["isReady"] = ps.ps2000_ready(self.chandle)
            ready = ctypes.c_int16(self.status["isReady"])

        # Create buffers ready for data
        bufferA = (ctypes.c_int16 * PS2000.MAX_SAMPLES)()
        bufferB = (ctypes.c_int16 * PS2000.MAX_SAMPLES)()

        # Get data from scope
        cmaxSamples = ctypes.c_int32(PS2000.MAX_SAMPLES)
        self.status["getValues"] = ps.ps2000_get_values(self.chandle, 
                                                        ctypes.byref(bufferA), 
                                                        ctypes.byref(bufferB), 
                                                        None, 
                                                        None, 
                                                        ctypes.byref(self.over_sample), 
                                                        cmaxSamples)
        assert_pico2000_ok(self.status["getValues"])

        # Find maximum ADC count value
        maxADC = ctypes.c_int16(32767)

        # convert ADC counts data to mV
        adc2mVChA =  adc2mV(bufferA, PS2000.RANGE_2V, maxADC)
        adc2mVChB =  adc2mV(bufferB, PS2000.RANGE_2V, maxADC)

        # Create time data
        time = np.linspace(0, (cmaxSamples.value -1) * self.timeInterval.value, cmaxSamples.value)

        return time, adc2mVChA, adc2mVChB


    # def plot(self, time, adc2mVChA, adc2mVChB) -> None:
    #     '''
    #     Plot data
    #     '''
    #     plt.plot(time, adc2mVChA[:])
    #     plt.plot(time, adc2mVChB[:])
    #     plt.xlabel('Time (ns)')
    #     plt.ylabel('Voltage (mV)')
    #     plt.show()

    #     self.close_unit()
    

    def close_unit(self):
        '''
        Closes the unit
        '''
        # Stop the scope
        self.status["stop"] = ps.ps2000_stop(self.chandle)
        assert_pico2000_ok(self.status["stop"])

        # Close unitDisconnect the scope
        self.status["close"] = ps.ps2000_close_unit(self.chandle)
        assert_pico2000_ok(self.status["close"])

        # display status returns
        print(self.status)