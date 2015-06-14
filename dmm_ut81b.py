#!/usr/bin/python
"""
Adam Pribyl, pribyl@lowlevel.cz, 10/2014, GPLv3

This a is python script to read out a measurements and graphs from UNI-TREND UNI-T UT81B scopemeter

Based on http://hmijailblog.blogspot.cz/2011/12/custom-software-for-interfacing-via-usb.html
and http://www.lowlevel.cz/log/files/UT81B%20communication%20protocol_en.txt

USB snoop: Working or not? http://lindi.iki.fi/lindi/usb/usbsnoop.txt
Snooped with Wireshark on Linux with WXP as a virtual http://wiki.wireshark.org/CaptureSetup/USB


     Sanitize ranges, try to read data to avoid exception failure
v1.2 Add more info needed to reconstruct the graphs in continuous capture mode
     Make the trigger scale and axis more accurate
v1.1 Rework the getAnswer routine for more reliable readouts
v1.0 Initial release

"""

import pylab
import numpy
import optparse
import logging
import time
import usb
import signal
import sys

LOGGING_LEVELS = {'critical': logging.CRITICAL,
                  'error': logging.ERROR,
                  'warning': logging.WARNING,
                  'info': logging.INFO,
                  'debug': logging.DEBUG}

timeout = 5

timebase = {
    0 :   (1,"ns"),
    1 :   (2,"ns"),
    2 :   (5,"ns"),
    3 :   (10,"ns"),
    4 :   (20,"ns"),
    5 :   (50,"ns"),
    6 :   (100,"ns"),
    7 :   (200,"ns"),
    8 :   (500,"ns"),
    9 :   (1,"us"),
    0xA : (2,"us"),
    0xB : (5,"us"),
    0xC : (10,"us"),
    0xD : (20,"us"),
    0xE : (50,"us"),
    0xF : (100,"us"),
    0x10: (200,"us"),
    0x11: (500,"us"),
    0x12: (1,"ms"),
    0x13: (2,"ms"),
    0x14: (5,"ms"),
    0x15: (10,"ms"),
    0x16: (20,"ms"),
    0x17: (50,"ms"),
    0x18: (100,"ms"),
    0x19: (200,"ms"),
    0x1A: (500,"ms"),
    0x1B: (1,"s"),
    0x1C: (2,"s"),
    0x1D: (5,"s")
    }
modes_voltage = {
    0 :   (20,"mV"),
    1 :   (50,"mV"),
    2 :   (100,"mV"),
    3 :   (200,"mV"),
    4 :   (500,"mV"),
    5 :   (1,"V"),
    6 :   (2,"V"),
    7 :   (5,"V"),
    8 :   (10,"V"),
    9 :   (20,"V"),
    0xA:  (50,"V"),
    0xB:  (100,"V"),
    0xC:  (200,"V"),
    0xD:  (500,"V")
    }
modes_amperage = {
    0 :   (20,"uA"),
    1 :   (50,"uA"),
    2 :   (100,"uA"),
    3 :   (200,"uA"),
    4 :   (500,"uA"),
    5 :   (1,"mA"),
    6 :   (2,"mA"),
    7 :   (5,"mA"),
    8 :   (10,"mA"),
    9 :   (20,"mA"),
    0xA:  (50,"mA"),
    0xB:  (100,"mA"),
    0xC:  (200,"mA"),
    0xD:  (500,"mA"),
    0xE:  (1,"A"),
    0xF:  (2,"A"),
    0x10: (5,"A")
    }
modes_resistance = {
    0 :  (400, "Ohm"),
    1 :  (4, "KOhm"),
    2 :  (40, "KOhm"),
    3 :  (400, "KOhm"),
    4 :  (4, "MOhm"),
    5 :  (40, "MOhm")
    }


def connect(device_info): # sends command used by MMeter software when pressing the "Connect" button
    logging.debug('Connect DMM')
    device_info.ctrl_transfer(0x21, 0x9, 0, 0, (0x80,0x25,0,0,3))

def disconnect(device_info): # sends command used by MMeter software when pressing the "Disconnect" button
    logging.debug('Disconnect DMM')
    device_info.ctrl_transfer(0x22, 0x9, 0, 0, (0x80,0x25,0,0,3))

def ask(ep): # causes the multimeter to dump a packet of data (up to 401 bytes)
    logging.debug('Ask for data')
    try:
        ep.write((2,0x5a,0,0,0,0,0,0), 0x300)
    except:
        logging.debug('Ask for data error')

def getAnswer(ep): #return list of read bytes; stop reading when timeout or when 10 empty trains appear after some bytes were already read
    result=[]
    t0 = time.time()
    bytesRead = 0
    t = 0
    emptyTrains = 0
    bytesToRead = 361

    # stop condition
    while not (len(result) >= (bytesToRead + 6) or t > timeout or (len(result) > 0 and emptyTrains > 10) ): 
        try:
            output = ep.read(8, 100)
        except:
            logging.debug('Read timeout')
            break
#        print output
        t = time.time() - t0
        actualBytesInOutput = output[0] & 15

        if actualBytesInOutput != 0:
            emptyTrains = 0
            t0 = time.time()
            result.extend(output[1:actualBytesInOutput+1])
            if result[0] == 0x5a: 
                if len(result) >= 8:
                    bytesToRead = result[1]*1000 + result[2]*100 + result[3]*10 + result[4]
            else:
                result = []

        else:
            emptyTrains = emptyTrains + 1


    logging.debug("Bytest to read: %d Bytes: %d (6b header %db data) Empty reads: %d Timeout: %f", bytesToRead, len(result), len(result) - 6, emptyTrains, t)
    return result

def dmmInit():
    d = usb.core.find(idVendor=0x1a86, idProduct=0xe008)

    logging.debug("Found device: %s ", d)

    # Did we found a device?
    if d is None:
        raise ValueError('Device not found')
    try:
        d.detach_kernel_driver(0)
    except: # this usually means that kernel driver has already been dettached
        pass

    # Set default configuration and claim the interface
    d.set_configuration()
    cfg = d.get_active_configuration()
    intf = cfg[(0,0)]
    usb.util.claim_interface(d, cfg[(0,0)].bInterfaceNumber)

    # Get IN end point
    ei = usb.util.find_descriptor(
        intf,
        # match the first IN endpoint
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_IN)

    # Get OUT end point
    eo = usb.util.find_descriptor(
        intf,
        # match the first OUT endpoint
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_OUT)

    assert ei is not None
    assert eo is not None

    return [d, ei, eo, cfg]

def dmmGetData(device):
    d = device[0]
    ei = device[1]
    eo = device[2]
    cfg = device[3]

    #clearBuffer(ei)
    ask(eo)
    data = getAnswer(ei)
    logging.debug('Raw data: %s', data)

    if not data or data[0] != 0x5A or len(data) < 41:
        return None
    else:
        return data


def dmmGetRange(data):
    # if we are here, we had no USB error, no timeout and a good header, so we assume a good answer
    mode=data[6]
    if (mode == 0x00 or mode == 0x80):
        if (data[11] < len(modes_voltage)):
            mRange = [modes_voltage[data[11]][0], modes_voltage[data[11]][1], "DC" if data[10] == 0 else "AC"]
    elif (mode == 0x01 or mode == 0x81):
        if (data[11] < len(modes_amperage)):
            mRange = [modes_amperage[data[11]][0], modes_amperage[data[11]][1], "DC" if data[10] == 0 else "AC"]
    elif (mode == 0x02 or mode == 0x82):
        mRange = ["-", "Hz", ""]
    elif (mode == 0x03):
        mRange = ["-", "F", ""]
    elif (mode == 0x04):
        if (data[18] < len(modes_resistance)):
            mRange = [modes_resistance[data[18]][0], modes_resistance[data[18]][1], ""]
    elif (mode == 0x05):
        mRange = ["-", "OFF", ""]
    elif (mode == 0x06):
        mRange = ["-", "Diode", ""]
    else:
        logging.info('Mode is unknown')
        return
    if (mRange == None):
        logging.info('Incorrect range value')

    return mRange

def dmmDisplayText(data, mRange):
    mrauto = ""
    tbauto = ""

    if data[8]==1:
        mrauto = "[AUTO]"

    if data[9]==1:
        tbauto = "[AUTO]"

    measurement = "RUN" if data[7]==1 else "HOLD"

    printout=""
    for i in range(20,40):     #ASCII decoding
        try:
            c=chr(data[i]) if data[i]!=0 else ' '
            printout+=c
        except IndexError:  #part of the expected ASCII is missing!
            printout+= ' '

    if options.cont_cap:
        logging.info("Range: %d %s %s %s  Timebase: %d %s %s Measurement: %s  Readout: %s", mRange[0], mRange[1], mRange[2], mrauto, \
            timebase[data[13]][0], timebase[data[13]][1], tbauto, measurement, printout)
    else:
        print "Range:", mRange[0], mRange[1], mRange[2], mrauto
        print "Time base:",timebase[data[13]][0],timebase[data[13]][1], tbauto
        print "Measurement:", measurement
        print "Actual readout:", printout

def dmmProcessData(data, mRange):
    # Display graph if we received data with plot
    if len(data) == 361:

        # Scope screen has 320 i.e. 40pix/div points on x axis
        x = numpy.linspace(0, 8*timebase[data[13]][0], 320) 

        # Scope screen has 128 (-64:64) i.e. 16pix/div points on y axis [x/myInt for x in myList]
        const = float(1)/64*mRange[0]*4
        point = data[12] if data[12]<127 else -(255-data[12])
        offset = float(point)*const
        offsety = [0 for point in data[40:360]]
        iy = [point if point<127 else -(255-point) for point in data[40:360]]
        y = [(float(point)*const) for point in iy]

        # Trigger time -76:76 (-160:160)
        trigx = numpy.linspace(-64*const-offset, 64*const-offset, 128)
        tpointx = data[14] if data[14]<127 else -(255-data[14])
        tpointx = (4*timebase[data[13]][0])+(tpointx*8*timebase[data[13]][0]/160)
        trigxx = [tpointx for point in trigx]

        # Trigger level -60:60 (-64:64)
        tpointy = data[16] if data[16]<127 else -(255-data[16])
        tpointy = float(tpointy)*const-offset
        trigy = [tpointy for point in data[40:360]]

        if options.cont_cap:
            logging.info('MeasuredData %s TotalTime: %d %s TotalRange: %d .. %d %s TrigTime: %.2f %s TrigValue: %.2f %s BaseOffset: %.2f %s Min: %.2f %s Max: %.2f %s',
                         y, 8*timebase[data[13]][0], timebase[data[13]][1], -mRange[0]*4-offset, mRange[0]*4-offset, mRange[1], tpointx, timebase[data[13]][1], tpointy, mRange[1], offset, mRange[1],
                         min(y[1:]), mRange[1], max(y[1:]), mRange[1]
                        )
        else:
            logging.info('TotalTime: %d %s TotalRange: %d .. %d %s TrigTime: %.2f %s TrigValue: %.2f %s BaseOffset: %.2f %s Min: %.2f %s Max: %.2f %s',
                         8*timebase[data[13]][0], timebase[data[13]][1], -mRange[0]*4-offset, mRange[0]*4-offset, mRange[1], tpointx, timebase[data[13]][1], tpointy, mRange[1], offset, mRange[1],
                         min(y[1:]), mRange[1], max(y[1:]), mRange[1]
                        )
            # http://matplotlib.org/users/pyplot_tutorial.html
            pylab.figure(1, figsize=(6,6))

            # Plot the complete image
            pylab.plot(x, y,  x, trigy, 'r--',  trigxx, trigx, 'r--',  x, offsety, 'g--')

            # Scope screen shows 8 timebase segments
            pylab.axis([0, 8*timebase[data[13]][0], -mRange[0]*4-offset, mRange[0]*4-offset])
            pylab.xlabel(timebase[data[13]][1])
            pylab.ylabel(mRange[1])
            pylab.grid(True)
            fig = pylab.gcf()
            fig.canvas.set_window_title('Scopemeter UT81B')

            if data[17] == 2:
                TrigMode = 'SHOT' 
            elif data[17] == 1:
                TrigMode = 'NORM'
            else:
                TrigMode = 'AUTO'
            TrigEdge = 'Rising' if data[15] == 0 else 'Falling'
            TrigText = 'Slope: '+TrigEdge+' Mode: '+TrigMode
            pylab.figtext(0.13, 0.91, mRange[2]+' '+str(mRange[0])+mRange[1]+'  '+str(timebase[data[13]][0])+timebase[data[13]][1]+'  '+TrigText)
            pylab.show()


    elif len(data) == 41:
        time.sleep(timeout)
    else:
        logging.info('Malformed data')

def exitGracefully(signum, frame):
    # restore the original signal handler 
    signal.signal(signal.SIGINT, original_sigint)

    disconnect(device[0])
    usb.util.release_interface(device[0], device[3][(0,0)].bInterfaceNumber)
    sys.exit(1)


if __name__ == "__main__":
    # store the original SIGINT handler
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exitGracefully)

    parser = optparse.OptionParser()
    parser.add_option('-l', '--logging-level', help='Logging level, Default=INFO')
    parser.add_option('-f', '--logging-file', help='Logging file name')
    parser.add_option('-c', '--continuous-capture', help='Continuously captures data in CONT_CAP[s] interval, Use shorter for faster triggers',  dest='cont_cap')
    (options, args) = parser.parse_args()

    # Just to enable easy logging/message printing
    logging_level = LOGGING_LEVELS.get(options.logging_level, logging.INFO)
    logging.basicConfig(level=logging_level, filename=options.logging_file,
            format='%(asctime)s %(levelname)s: %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S')

    device = dmmInit()

    connect(device[0])

    if options.cont_cap:
        timeout = int(options.cont_cap)

    data = dmmGetData(device)

    if options.cont_cap:
        while True:
            if (data != None):
                mRange = dmmGetRange(data)
                dmmDisplayText(data, mRange)
                dmmProcessData(data, mRange)
            data = dmmGetData(device)


    if (data != None):
        mRange = dmmGetRange(data)
        dmmDisplayText(data, mRange)
        dmmProcessData(data, mRange)

    disconnect(device[0])
    usb.util.release_interface(device[0], device[3][(0,0)].bInterfaceNumber)

