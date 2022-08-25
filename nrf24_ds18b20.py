"""Test for nrf24l01 module.  Portable between MicroPython targets."""

import usys
import ustruct as struct
import utime

from machine import Pin, SPI
from nrf24l01 import NRF24L01
from micropython import const
from struct import *
from onewire import OneWire
from ds18x20 import DS18X20


unitID = 0xa5

Cycle = {"init":1, "listen" : 2, "post":3, "wait":4, "prepare sensor":5, "read sensor":6}
PacketType = {"get" : 0, "init":1 , "ds18b20":3}

def getDictText(Dict,value):
    for i in Dict.keys():
        if Dict[i]== value:
            return i
    return "???"


class ReceivePacket:
    def __init__(self):
        self.header=0
        self.structSize=12
        self.structType=0
        self.txmUnitId=1
        self.currentTime=0
        self.nextTimeReading=0
        self.nextTimeOnTimeOut=0
        
    def Extract(self,Data):
        try:
            (self.header,self.structSize,self.structType,
            self.txmUnitId,self.currentTime,
            self.nextTimeReading,self.nextTimeOnTimeOut) = unpack("<bBBBLHH",Data)
            return True
        except struct.error:
            return False
                            
    def Print(self):
        print("==== Rcv Packet ====")
        print("header=",self.header)
        print("Size=",self.structSize)
        print("Type=",getDictText(PacketType,self.structType))
        print("UnitId=",self.txmUnitId)
        print("self.txmUnitId=",self.txmUnitId)
        print("self.currentTime=",self.currentTime)
        print("self.nextTimeReading=",self.nextTimeReading)
        print("self.nextTimeOnTimeOut=",self.nextTimeOnTimeOut)
        print(" ")
         

if usys.platform == "pyboard":
    cfg = {"spi": 2, "miso": "Y7", "mosi": "Y8", "sck": "Y6", "csn": "Y5", "ce": "Y4"}
elif usys.platform == "esp8266":  # Hardware SPI
    cfg = {"spi": 1, "miso": 12, "mosi": 13, "sck": 14, "csn": 4, "ce": 5}
elif usys.platform == "esp32":  # Software SPI
    cfg = {"spi": -1, "miso": 32, "mosi": 33, "sck": 25, "csn": 26, "ce": 27}
elif usys.platform == "rp2": # pico
    cfg = {"spi": 0, "miso": 20, "mosi": 19, "sck": 18, "csn": 21, "ce": 28}
else:
    raise ValueError("Unsupported platform {}".format(usys.platform))


#pipes = ([0x1,0xE0,0xE0,0xE0,0xE0], [unitID,0xa5,0xa5,0xa5,0xa5])
pipes = ([0x1,0xE0,0xE0,0xE0,0xE0], [unitID,0xa5,0xa5,0xa5,0xa5])


def getSPI():
    if usys.platform == "rp2":
        return  SPI(cfg['spi'],baudrate=400000,sck=Pin(cfg["sck"]), mosi=Pin(cfg["mosi"]), miso=Pin(cfg["miso"]))
    else:
        if cfg["spi"] == -1:
            return SPI(-1, sck=Pin(cfg["sck"]), mosi=Pin(cfg["mosi"]), miso=Pin(cfg["miso"]))
    return SPI(cfg["spi"])



def setPacket(Type,UnitID=unitID,Status=0,stampTime=0,CpuVoltage=0,temperature=0):
   packet=pack("<bBBBBLHh",b'*'[0],13,Type,UnitID,Status,stampTime,CpuVoltage,temperature)
   #print("packet : ",packet)
   #for i in packet: print(hex(i),end=" ")
   #print("")
   #for i in packet: print(i,end=" ")
   #print("")
   
   return packet
   


cycle = Cycle["init"]
oldcycle = 999

csn = Pin(cfg["csn"], mode=Pin.OUT, value=1)
ce = Pin(cfg["ce"], mode=Pin.OUT, value=0)
spi = getSPI()
nrf = NRF24L01(spi, csn, ce,channel=80, payload_size=32)
nrf.open_tx_pipe(bytes(pipes[0]))
nrf.open_rx_pipe(1, bytes(pipes[1]))
nrf.setAutoAck(True)
nrf.enableDynamicPayloads()
nrf.enableAckPayload()
nrf.stop_listening()
nrf.start_listening()
rcvPacket = ReceivePacket()
WakeTime= utime.time()

timestamp=0;

#cpu temperature declaration
cpu_temp = machine.ADC(machine.ADC.CORE_TEMP)
conversion_factor = 3.3 / (65535)

def readCpuTemperature():
  reading = cpu_temp.read_u16() * conversion_factor
  return 27 - (reading - 0.706)/0.001721


def setPad(gpio, value):
    machine.mem32[0x4001c000 | (4+ (4 * gpio))] = value
    
def getPad(gpio):
    return machine.mem32[0x4001c000 | (4+ (4 * gpio))]


#GP 25 & GP 29 are use in PICOW for wifi
# needs to temporary set them to read Vsys
# ok pico is different from pico W
def readVsys():
    if usys.implementation._machine.find("Pico W")>=0:
        p25pad = getPad(25)
        p25=machine.Pin(25,machine.Pin.OUT)
        p25.value(1)
        oldpad = getPad(29)
        setPad(29,128)  #no pulls, no output, no input
        utime.sleep_ms(50)
        adc_Vsys = machine.ADC(3)
        Vsys = adc_Vsys.read_u16() * 3.0 * conversion_factor
        setPad(29,oldpad)
        setPad(25,p25pad)
        return Vsys
    if usys.implementation._machine.find("Pico")>=0:
        Pin29 = machine.Pin(29,machine.Pin.IN)
        utime.sleep_ms(50)
        adc_Vsys = machine.ADC(29)
        Vsys = adc_Vsys.read_u16() * 3.0 * conversion_factor
        return Vsys
    return 0.0

InitFlag=True

# set one wire for ds18b20
ds = DS18X20(OneWire(machine.Pin(28)))

sensorValid=0
sensor=[]
sensorValue = 999.9
while True:

    if oldcycle != cycle:
      print("cycle :",cycle,"  ",getDictText(Cycle,cycle))
      oldcycle = cycle

    if cycle == Cycle["init"]:
        packet = setPacket(Type=PacketType["init"])
        nrf.writeAckPayload(1,packet)
        cycle = Cycle["listen"]
        ListenTimeStart = utime.time()

    elif cycle == Cycle["post"]:
        v1 = readVsys()
        print("Vsys =",v1)
        v = int(v1 * 1000.0)
        if sensorValid==1:
            print("temperature=",sensorValue)
            t = int(sensorValue * 100.0)
        else:
            t = 0x7fff
        packet = setPacket(Type=PacketType["ds18b20"],Status=sensorValid,
                           stampTime= rcvPacket.currentTime + utime.time() - timestamp,
                           CpuVoltage=v,temperature=t)
        print("post", packet)
        nrf.writeAckPayload(1,packet)
        cycle = Cycle["listen"]
        ListenTimeStart = utime.time()
    elif cycle == Cycle["listen"]:
        if nrf.any():
        #if nrf.available():    
            rcv_size = nrf.getDynamicPayloadSize()
            rcv_data = nrf.recv()
            #print("got ",rcv_size," bytes =", rcv_data)
            if rcvPacket.Extract(rcv_data):
                rcvPacket.Print()
                if(rcvPacket.structType!=0):  continue
                if(rcvPacket.header!= 0x2a):  continue
                timestamp=utime.time()
                cycle=Cycle["wait"]
            else:
                print("Receive Packet invalid")
    elif cycle == Cycle["wait"]:
        # calculate sleep mode time
        if rcvPacket.nextTimeReading<0:
            rcvPacket.nextTimeReading= 300
        if rcvPacket.nextTimeReading>3600:
            rcvPacket.nextTimeReading= 300
        print("go sleep for",rcvPacket.nextTimeReading, " seconds")            

        #todo
        #need to change nrf24l01 code to shutdown and wake up
        #utime.sleep_ms(100) #lets print to finish
        #nrf.shutdown()
        #machine.lightsleep(rcvPacket.nextTimeReading * 1000)
        utime.sleep(rcvPacket.nextTimeReading)
        
        print("wake up")        
        cycle = Cycle['prepare sensor']

    elif cycle == Cycle["prepare sensor"]:
        #Scanfor ds18b20 sensor
        sensors = ds.scan()
        if len(sensors)==0:
            sensorValid=0
        else:
            sensorValid=1
            sensor = sensors[0]
            #start conversion
            ds.convert_temp()
            #wait at least 750ms
            utime.sleep_ms(750)
            cycle = Cycle["read sensor"]
    elif cycle == Cycle["read sensor"]:
        if sensorValid==1:
            #read sensor
            sensorValue = ds.read_temp(sensor)
        cycle = Cycle["post"]
    else:
        cycle = Cycle["init"]
    