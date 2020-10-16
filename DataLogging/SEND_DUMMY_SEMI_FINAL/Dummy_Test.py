import serial, time, spidev, smbus
import csv
import threading
from datetime import datetime
from SX127x.LoRa import *
 
import seven_segment_display
import seven_segment_i2c
#from SX127x.LoRaArgumentParser import LoRaArgumentParser
from SX127x.board_config import BOARD
 
BOARD.setup()
BOARD.reset()
#parser = LoRaArgumentParser("Lora tester")
 
segment_address1 = 0x71
segment_address2 = 0x42
 
bus1 = seven_segment_i2c.SevenSegmentI2c(1,segment_address1)
bus2 = seven_segment_i2c.SevenSegmentI2c(1,segment_address2)
 
display1 = seven_segment_display.SevenSegmentDisplay(bus1)
display1.clear_display()
 
display2 = seven_segment_display.SevenSegmentDisplay(bus2)
display2.clear_display()
 
spi_sensor = spidev.SpiDev()
spi_sensor.open(0,1)
spi_sensor.max_speed_hz = 1000000
 
curve1 = 1.0#0.1884565 / 4
curve2 = 1.0#0.188750 / 4
 
speed_curve1 = 1.0
speed_curve2 = 1.0

 
class MLX90614():
 
    MLX90614_RAWIR1=0x04
    MLX90614_RAWIR2=0x05
    MLX90614_TA=0x06
    MLX90614_TOBJ1=0x07
    MLX90614_TOBJ2=0x08
 
    MLX90614_TOMAX=0x20
    MLX90614_TOMIN=0x21
    MLX90614_PWMCTRL=0x22
    MLX90614_TARANGE=0x23
    MLX90614_EMISS=0x24
    MLX90614_CONFIG=0x25
    MLX90614_ADDR=0x0E
    MLX90614_ID1=0x3C
    MLX90614_ID2=0x3D
    MLX90614_ID3=0x3E
    MLX90614_ID4=0x3F
 
    def __init__(self, address=0x5a, bus_num=1):
        self.bus_num = bus_num
        self.address = address
        self.bus = smbus.SMBus(bus=bus_num)
 
    def read_reg(self, reg_addr):
        return self.bus.read_word_data(self.address, reg_addr)
 
 
    def get_amb_temp(self):
        data = self.read_reg(self.MLX90614_TA)
        return data
 
    def get_obj_temp(self):
        data = self.read_reg(self.MLX90614_TOBJ1)
        return data
 
 
class filter_member:
    storage=[]#voltage & current sensor
    point = 0
    avg= 0
    num = 150
    def __init__(self,channel,curve1,curve2):
        self.curve1 = curve1
        self.curve2 = curve2
        self.channel = channel
        
    def sensor_init(self):
        adc_value = analog_read(self.channel)
        if (adc_value >= 2124):
            value = adc_value * self.curve1
        else:
            value = adc_value * self.curve2 #We will change setting.
            
        storage_temp = [ value / self.num ]
        self.avg += (value / self.num )
            
        for i in range(self.num-1):
            adc_value = analog_read(self.channel)
            if (adc_value >= 2124):
                value = adc_value * self.curve1
            else:
                value = adc_value * self.curve2 #We will change setting.
                
            storage_temp.insert(i+1, value/self.num)
            self.avg += storage_temp[i]
        
        self.storage = storage_temp
    
    def sensor_reading(self):
 
            adc_value = analog_read(self.channel)
            if (adc_value>= 2124):
                value = adc_value*self.curve1
            else:
                value = adc_value*self.curve2   
 
            self.avg = (self.avg + (value/self.num))
            self.avg = self.avg - (self.storage[self.point])
            self.storage[self.point] = value/self.num
            self.point += 1
            
            if(self.point >= self.num):
                self.point= 0
    def write_to_csv():
        list = [current1.avg, current2.avg, current3.avg, voltage1.avg, Speed_Sensor.avg]
        with open(name,'a',newline='') as csvfile:
            data = csv.writer(csvfile)
            data.writerow(list)
            
    def seperate_float(SPEED):
        natural_num_list = [0,0,0,0,0,0,0,0,0,0] #list[10] is value of temperature
        
        natural_num_list[0] = int(SPEED)
        natural_num_list[1] = int(round(SPEED - natural_num_list[0],2) * 100)
        
        natural_num_list[2] = int(voltage1.avg)
        natural_num_list[3] = int(round(voltage1.avg - natural_num_list[2],2) * 100)
        
        natural_num_list[4] = int(current1.avg)
        natural_num_list[5] = int(round(current1.avg - natural_num_list[4],2) * 100)
        
        natural_num_list[6] = int(current2.avg)
        natural_num_list[7] = int(round(current2.avg - natural_num_list[6],2) * 100)
        
        natural_num_list[8] = int(current3.avg)
        natural_num_list[9] = int(round(current3.avg - natural_num_list[8],2) * 100)
        
        return natural_num_list
    
    def task1():
        while True:            
            current1.sensor_reading() #Motor
            current2.sensor_reading() #Solar_Cell
                
            current3.sensor_reading() #Battery
            voltage1.sensor_reading() #Battery
                
            Speed_Sensor.sensor_reading() #Speed
            
            #print(current1.avg)
            #print(current2.avg)
            #print(current3.avg)
            #print(voltage1.avg)
            #print(Speed_Sensor.avg)
            #print('-------------------------')
            #print(temperature.get_amb_temp() * 0.02 - 273.15)
            #print(temperature.get_obj_temp() * 0.02 - 273.15)
 
            # RPM = 370*(0.001220703*Speed_Sensor.avg)  Motor_RPM
            # SPEED = 0.27 * RPM * 0.10472 * 3.6
            # >>> 0.0459735070110624 * Speed_Sensor.avg
            SPEED = 0.045973507 * Speed_Sensor.avg # km/h
            
            #filter_member.write_to_csv()
                
            display1.set_nondigits([0b00000100, 0])
            display1.write_int(int(SPEED*10))
            display2.set_nondigits([0b00000100, 0])
            display2.write_int(int(voltage1.avg*10))#voltage1.avg*10
 
def analog_read(channel):
    r = spi_sensor.xfer2([6+((4&channel)>>2),(3&channel)<<6,0])
    adcValue = ((r[1] & 15) << 8) + r[2]
    return adcValue
 
 
class mylora(LoRa):
    def __init__(self, verbose=False):
        super(mylora, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)
        self.var=0
 
    def on_rx_done(self):
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)
        print ("Receive: ")
        print(bytes(payload).decode("utf-8",'ignore')) # Receive DATA
        BOARD.led_off()
        time.sleep(2) # Wait for the client be ready
        print ("Send: ACK")
        self.write_payload([255, 255, 0, 0, 65, 67, 75, 0]) # Send ACK
        self.var=1
 
    def on_tx_done(self):
        print("\nTxDone")
        print(self.get_irq_flags())
 
    def on_cad_done(self):
        print("\non_CadDone")
        print(self.get_irq_flags())
 
    def on_rx_timeout(self):
        print("\non_RxTimeout")
        print(self.get_irq_flags())
 
    def on_valid_header(self):
        print("\non_ValidHeader")
        print(self.get_irq_flags())
 
    def on_payload_crc_error(self):
        print("\non_PayloadCrcError")
        print(self.get_irq_flags())
 
    def on_fhss_change_channel(self):
        print("\non_FhssChangeChannel")
        print(self.get_irq_flags())
 
    def start(self):
        i=0
        while True:
            #print ("Send: DATA")
            #data_list = filter_member.seperate_float(Speed_Sensor.avg * 0.045973507)
            data_list = [i,0,0,0,0,0,0,0,0,0]
            self.write_payload(data_list) # Send INF
            self.set_mode(MODE.TX)
            #time.sleep(2) # there must be a better solution but sleep() works
            time.sleep(1.5)
            self.reset_ptr_rx()
            #time.sleep(2)
            #print('SEND')
            #print(data_list)
            i = i +1
 
lora = mylora(verbose=False)
#args = parser.parse_args(lora) # configs in LoRaArgumentParser.py
 
#     Slow+long range  Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. 13 dBm
lora.set_pa_config(pa_select=1, max_power=21, output_power=15)
lora.set_bw(BW.BW125)
lora.set_coding_rate(CODING_RATE.CR4_8)
lora.set_spreading_factor(12)
lora.set_rx_crc(True)
#lora.set_lna_gain(GAIN.G1)
#lora.set_implicit_header_mode(False)
lora.set_low_data_rate_optim(True)
 
#  Medium Range  Defaults after init are 434.0MHz, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on 13 dBm
#lora.set_pa_config(pa_select=1)
 
 
assert(lora.get_agc_auto_on() == 1)
 
temperature = MLX90614()
thread = threading.Thread(target = filter_member.task1)
try:
    print("START")
    
    current1 = filter_member(1,curve1,curve2)
    current2 = filter_member(2,curve1,curve2)
    current3 = filter_member(4,curve1,curve2)
    voltage1 = filter_member(0,curve1,curve2)
    Speed_Sensor = filter_member(3,speed_curve1,speed_curve2)
 
    current1.sensor_init()
    current2.sensor_init()
    current3.sensor_init()
    voltage1.sensor_init()
    Speed_Sensor.sensor_init()

    thread.start()
    lora.start()
except KeyboardInterrupt:
    sys.stdout.flush()
    print("Exit")
    sys.stderr.write("KeyboardInterrupt\n")
finally:
    sys.stdout.flush()
    print("Exit")
    lora.set_mode(MODE.SLEEP)
    BOARD.teardown()
 