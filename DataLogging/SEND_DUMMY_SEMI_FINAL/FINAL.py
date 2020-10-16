import serial, time, spidev, smbus, threading, csv
import seven_segment_display
import seven_segment_i2c
import datetime
from SX127x.LoRa import *
from SX127x.board_config import BOARD

SOC_DATA = 0

BOARD.setup()
BOARD.reset()

SOC_DATA_READ = open('SOC190714.csv','r',encoding=('UTF-8'))
SOC_DATA = float(SOC_DATA_READ.readline())
SOC_DATA_READ.close()

SOC_DATA_WRITE = open('SOC190714.csv','w',encoding=('UTF-8'))
SOC_DATA_WRITE.write(str(SOC_DATA))
SOC_DATA_WRITE.close()

segment_address1 = 0x71
segment_address2 = 0x42

bus1 = seven_segment_i2c.SevenSegmentI2c(1,segment_address1)
bus2 = seven_segment_i2c.SevenSegmentI2c(1,segment_address2)

display1 = seven_segment_display.SevenSegmentDisplay(bus1)
display1.clear_display()
display2 = seven_segment_display.SevenSegmentDisplay(bus2)
display2.clear_display()

spi_sensor = spidev.SpiDev()
spi_sensor.open(0,0)
spi_sensor.max_speed_hz = 1000000

lock = threading.Lock()

Data_list = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

def analog_read(channel):
    r = spi_sensor.xfer2([6+((4&channel)>>2),(3&channel)<<6,0])
    adcValue = ((r[1] & 15) << 8) + r[2]
    return adcValue

class Data:
    storage=[]
    point = 0
    avg= 0
    num = 100
    MAX = 1
    ADC4096 = 4096
    
    def __init__(self,channel,MAX):
        self.channel = channel
        self.MAX = MAX
    def sensor_init(self):
        adc_value = analog_read(self.channel) * MAX / ADC4096

        storage_temp = [ adc_value / self.num ]
        self.avg += (adc_value / self.num )
            
        for i in range(self.num-1):
            adc_value = analog_read(self.channel)* MAX / ADC4096
            storage_temp.insert(i+1, adc_value/self.num)
            self.avg += storage_temp[i]
        
        self.storage = storage_temp
    
    def sensor_reading(self):
            adc_value = analog_read(self.channel)* MAX / ADC4096
            self.avg = (self.avg + (adc_value/self.num))
            self.avg = self.avg - (self.storage[self.point])
            self.storage[self.point] = adc_value/self.num
            self.point += 1
            
            if(self.point >= self.num):
                self.point= 0
    
class SUMSUM:
    SOC = 0
    DISTANCE = 0
    
    def __init__(self,get_soc):
        self.SOC = get_soc
            
    def calc(self,dt):
        if((current1.avg + current3.avg)-current2.avg > -0.05):
            self.SOC = self.SOC - abs(current1.avg + current3.avg - current2.avg)*dt
        else:
            self.SOC = self.SOc + (current2.avg - current1.avg - current3.avg)*dt
        
        self.DISTANCE = self.DISTANCE + (dt * Speed_Sensor.avg /3.6) #km
        #print(self.SOC)

def write_soc_csv():
    with open('SOC190714.csv','w',encoding=('utf-8')) as csvfile:
        csvfile.write(str(SUM.SOC))

def seperate_float():
    global Data_list
    #print(current1.avg,' ',current2.avg,' ',current3.avg*50/4096,' ',voltage1.avg*200/4096)
    Data_list[0] = int(current1.avg)                                     #Current1_Motor
    Data_list[1] = int(round(current1.avg - Data_list[0],2) * 100)

    Data_list[2] = int(current2.avg)                                     #Current2_mppt
    Data_list[3] = int(round(current2.avg - Data_list[2],2) * 100)
        
    Data_list[4] = int(current3.avg)                                     #Current3_Converter
    Data_list[5] = int(round(current3.avg - Data_list[4],2) * 100)
    
    Data_list[6] = int(voltage1.avg+0.3)                                     #Voltage
    Data_list[7] = int(round((voltage1.avg+0.3) - Data_list[6],2) * 100)
    #print(Data_list[6]+Data_list[7]*0.01)
    Data_list[8] = int(Speed_Sensor.avg)                                 #SPEED
    Data_list[9] = int(round(Speed_Sensor.avg - Data_list[8],2) * 100)

    Data_list[10] = int(SUM.SOC)                                         #SOC
    Data_list[11] = int(round(SUM.SOC - Data_list[10],2) * 100)
        
    Data_list[12] = int(SUM.DISTANCE)                                    #Distance
    Data_list[13] = int(round(SUM.DISTANCE - Data_list[12],2) * 100)
    
    Data_list[14] = int(current4.avg)                                    #Solar_Chargeing
    Data_list[15] = int(round(current4.avg - Data_list[14],2) * 100)
    #print(current1.avg*250/4096)
    #print(Speed_Sensor.avg)
    #print(voltage1.avg*200/4096)
    
def task1():
    delta = 0
    while True:
        time1 = datetime.datetime.today()
        SUM.calc(delta)
        current1.sensor_reading() #Motor
        current2.sensor_reading() #Solar_Cell
        current3.sensor_reading() #Battery
        voltage1.sensor_reading() #Battery
        Speed_Sensor.sensor_reading() #Speed
        #SUM.dt = (datetime.datetime.today()-time1).tatal_seconds() #Calc dT
        delta = (datetime.datetime.today()-time1).total_seconds()
        

        
def task2():
    segment_count = 0
    while True:
        # RPM = 370*(0.001220703*Speed_Sensor.avg)  Motor_RPM
        # SPEED = 0.27 * RPM * 0.10472 * 3.6
        # >>> 0.0459735070110624 * Speed_Sensor.avg
        display1.set_nondigits([0b00000100, 0])
        display1.write_int(int(Speed_Sensor.avg*10))
        display2.set_nondigits([0b00000100, 0])
        display2.write_int(int(SUM.SOC*10))
        if(segment_count = 250):
            display1.clear_display()
            display2.clear_display()
            segment_count = 0
        segment_count = segment_count + 1

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
        global Data_list
        
        while True:
            lock.acquire()
            seperate_float()
            self.write_payload(Data_list) # Send INF
            self.set_mode(MODE.TX)
            time.sleep(2)
            self.reset_ptr_rx()
            #print(Data_list)
            
            lock.release()

lora = mylora(verbose=False)
lora.set_pa_config(pa_select=1, max_power=21, output_power=15)
lora.set_bw(BW.BW125)
lora.set_coding_rate(CODING_RATE.CR4_8)
lora.set_spreading_factor(12)
lora.set_rx_crc(True)
lora.set_low_data_rate_optim(True)

assert(lora.get_agc_auto_on() == 1)

thread_sensor_reading = threading.Thread(target = task1)
thread_display = threading.Thread(target = task2)

try:
    print("START")
    
    current1 = Data(5,50)
    current2 = Data(6,50)
    current3 = Data(7,50)
    current4 = Data(4,50)
    voltage1 = Data(0,200)
    Speed_Sensor = Data(1,149.646)  #Speed_Value = 5 * 294 * 0.1018 = 149.646
    SUM = SUMSUM(SOC_DATA)
    

    current1.sensor_init()
    current2.sensor_init()
    current3.sensor_init()
    current4.sersor_init()
    voltage1.sensor_init()
    Speed_Sensor.sensor_init()

    thread_sensor_reading.start()
    thread_display.start()
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
 
