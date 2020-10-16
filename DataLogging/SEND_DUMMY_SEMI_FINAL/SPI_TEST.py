import serial, spidev, time

 
spi_sensor = spidev.SpiDev()
spi_sensor.open(0,1)
spi_sensor.max_speed_hz = 300000

curve1 = 1
curve2 = 1

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
        natural_num_list = [0,0,0,0,0,0,0,0,0,0,0] #list[10] is value of temperature
        
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


current1 = filter_member(7,curve1,curve2)

current1.sensor_init()


while True:
    current1.sensor_reading()
    SPEED = 1 * current1.avg  * 0.045973507
    print(SPEED)
