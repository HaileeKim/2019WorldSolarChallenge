 
import seven_segment_display
import seven_segment_i2c


segment_address1 = 0x71
segment_address2 = 0x42
 
bus1 = seven_segment_i2c.SevenSegmentI2c(1,segment_address1)
bus2 = seven_segment_i2c.SevenSegmentI2c(1,segment_address2)
 
display1 = seven_segment_display.SevenSegmentDisplay(bus1)
display1.clear_display()
 
display2 = seven_segment_display.SevenSegmentDisplay(bus2)
display2.clear_display()
 
while True:                   
    display1.set_nondigits([0b00000100, 0])
    display1.write_int(1234)
    display2.set_nondigits([0b00000100, 0])
    display2.write_int(7894)#voltage1.avg*10

