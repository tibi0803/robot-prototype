from machine import Pin, ADC, UART
import utime

sensor1 = ADC(Pin(27))
sensor1.atten(ADC.ATTN_11DB)
sensor2 = ADC(Pin(26))
sensor2.atten(ADC.ATTN_11DB)
sensor3 = ADC(Pin(25))
sensor3.atten(ADC.ATTN_11DB)
sensor4 = ADC(Pin(33))
sensor4.atten(ADC.ATTN_11DB)
sensor5 = ADC(Pin(32))
sensor5.atten(ADC.ATTN_11DB)

while True:
    s1value = sensor1.read()
    s2value = sensor2.read()
    s3value = sensor3.read()
    s4value = sensor4.read()
    s5value = sensor5.read()

    print(s1value,s2value,s3value,s4value,s5value)
    if s3value < 2000:
        #keep going as lower values mean black
        print('stay')
    elif s1value < 3500 or s2value < 1600:
        print('turn left')
    elif s5value < 3500 or s4value < 1600:
        print('turn more right')
    
utime.sleep(1)