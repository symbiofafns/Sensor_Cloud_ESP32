from machine import Pin, I2C, SPI
from micropython import const
import time, math, network, urequests

class Sensor:
    # Sensor Define Value
    AM2320_I2C_ADDR                         = const(0x5C)
    AM2320_I2C_WAKEUP_CMD                   = [0x00]
    AM2320_I2C_READ_CMD                     = [0x03,0x00,0x04]
    BH1750_I2C_ADDR                         = const(0x23)
    BH1750_I2C_ONE_TIME_HIGH_RES_MODE_CMD   = [0x20]
    SGP30_I2C_ADDR                          = const(0x58)
    SGP30_I2C_GET_SERIAL_ID_CMD             =[0x36,0x82]
    SGP30_I2C_INIT_AIR_QUALITY_CMD          =[0x20,0x03]
    SGP30_I2C_MEASURE_AIR_QUALITY_CMD       =[0x20,0x08]
    SGP30_I2C_SET_HUMIDITY_CMD              =[0x20,0x61]

    def __am2320_check_crc(self,data):
        crc = 0xffff
        for index in range(len(data)):
            crc ^= data[index]
            for bit in range(8):
                result = crc & 0x0001
                if result > 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
                crc &= 0xffff
        if crc == 0:
            return True
        return False
    
    def __am2320_get_value(self,data):
        hum          = int.from_bytes(data[2:4],"Big",False)
        hum         /= 10.0
        temp         = int.from_bytes(data[4:6],"Big",True)
        temp        /= 10.0
        return temp,hum
    
    def __bh1750_get_mtreg(self,data):
        res    = [0x00,0x00]
        res[0] = 0x40 | (data >> 5)
        res[1] = 0x60 | (data & 0x1f)
        return res

    def __sgp30_crc(self,check_enable,data):
        crc = 0xff
        for index in range(len(data)):
            crc ^= data[index]
            for bit in range(8):
                result = crc & 0x80
                if result > 0:
                    crc <<= 1
                    crc ^= 0x31
                else:
                    crc <<= 1
            crc &= 0xff
        res_bool = True
        if check_enable == True and crc != 0:
            res_bool = False
        return res_bool, crc
    
    def __sgp30_get_humidity_compensation_value(self,humidity,temperature):
        e_sat           = 6.11 * math.pow(10.0, (7.5 * temperature / (237.7 + temperature)))
        vapor_pressure  = (humidity * e_sat) / 100
        abs_humidity    = 1000 * vapor_pressure * 100 / ((temperature + 273) * 461.5)
        print('absHumidity={:6.2f} g/m^3'.format(abs_humidity))
        number          = math.floor(abs_humidity * 256.0)
        return number
    
    def __is_sgp30_detected(self):
        #Get Serial ID Command
        try:
            self.__i2c_dev.writeto(self.SGP30_I2C_ADDR, bytes(self.SGP30_I2C_GET_SERIAL_ID_CMD))
        except OSError:
            print('SGP30 Get Serial ID CMD No ACK')
            return False        
        time.sleep_ms(2)
        #Get Serial ID Data
        try:
            data        = self.__i2c_dev.readfrom(self.SGP30_I2C_ADDR, 9)
            res_bool    = True
            id_array    = bytearray(6)
            id_cnt      = 0
            for index in range(0,9,3):
                res  = self.__sgp30_crc(True, data[index:index+3])
                if res[0] == False:
                    res_bool = False                   
                    break
                else:
                    id_array[id_cnt:id_cnt+2] = data[index:index+2]
                    id_cnt += 2
            print('Serial ID={}'.format(id_array))
            return res_bool            
        except OSError:
            print('SGP30 Read No ACK')
            return False
    
    def __sgp30_init_air_quality(self):
        if self.__is_sgp30_detected() == True:
            time.sleep_ms(2)
            #Set Init Air Quality Command
            try:
                self.__i2c_dev.writeto(self.SGP30_I2C_ADDR, bytes(self.SGP30_I2C_INIT_AIR_QUALITY_CMD))
            except OSError:
                print('SGP30 Init Air Quality CMD No ACK')

    def __sgp30_set_humidity(self,compensation):
        data      = bytearray(5)
        data[0:2] = bytes(self.SGP30_I2C_SET_HUMIDITY_CMD)
        data[2:4] = compensation.to_bytes(2, 'big')
        res       = self.__sgp30_crc(False, data[2:4])
        data[4]   = res[1]
        try:
            self.__i2c_dev.writeto(self.SGP30_I2C_ADDR, data)
            return True
        except OSError:
            print('SGP30 Set Humidity Compensation CMD No ACK')
            return False
        
    def __init__(self, pin_scl=23, pin_sda=21):
        self.__am2320_start_time = time.ticks_ms()
        self.__bh1750_start_time = time.ticks_ms()
        self.__sgp30_start_time  = time.ticks_ms()
        self.initial_flag  = const(0x00)
        self.__temperature = 0
        self.__humidity    = 0
        self.__lumen       = 0
        self.__co2         = 0
        self.__tvoc        = 0
        self.__i2c_dev     = I2C(0, scl=Pin(pin_scl), sda=Pin(pin_sda), freq=100000)
    
    def read_temperature_humidity_value(self):
        #WakeUp
        try:
            self.__i2c_dev.writeto(self.AM2320_I2C_ADDR, bytes(self.AM2320_I2C_WAKEUP_CMD))
        except OSError:
            pass
            #print('AM2320 WakeUp')
        time.sleep_ms(2)
        #Set Read Register Data Command
        try:
            self.__i2c_dev.writeto(self.AM2320_I2C_ADDR, bytes(self.AM2320_I2C_READ_CMD))
        except OSError:
            print('AM2320 Write No ACK')
            return False, 0, 0

        time.sleep_ms(2)
        #Read Register Data
        try:
            data = self.__i2c_dev.readfrom(self.AM2320_I2C_ADDR, 8)
            if self.__am2320_check_crc(data) == True:
                temp,hum = self.__am2320_get_value(data)
                return True, temp, hum
        except OSError:
            print('AM2320 Read No ACK')
            return False, 0, 0

    def read_ambient_light_value(self,):
        #Measurement Command
        try:
            self.__i2c_dev.writeto(self.BH1750_I2C_ADDR, bytes(self.BH1750_I2C_ONE_TIME_HIGH_RES_MODE_CMD))
        except OSError:
            print('BH1750 Command1 No ACK')
            return False, 0
        #Set MT Register Value
        reg_data = bytes(self.__bh1750_get_mtreg(69))       
        #Change Measurement time - H
        try:
            self.__i2c_dev.writeto(self.BH1750_I2C_ADDR, reg_data[0:1])
        except OSError:
            print('BH1750 Set MT Register HByte No ACK')
            return False, 0
        #Change Measurement time - L
        try:
            self.__i2c_dev.writeto(self.BH1750_I2C_ADDR, reg_data[1:2])
        except OSError:
            print('BH1750 Set MT Register LByte No ACK')
            return False, 0
        #Measurement Command
        try:
            self.__i2c_dev.writeto(self.BH1750_I2C_ADDR, bytes(self.BH1750_I2C_ONE_TIME_HIGH_RES_MODE_CMD))
        except OSError:
            print('BH1750 Command2 No ACK')
            return False, 0 
        #Read Value
        try:
            data = self.__i2c_dev.readfrom(self.BH1750_I2C_ADDR, 2)
            return True, int.from_bytes(data,"Big",False)
        except OSError:
            print('BH1750 Red No ACK')

    def read_gas_value(self,humidity = None, temperature = None):
        #Gas Initial
        flag_value = self.initial_flag & 0x04 
        if flag_value == 0x00:
            res_bool = self.__sgp30_init_air_quality()
            if res_bool == False:
                return False, 0, 0
            time.sleep_ms(12)
        #Set Humidity Compensation Command
        if humidity != None and temperature != None:
            set_value = self.__sgp30_get_humidity_compensation_value(humidity,temperature)
            if self.__sgp30_set_humidity(set_value) == False:
                return False, 0, 0
            time.sleep_ms(1000)
        #Set Measure Air Quality Command
        try:
            self.__i2c_dev.writeto(self.SGP30_I2C_ADDR, bytes(self.SGP30_I2C_MEASURE_AIR_QUALITY_CMD))
        except OSError:
            print('SGP30 Measure Air Quality CMD No ACK')
            return False, 0, 0           
        time.sleep_ms(12)
        #Get Measure Air Quality Data
        try:
            data     = self.__i2c_dev.readfrom(self.SGP30_I2C_ADDR, 6)
            res_bool = True
            for index in range(0,6,3):
                res  = self.__sgp30_crc(True, data[index:index+3])
                if res[0] == False:
                    res_bool = False                   
                    break
            co2  = int.from_bytes(data[0:2],"Big",False)
            tvoc = int.from_bytes(data[3:5],"Big",False)
            return res_bool, co2, tvoc
        except OSError:
            print('SGP30 Read No ACK')
            return False, 0, 0 
   
    def get_result_value(self):
        #Get Temperature & Humidity
        interval_time = time.ticks_diff(time.ticks_ms(), self.__am2320_start_time)
        flag_value    = self.initial_flag & 0x01
        if flag_value == 0 or interval_time > 30000:
            res_bool, res_value1, res_value2 = self.read_temperature_humidity_value()
            if res_bool == True:
                self.__temperature = res_value1
                self.__humidity    = res_value2
                self.initial_flag |= 0x01
            time.sleep_ms(20)
            self.__am2320_start_time = time.ticks_ms()
        #Get Lumen Value
        interval_time = time.ticks_diff(time.ticks_ms(), self.__bh1750_start_time)
        flag_value    = self.initial_flag & 0x02
        if flag_value == 0 or interval_time > 30000:
            res_bool, res_value1 = self.read_ambient_light_value()
            if res_bool == True:
                self.__lumen       = res_value1
                self.initial_flag |= 0x02
            time.sleep_ms(20)
            self.__bh1750_start_time = time.ticks_ms()
        #Get Gas Value
        interval_time = time.ticks_diff(time.ticks_ms(), self.__sgp30_start_time)
        flag_value    = self.initial_flag & 0x04
        if flag_value == 0 or interval_time > 1000:
            if self.__co2 > 0:
                res_bool, res_value1, res_value2 = self.read_gas_value()
            else:
                if self.__temperature > 0 and self.__humidity > 0:
                    res_bool, res_value1, res_value2 = self.read_gas_value(self.__humidity,\
                                                                           self.__temperature)
            if res_bool == True:
                self.__co2         = res_value1
                self.__tvoc        = res_value2
                self.initial_flag |= 0x04
            self.__sgp30_start_time = time.ticks_ms()
        #Set Output string
        result=[]
        if self.__co2 > 0:
            #Set Output string        
            string  = '{:4.1f}/{:4.1f}/{:d}/{:d}/{:d}'.format(self.__temperature, \
                                                              self.__humidity,    \
                                                              self.__lumen,       \
                                                              self.__co2,         \
                                                              self.__tvoc)
            result  = string.split('/')
        return result

class ThingspeakCloud:
    def __init__(self, essid, password, api_key):
        self.__essid    = essid
        self.__password = password
        self.__api_key  = api_key
        self.__wlan     = network.WLAN(network.STA_IF)
    
    def __wifi_connect(self):
        try:
            self.__wlan.active(False)
            self.__wlan.active(True)
            self.__wlan.connect(self.__essid,self.__password)
            print('Start to connect WiFi')
            tick_start = time.ticks_ms()
            tick_sec   = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), tick_start) < 60000:
                if self.__wlan.isconnected():
                    break
                if time.ticks_diff(time.ticks_ms(), tick_sec) >= 1000:
                    sec      = time.ticks_diff(time.ticks_ms(), tick_start) / 1000
                    tick_sec = time.ticks_ms()
                    print('Try to connect WiFi in {:<2.0f}s'.format(sec))
            res_connect = self.__wlan.isconnected()
            if res_connect == True:
                print('WiFi connection OK!')
                print('Network Config=',self.__wlan.ifconfig())                
            else:
                print('WiFi connection Error')
            return res_connect         
        except Exception as e: 
            print(e)
            return False
    
    def connect_to_cloud(self):
        condition = True
        while condition == True:
            if self.__wifi_connect() == True:
                condition = False
            else:
                print('Try Connect Again!!')
                time.sleep(10) 

    def upload_to_cloud(self, string_value):
        #Set Thingspeak URL
        thing_url  = 'http://api.thingspeak.com/update?api_key='
        thing_url += self.__api_key
        #Set Thingspeak Value
        for i in range(len(string_value)):
            str = '&field{:d}={:s}'.format(i+1,string_value[i])
            thing_url += str
        #print('URL={:s}'.format(thing_url))        
        res_bool = False
        try:
            http_response = urequests.get(thing_url)
            res_bool      = True
            print('code={},content=  {}'.format(http_response.status_code,http_response.text))
        except Exception as e: 
            print(e)
        http_response.close()
        return res_bool

class SSD1306:
    SSD1306_SET_CONTRAST        = const(0x81)
    SSD1306_SET_ENTIRE_ON       = const(0xA4)
    SSD1306_SET_NORM_INV        = const(0xA6)
    SSD1306_SET_DISP            = const(0xAE)
    SSD1306_SET_MEM_ADDR        = const(0x20)
    SSD1306_SET_COL_ADDR        = const(0x21)
    SSD1306_SET_PAGE_ADDR       = const(0x22)
    SSD1306_SET_DISP_START_LINE = const(0x40)
    SSD1306_SET_SEG_REMAP       = const(0xA0)
    SSD1306_SET_MUX_RATIO       = const(0xA8)
    SSD1306_SET_IREF_SELECT     = const(0xAD)
    SSD1306_SET_COM_OUT_DIR     = const(0xC0)
    SSD1306_SET_DISP_OFFSET     = const(0xD3)
    SSD1306_SET_COM_PIN_CFG     = const(0xDA)
    SSD1306_SET_DISP_CLK_DIV    = const(0xD5)
    SSD1306_SET_PRECHARGE       = const(0xD9)
    SSD1306_SET_VCOM_DESEL      = const(0xDB)
    SSD1306_SET_CHARGE_PUMP     = const(0x8D)
    SSD1306_WIDTH               = const(128)
    SSD1306_HEIGHT              = const(64)

    def __write_cmd(self, cmd):
        self.__gpio_cs.value(1)
        self.__gpio_dc.value(0)
        self.__gpio_cs.value(0)
        self.__spi_dev.write(bytearray([cmd]))
        self.__gpio_cs.value(1)

    def __write_data(self, buf):
        self.__gpio_cs.value(1)
        self.__gpio_dc.value(1)
        self.__gpio_cs.value(0)
        self.__spi_dev.write(buf)
        self.__gpio_cs.value(1)
    
    def __initial(self):
        self.__write_cmd(self.SSD1306_SET_DISP)
        # the suggested ratio 0x80
        self.__write_cmd(self.SSD1306_SET_DISP_CLK_DIV)
        self.__write_cmd(0x80)
        self.__write_cmd(self.SSD1306_SET_MUX_RATIO)
        self.__write_cmd(self.SSD1306_HEIGHT - 1)
        # no offset
        self.__write_cmd(self.SSD1306_SET_DISP_OFFSET)
        self.__write_cmd(0x00)
        # line #0
        self.__write_cmd(self.SSD1306_SET_DISP_START_LINE | 0x00)
        # display voltage from 3.3V
        self.__write_cmd(self.SSD1306_SET_CHARGE_PUMP)
        self.__write_cmd(0x14)
        # act like ks0108
        self.__write_cmd(self.SSD1306_SET_MEM_ADDR)
        self.__write_cmd(0x00)
        self.__write_cmd(self.SSD1306_SET_SEG_REMAP | 0x01)
        self.__write_cmd(self.SSD1306_SET_COM_OUT_DIR | 0x08)
        self.__write_cmd(self.SSD1306_SET_COM_PIN_CFG)
        self.__write_cmd(0x12)
        self.__write_cmd(self.SSD1306_SET_CONTRAST)
        self.__write_cmd(0xCF)
        self.__write_cmd(self.SSD1306_SET_PRECHARGE)
        self.__write_cmd(0xF1)
        self.__write_cmd(self.SSD1306_SET_VCOM_DESEL)
        self.__write_cmd(0x40)
        self.__write_cmd(self.SSD1306_SET_ENTIRE_ON)
        self.__write_cmd(self.SSD1306_SET_NORM_INV)
        # Stop scroll
        self.__write_cmd(0x2E)
        self.__write_cmd(self.SSD1306_SET_DISP | 0x01)

    def __init__(self, pin_sck=18, pin_sda=22, pin_res=17, pin_dc=16, pin_cs=5):
        self.__spi_dev  = SPI(2,                \
                              baudrate=10000000,\
                              polarity=0,       \
                              phase=0,          \
                              bits=8,           \
                              firstbit=0,       \
                              sck=Pin(pin_sck), mosi=Pin(pin_sda), miso=Pin(19))
        self.__gpio_res = Pin(pin_res, Pin.OUT)
        self.__gpio_res.value(0)
        self.__gpio_dc  = Pin(pin_dc,  Pin.OUT)
        self.__gpio_dc.value(0)
        self.__gpio_cs  = Pin(pin_cs,  Pin.OUT)
        self.__gpio_cs.value(1)
        #Reset Device
        self.__gpio_res.value(1)
        time.sleep_ms(1)
        self.__gpio_res.value(0)
        time.sleep_ms(10)
        self.__gpio_res.value(1)
        #Initial
        self.__initial()
        print('Driver Initial OK!')
    
    def contrast(self, value):
        self.__write_cmd(self.SSD1306_SET_CONTRAST)
        self.__write_cmd(value)

    def invert(self, value):
        self.__write_cmd(self.SSD1306_SET_NORM_INV | (value & 1))

    def rotate(self, value):
        self.__write_cmd(self.SSD1306_SET_COM_OUT_DIR | ((value & 1) << 3))
        self.__write_cmd(self.SSD1306_SET_SEG_REMAP   | (value & 1))

    def draw(self, buf):
        x0      = 0
        x1      = self.SSD1306_WIDTH - 1
        pages   = self.SSD1306_HEIGHT / 8
        self.__write_cmd(self.SSD1306_SET_COL_ADDR)
        self.__write_cmd(x0)
        self.__write_cmd(x1)
        self.__write_cmd(self.SSD1306_SET_PAGE_ADDR)
        self.__write_cmd(0)
        self.__write_cmd(pages)
        self.__write_data(self.buf)

def main():

    
    
    while True:
        time.sleep(1)

    #Sensor Initial
    obj_sensor  = Sensor()
    #Sensor Connect
    result = obj_sensor.get_result_value()
    print(result)
    
    #WIFI Initial
    wifi_essid    = 'WiFi_Linag'
    wifi_password = 'Chen_7321155'
    cloud_api_key = 'UOMGVLT0MLZG5WZS'
    obj_cloud     = ThingspeakCloud(wifi_essid, wifi_password,cloud_api_key)    
    #WiFi Connect
    #obj_cloud.connect_to_cloud()
    
    start_time = time.ticks_ms()
    while True:
        #Get Sensor Value
        result = obj_sensor.get_result_value()
        if time.ticks_diff(time.ticks_ms(), start_time) >= 30000:
            start_time = time.ticks_ms()
            print(result)
#       if obj_cloud.upload_to_cloud(result) == False:
            #WiFi Connect
#           obj_cloud.connect_to_cloud()
                    
if __name__ == '__main__':
    main()