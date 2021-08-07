from machine import Pin, I2C
import time, math, network, urequests

class Sensor:
    # Sensor Define Value
    AM2320_I2C_ADDR                         = 0x5C
    AM2320_I2C_WAKEUP_CMD                   = [0x00]
    AM2320_I2C_READ_CMD                     = [0x03,0x00,0x04]
    BH1750_I2C_ADDR                         = 0x23
    BH1750_I2C_ONE_TIME_HIGH_RES_MODE_CMD   = [0x20]
    SGP30_I2C_ADDR                          = 0x58
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
    
    def __sgp43_get_humidity_compensation_value(self,humidity,temperature):
        e_sat           = 6.11 * math.pow(10.0, (7.5 * temperature / (237.7 + temperature)))
        vapor_pressure  = (humidity * e_sat) / 100
        abs_humidity    = 1000 * vapor_pressure * 100 / ((temperature + 273) * 461.5)
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
            
    def __init__(self, pin_scl, pin_sda):
        self.__temperature = -100
        self.__humidity    = -100
        self.__lumen       = -100
        self.__co2         = 0
        self.__tvoc        = 0
        self.__gas_flag    = False
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
        if self.__gas_flag == False:
            res_bool = self.__sgp30_init_air_quality()
            if res_bool == False:
                return False, 0, 0
            time.sleep_ms(12)
            #Set Measure Air Quality Command
            try:
                self.__i2c_dev.writeto(self.SGP30_I2C_ADDR, bytes(self.SGP30_I2C_MEASURE_AIR_QUALITY_CMD))
            except OSError:
                print('SGP30 Measure Air Quality CMD No ACK')
                return False, 0, 0           
            time.sleep_ms(12)
        #Set Humidity Compensation Command
        if humidity != None and temperature != None:
            set_humidity  = self.__sgp43_get_humidity_compensation_value(humidity,temperature)
            data          = bytearray(5)
            data[0:2]     = bytes(self.SGP30_I2C_SET_HUMIDITY_CMD)
            data[2:4]     = set_humidity.to_bytes(2,'big')
            res_bool, crc = self.__sgp30_crc(False,data[3:5]) 
            data[4]       = crc
            print('array={},v1={:d},v2={:d}'.format(data,set_humidity,crc))
            try:
                self.__i2c_dev.writeto(self.SGP30_I2C_ADDR, data)
            except OSError:
                print('SGP30 Set Humidity Compensation CMD No ACK')
                return False, 0, 0
            print('Absolute Humidity={:d} g/m^3'.format(set_humidity))
            time.sleep_ms(100)
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
            #Set Gas Flag
            if self.__gas_flag == False and res_bool == True:
                self.__gas_flag = False
            return res_bool, co2, tvoc
        except OSError:
            print('SGP30 Read No ACK')
            return False, 0, 0 
   
    def get_result_value(self):
        #Get Temperature & Humidity
        res_bool, res_value1, res_value2 = self.read_temperature_humidity_value()
        if res_bool == True:
            self.__temperature = res_value1
            self.__humidity    = res_value2
        #Get Gas Value
        if self.__gas_flag == False:
            if res_bool == True:
                res_bool, res_value1, res_value2 = self.read_gas_value()
        else:
            res_bool, res_value1, res_value2 = self.read_gas_value()        
        if res_bool == True:
            self.__co2  = res_value1
            self.__tvoc = res_value2
        #Get Lumen Value
        res_bool, res_value1 = self.read_ambient_light_value()
        if res_bool == True:
            self.__lumen = res_value1     
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

def main():
    #Sensor Initial
    I2C_SCL_PIN = 23
    I2c_SDA_PIN = 21
    obj_sensor  = Sensor(I2C_SCL_PIN, I2c_SDA_PIN)
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
    
    while True:
        time.sleep(30)
        #Get Sensor Value
        result = obj_sensor.get_result_value()
        print(result)
#        if obj_cloud.upload_to_cloud(result) == False:
            #WiFi Connect
#            obj_cloud.connect_to_cloud()
                    
if __name__ == '__main__':
    main()