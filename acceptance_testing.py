#import RPi.GPIO as GPIO
import serial
import time


def comms(port, baud, timeout):             # Serial Communication with VAL364
    try:                                        
        ser=serial.Serial(port, baud, timeout=timeout)
        print("Connected")
        return(ser)

    except:
        print("Connection Error")
        #soft_pwm.stop()
        #GPIO.cleanup()


class ccTalk_read():                        # Checking slave data
    '''
    This class takes recieved data from a slave device,
    cleans the data into a readable decimal format,
    and after checking the validity of the message,
    will return the recived message information.
    '''
    def __init__(self, msg):                    # Initialization
        self.msg = msg                              # This data variable should always be a byte array.
        self.decimal = self.hex_convert()           # Class will always convert byte array to hexidecimal to decimal.   


    def msg_check(self):                        # Checks the converted decimal array's CRC values                                   
        self.address = self.decimal[0]  
        self.header = self.decimal[3]

        if self.decimal[1] == 0:                    # Checks if message has data to convert
            self.length = 0
            self.data = None
        else:
            self.length = self.decimal[1]
            self.data = self.decimal[4 : 4 + self.decimal[1]]     # data starts at the 4th byte and count from 4 + length of data.

                                                    # Extracted message parsed to ccTalk_msg.message() method 
                                                    # to generate a calculated message
        self.message = ccTalk_msg(self.address, self.length, self.header, self.data).message()

        if self.message == self.decimal:            # Compares extracted message(.decimal) to calculated message(.message)
            return(self.message)
        else:                                       # If comparision fails, the extracted message is wrong 
            print('CRC check failed')
            return(1)


    def hex_convert(self):                      # Conversion from byte array to hexidecimal list
        bite_array = list(self.msg.hex())   
    
        hex_array = []                              # Since the hexidecimal array is individual characters within a list
        array_count = 0                             # they must be appended into pairs of hexidecimal bytes
        loop_count = int(len(bite_array)/2) 
    
        for bit in range(loop_count):               # This loop takes 2 bytes of the hexidecimal list and appends them into hex_array
            hex_array.append(bite_array[array_count] + bite_array[array_count+1])
            array_count += 2
    
        dec_array = []                              # Conversion from hexidecimal array to decimal array
        array_count = 0
        loop_count = int(len(hex_array))

        for bit in range(loop_count):               # This loop takes the appended bytes from hex_array and applies decimal conversion
            dec_array.append(int(hex_array[array_count], 16))
            array_count += 1

        return(dec_array)                           # Return the decimal converted byte array


    def msg_translation(self):                  # Isolates ccTalk message parameters for data manipulation                                          
        self.address = self.decimal[0]              # Address destination
        self.length = self.decimal[1]               # Data length
        self.header = self.decimal[3]               # Device recipient
        self.data = self.decimal[4 : 4 + self.decimal[1]]     # data starts at the 4th byte and count from 4 + length of data.
        return(self.address, self.length, self.header, self.data)


    def slave_msg_label(self):                  # Slave message identification
        header_types = {                            # Dictionary of responses                  
        '[1, 0, 48, 0, 55]' : "ACK",
        '[1, 0, 149, 5, 103]' : "NAK",
        '[1, 0, 246, 6, 87]' : "BUSY"
        }
        string_convert = str(self.decimal)          # String conversion for easier terminal readability
        return(header_types.get(string_convert))


class ccTalk_msg():                         # ccTalk message generator
    '''
    This class generates a ccTalk message in the form a decimal array,
    the input data required is only the: address, length of data, header and data,
    CRC is calculated within itself and will return the result in an array
    '''
    def __init__(self, address, length, header, data):  # Initialization
        self.address = address
        self.length = length                        # Class global variables
        self.header = header
        
        if data != None:                            # Checking if data is present
            if self.length > 1:                     # If there is more than 1 byte of data in the message
                self.data = []
                for bite in data:
                    self.data.append(bite)          # Individually append each data byte into a single message list
        
            else:
                self.data = data                    # Append a single byte of data into the message list
        
        else:
            self.data = data                        # Append datatype "None" into the global data variable
        
                   
    def message(self):                          # Message generation
        result = [] 
        result.append(self.address)                 # Append global class variables into a message list array
        result.append(self.length)
        
        crc_hex = self.crc_calculation()            # CRC calculation from global variables
        self.crc_lsb = int(crc_hex[1], 16)
        self.crc_msb = int(crc_hex[0], 16)
        
        result.append(self.crc_lsb)                 # Each 'result.append' is in order of the ccTalk protocol
        result.append(self.header)

        if self.data != None:                       # Checks if data is to be inserted in the message
            if self.length > 1:                     # If data is great than 1 byte, it needs to be extraced out of the
                for bite in self.data:              # generated data array and appended into message list array
                    result.append(bite)
        
            elif self.length == 1:                  # If data is a single byte append the single byte
                result.append(self.data[0])         # into the message list array
                                                    # If no data exists(None), do not append
        result.append(self.crc_msb)
            
        return result                               # Return generated message


    def crc_calculation(self):                  # 16-bit CRC calculation
        if self.data == None:                       # CRC calculation is dependant on if data is present or not
            array = [self.address, self.length, self.header]
        else:                                       # If data is present append the lists
            array = [self.address, self.length, self.header] + self.data


        crc = 0x0000                                # Initial CRC register
        poly = 0x1021                               # CRC polynomial

        ##############*CRC Algorithm*######################## Black box of unknown
        for bite in array:                                  #
            crc ^= (bite << 8) & 0xffff                     #
            for j in range(0, 8):                           #
                if crc & 0x8000:                            #
                    crc = ((crc << 1) ^ poly) & 0xffff      #
                else:                                       #
                    crc <<= 1                               #
                    crc &= 0xffff                           # 
        ##################################################### Result is in Decimal request Hex conversion        
        
        hex = '{0:x}'.format(crc)                   # I assume hexidecimal formatting      
        hex_convert = list(hex)                     # Place formatted data into a list
        
        if len(hex) == 3:
            hex_convert = ['0'] + hex_convert       # Add hidden zero value to front of hexidecimal 

        crc_array = [hex_convert[0] + hex_convert[1], hex_convert[2] + hex_convert[3]] # [LSB, MSB] string clean up
        
        return(crc_array)                           # Return calculated CRC values                                          


    def host_msg_label(self):                   # Host message identificaion
        header_types = {                            # Dictionary of responses
        255 : 'Factory set:up and test',
        254 : 'Simple poll',
        253 : 'Address poll',
        252 : 'Address clash',
        251 : 'Address change',
        250 : 'Address random',
        249 : 'Request polling priority',
        248 : 'Request status',
        247 : 'Request variable set',
        246 : 'Request manufacturer id',
        245 : 'Request equipment category id',
        244 : 'Request product code',
        243 : 'Request database version',
        242 : 'Request serial number',
        241 : 'Request software revision',
        240 : 'Test solenoids',
        239 : 'Operate motors',
        238 : 'Test output lines',
        237 : 'Read input lines',
        236 : 'Read opto states',
        235 : 'Read last credit or error code',
        234 : 'Issue guard code',
        233 : 'Latch output lines',
        232 : 'Perform self:check',
        231 : 'Modify inhibit status',
        230 : 'Request inhibit status',
        229 : 'Read buffered credit or error codes',
        228 : 'Modify master inhibit status',
        227 : 'Request master inhibit status',
        226 : 'Request insertion counter',
        225 : 'Request accept counter',
        224 : 'Dispense coins',
        223 : 'Dispense change',
        222 : 'Modify sorter override status',
        221 : 'Request sorter override status',
        220 : 'One:shot credit',
        219 : 'Enter new PIN number',
        218 : 'Enter PIN number',
        217 : 'Request payout high / low status',
        216 : 'Request data storage availability',
        215 : 'Read data block',
        214 : 'Write data block',
        213 : 'Request option flags',
        212 : 'Request coin position',
        211 : 'Power management control',
        210 : 'Modify sorter paths',
        209 : 'Request sorter paths',
        208 : 'Modify payout absolute count',
        207 : 'Request payout absolute count',
        206 : 'Empty payout',
        205 : 'Request audit information block',
        204 : 'Meter control',
        203 : 'Display control',
        202 : 'Teach mode control',
        201 : 'Request teach status',
        200 : 'Upload coin data',
        199 : 'Configuration to EEPROM',
        198 : 'Counters to EEPROM',
        197 : 'Calculate ROM checksum',
        196 : 'Request creation date',
        195 : 'Request last modification date',
        194 : 'Request reject counter',
        193 : 'Request fraud counter',
        192 : 'Request build code',
        191 : 'Keypad control',
        190 : 'Request payout status',
        189 : 'Modify default sorter path',
        188 : 'Request default sorter path',
        187 : 'Modify payout capacity',
        186 : 'Request payout capacity',
        185 : 'Modify coin id',
        184 : 'Request coin id',
        183 : 'Upload window data',
        182 : 'Download calibration info',
        181 : 'Modify security setting',
        180 : 'Request security setting',
        179 : 'Modify bank select',
        178 : 'Request bank select',
        177 : 'Handheld function',
        176 : 'Request alarm counter',
        175 : 'Modify payout float',
        174 : 'Request payout float',
        173 : 'Request thermistor reading',
        172 : 'Emergency stop',
        171 : 'Request hopper coin',
        170 : 'Request base year',
        169 : 'Request address mode',
        168 : 'Request hopper dispense count',
        167 : 'Dispense hopper coins',
        166 : 'Request hopper status',
        165 : 'Modify variable set',
        164 : 'Enable hopper',
        163 : 'Test hopper',
        162 : 'Modify inhibit and override registers',
        161 : 'Pump RNG',
        160 : 'Request cipher key',
        159 : 'Read buffered bill events',
        158 : 'Modify bill id',
        157 : 'Request bill id',
        156 : 'Request country scaling factor',
        155 : 'Request bill position',
        154 : 'Route bill',
        153 : 'Modify bill operating mode',
        152 : 'Request bill operating mode',
        151 : 'Test lamps',
        150 : 'Request individual accept counter',
        149 : 'Request individual error counter',
        148 : 'Read opto voltages',
        147 : 'Perform stacker cycle',
        146 : 'Operate bi:directional motors',
        145 : 'Request currency revision',
        144 : 'Upload bill tables',
        143 : 'Begin bill table upgrade',
        142 : 'Finish bill table upgrade',
        141 : 'Request firmware upgrade capability',
        140 : 'Upload firmware',
        139 : 'Begin firmware upgrade',
        138 : 'Finish firmware upgrade',
        137 : 'Switch encryption code',
        136 : 'Store encryption code',
        135 : 'Set accept limit',
        134 : 'Dispense hopper value',
        133 : 'Request hopper polling value',
        132 : 'Emergency stop value',
        131 : 'Request hopper coin value',
        130 : 'Request indexed hopper dispense count',
        129 : 'Read barcode data',
        128 : 'Request money in',
        127 : 'Request money out',
        126 : 'Clear money counters',
        125 : 'Pay money out',
        124 : 'Verify money out',
        123 : 'Request activity register',
        122 : 'Request error status',
        121 : 'Purge hopper',
        120 : 'Modify hopper balance',
        119 : 'Request hopper balance',
        118 : 'Modify cashbox value',
        117 : 'Request cashbox value',
        116 : 'Modify real time clock',
        115 : 'Request real time clock',
        114 : 'Request USB id',
        113 : 'Switch baud rate',
        112 : 'Read encrypted events',
        111 : 'Request encryption support',
        110 : 'Switch encryption key',
        109 : 'Request encrypted hopper status',
        108 : 'Request encrypted monetary id',
        107 : "Operate escrow",
        106 : 'Request escrow status',
        105 : 'Data stream',
        104 : 'Request service status',
        4 : 'Request comms revision',
        3 : 'Clear comms status variables',
        2 : 'Request comms status variables',
        1 : 'Reset device',
        0 : 'Reply'
        }
        return(header_types.get(self.header))


class ccTalk_write():                       # Master command label to decimal conversion
    '''
    This class generates ccTalk parameters from human commands.
    Commands are currently implimented within the cmd_msg_label() method.
    This class is implimented to make commands easier for the programmer.
    '''
    def __init__(self, cmd):                    # Initialization
        self.cmd = cmd

    def command(self):                          # Generate message parameters from command label              
        self.parameters = self.cmd_msg_label()
        
        if len(self.parameters) < 4:                # Check command message for the presence of data
            self.address = self.parameters[0]
            self.length = self.parameters[1]
            self.header = self.parameters[2]
            self.data = None                        
        else:
            self.address = self.parameters[0]
            self.length = self.parameters[1]
            self.header = self.parameters[2]
            self.data = self.parameters[3:]         # Creates a specific array for data, which is extracted in .message()
                                                    # Parses message parameters to generate ccTalk message
        host_reply = ccTalk_msg(self.address, self.length, self.header, self.data)
        host_msg = host_reply.message()
                                                    # Cross checks message label from generated message
        host_label = host_reply.host_msg_label() 
        val364.write(host_msg)                      # Sends generated message from ccTalk_msg() class
        #print("Sent message:", host_msg, host_label)

        slave_msg_head = val364.read(4)             # Read the first 4 bytes of data from slave
        try:
            slave_msg_tail = val364.read(slave_msg_head[1] + 1) # Reads the remain bytes of data including the MSB CRC
            slave_msg_raw = slave_msg_head + slave_msg_tail     # Combine the data arrays to get full ccTalk message
            slave_reply = ccTalk_read(slave_msg_raw)
            slave_msg = slave_reply.msg_check()  # Cross checks the CRC to insure correct message was received
            
            if slave_msg[1] == 0:
                slave_label = slave_reply.slave_msg_label()  # Cross checks message label from received message
                #print("Recieved message:", slave_msg, slave_label)

            else:
                slave_label = slave_reply.msg_translation() # Extracts data from recived message
                #print("Recieved data:", slave_label[3])
                #print("address =", slave_label[0], "/ data length =", slave_label[1], "/ header =", slave_label[2])
                return(slave_label[3])


        except:
            error_msg = ccTalk_read(slave_msg_head)
            error_label = error_msg.msg_translation()
            print('No response or an error occured;', 
                  "Address:", error_label[0], 
                  "/ Data length:", error_label[1],
                  "/ Header:", error_label [2],
                  "/ Recieved data:", error_label[3])           # If 'slave_msg'tail' fails,
                                                                # this means the device failed to respond
                                                                # most likely due to an error from the master message


    def cmd_msg_label(self):                    # Master message identification
        command_types = {
            'poll' : [55, 0, 254],                  # Currently set for CX only(55), backplane is 240, To be improved for device unification
            'self_check' : [55, 0, 232],
            'enable_coin' : [55, 2, 231, 255, 255],
            'read_credit' : [55, 0, 229],
            'enable_master' : [241, 1, 228, 1],         
            'abort_dispense' : [240, 0 ,96],            # Command to re-enable coin acceptance
            0 : [240, 6, 97, 1, 0, 0, 0, 0, 0],         # Dispense A
            1 : [240, 6, 97, 0, 1, 0, 0, 0, 0],         # Dispense B
            2 : [240, 6, 97, 0, 0, 1, 0, 0, 0],         # Dispense C
            3 : [240, 6, 97, 0, 0, 0, 1, 0, 0],         # Dispense D
            4 : [240, 6, 97, 0, 0, 0, 0, 1, 0]          # Dispense E
            
        }
        return(command_types.get(self.cmd))


if __name__ == "__main__":
    # Serial Communication Parameters
    windows_com_port = "COM3"
    linux_com_port = "/dev/ttyUSB0"
    baud_rate = 57600
    timeout = 2
    
    # RPi setup
    '''
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(40, GPIO.OUT)
    GPIO.setup(18, GPIO.OUT)
    soft_pwm = GPIO.PWM(40, 1)
    soft_pwm.start(0)
    motor_speed = 0
    '''

    # CX Setup
    event_count = 0
    tube_position = 0
    loop_count = 0

    # Main code
    val364 = comms(windows_com_port, baud_rate, timeout)
    
    ccTalk_write('poll').command()
    ccTalk_write('self_check').command()
    ccTalk_write('enable_coin').command()
    # ccTalk_write('enable_master').command()
    read_coin = ccTalk_write('read_credit')
    
    #soft_pwm.ChangeDutyCycle(motor_speed)

    while True: 
        event_data = read_coin.command()
        #print(event_data)
        time.sleep(0.3)
        

        
        if event_count != event_data[0]:
            event_count = event_data[0]
            
            if event_data[1] == 0 and event_data[2] == 1:
                continue
            
            else:
                ccTalk_write(tube_position).command()
                time.sleep(0.5)
            
                ccTalk_write('abort_dispense').command()
                print("Event count:", event_count, "Tube position:", tube_position)
            
                if tube_position == 4:
                    tube_position = 0
                else:
                    tube_position += 1
            

