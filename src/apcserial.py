'''
This is a test application to communicate with an APC SMC1000i UPS over the serial port.
The protocol (Microlink) does not seem to be documented, but references to some parameters
and fields can be found in various APC datasheets/manuals.
For more details on the development, visit https://sites.google.com/site/klaasdc/apc-smartups-decode
------
Copyright (C) 2019 Klaas De Craemer

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

@author: klaasdc
'''
import sys
import threading
import time
from enum import Enum, auto
from checksum.fletcherNbit import Fletcher
import serial
import datetime

APC_RCV_TIMEOUT = 0.25
APC_RCV_SIZE = 19
# APC_RCV_TIMEOUT = 0.5

APC_CMD_INIT = [0xF7, 0xFD]
APC_CMD_BACK = [0xF7]
APC_CMD_RESET = [0xFD]
APC_CMD_NEXT = [0xFE]

class CommState(Enum):
    INIT = auto()
    INIT_RESET = auto()
    MODE0 = auto()
    MODE1 = auto()

class ApcComm(threading.Thread):
    
    def __init__(self, serial_port):
        super(ApcComm, self).__init__()
        
        self.s = serial_port
        
        self.state = CommState.INIT
        self.prev_state = CommState.INIT
        self.next_apc_msg = APC_CMD_NEXT#Next message to be sent to the UPS, for internal use
        self.running = True
        self.daemon = True
        
        self.ups_state = {"comm_state": "offline"}
    
    def send_apc_msg(self, raw_msg):
        '''
        Schedule the next message to be sent.
        Blocks until the message is succesfully sent.
        '''
        if self.state is not CommState.MODE1:
            return False
        self.next_apc_msg = raw_msg
        while self.next_apc_msg is not APC_CMD_NEXT:
            time.sleep(APC_RCV_TIMEOUT)
        return True
    
    def run(self):
        while (self.running):
            if self.state == CommState.INIT:
                '''
                Initialize the communication
                '''
                self.s.write(APC_CMD_INIT)
                rcv_data = self.receive_msg()
                
                if not self.handle_apc_msg(rcv_data):
                    self.state = CommState.INIT_RESET
                else:
                    self.state = CommState.MODE0
            
            elif self.state == CommState.INIT_RESET:
                '''
                Reset the UPS communication
                '''
                time.sleep(1)
                self.s.write(APC_CMD_RESET)
                rcv_data = self.receive_msg()
                
                if not self.handle_apc_msg(rcv_data):
                    self.state = CommState.INIT
                else:
                    self.state = CommState.MODE0
                    
            elif self.state == CommState.MODE0:
                '''
                Normal communication flow with UPS according to MODE0
                '''
                self.s.write(self.next_apc_msg)
                rcv_data = self.receive_msg()
                
                if not self.handle_apc_msg(rcv_data):
                    self.state = CommState.INIT
            
            elif self.state == CommState.MODE1:
                '''
                Normal communication flow with UPS according to MODE1
                '''
                self.s.write(self.next_apc_msg)
                rcv_data = self.receive_msg()
                
                if self.next_apc_msg is not None:
                    self.next_apc_msg = APC_CMD_NEXT
                
                if not self.handle_apc_msg(rcv_data):
                    self.state = CommState.INIT
        
            if self.state is not self.prev_state:
                if self.state == CommState.MODE0 or self.state == CommState.MODE1:
                    self.ups_state['comm_state'] = 'online'
                else:
                    self.ups_state['comm_state'] = 'offline'
#                 print(self.state)
            self.prev_state = self.state
                
    def receive_msg(self):
        data = []
        curTime = time.time()
        while (time.time() - curTime) < APC_RCV_TIMEOUT:
            data += self.s.read(APC_RCV_SIZE)
        return data
    
    def verify_msg_checksum(self, raw_msg):
        msg_chksum = int.from_bytes(raw_msg[-2:], byteorder='big', signed=False)
        
        f8 = Fletcher()
        f8.update(raw_msg[0:-2])
        checksum = (f8.cb0 << 8) + f8.cb1
        checksum_result = True if msg_chksum == checksum else False
        
        return checksum_result
    
    def calc_checksum(self, raw_msg):
        f8 = Fletcher()
        f8.update(raw_msg[0:15])
        checksum = (f8.cb0 << 8) + f8.cb1
        return checksum
        
    def create_msg_data(self, msg_id, offset, msg_data):
        '''
        Create a message to send to the UPS
        
        msg_id        Message ID
        offset        Byte offset where to write to in the UPS
        msg_data      Data to set as bytearray
        '''
        length = len(msg_data)
        raw_msg = bytearray([msg_id, offset, length]) + msg_data
        #Add checksum
        raw_msg += self.calc_checksum(raw_msg).to_bytes(2, byteorder='big', signed=False)
        return raw_msg
    
    def handle_apc_msg(self, raw_msg):
        if raw_msg is not None and len(raw_msg) > 0:
            #Convert to bytes
            raw_msg = bytearray(raw_msg)
            #Extract message parts
            msg_id = raw_msg[0]
            msg_data = raw_msg[1:-2]     
            
#             print("Received message ID " + hex(msg_id))
            
            if not self.verify_msg_checksum(raw_msg):
                self.next_apc_msg = APC_CMD_BACK
                return True
            
            #Identify data
            if msg_id == 0x00:
                self.ups_state['protocol_version'] = msg_data[0]
                self.ups_state['msg_size'] = msg_data[1]
                self.ups_state['num_ids'] = msg_data[2]
                self.ups_state['series_id'] = int.from_bytes(bytes=msg_data[3:5], byteorder='big', signed=False)
                self.ups_state['series_id_raw'] = msg_data[3:5]
                self.ups_state['series_data_version'] = msg_data[5]
                self.ups_state['unknown_3'] = msg_data[6]
                self.ups_state['unknown_4'] = msg_data[7]
                
                self.ups_state['header_raw'] = msg_data[0:8]#Needed for challenge calculation
                
            elif msg_id == 0x40:
                self.ups_state['serial_nb'] = msg_data[0:14].decode()
                self.ups_state['serial_nb_raw'] = msg_data[0:14]#0x33 0x53 0x31 0x36 0x30 0x37 0x58 0x30 0x30 0x35 0x38 0x38 0x20 0x20
                self.ups_state['production_date'] = self.convert_to_datetime(int.from_bytes(bytes=msg_data[14:16], byteorder='big', signed=False))
                
            elif msg_id == 0x41:
                #First 16 bytes of ups name
                self.ups_state['ups_type'] = msg_data.decode()
                
            elif msg_id == 0x42:
                #Last 16 bytes of ups name
                self.ups_state['ups_type'] += msg_data.decode()
                
            elif msg_id == 0x43:
                #First 16 bytes of SKU
                self.ups_state['ups_sku'] = msg_data.decode()
                
            elif msg_id == 0x44:
                #Last 4 bytes of SKU
                self.ups_state['ups_sku'] += msg_data[0:4].decode()
                
            elif msg_id == 0x45:
                self.ups_state['fw_version_1'] = msg_data[0:8].decode()
                self.ups_state['fw_version_2'] = msg_data[8:].decode()
                
            elif msg_id == 0x46:
                self.ups_state['fw_version_3'] = msg_data[0:8].decode()
                self.ups_state['fw_version_4'] = msg_data[8:].decode()
            
            elif msg_id == 0x47:
                self.ups_state['battery_install_date'] = self.convert_to_datetime(int.from_bytes(bytes=msg_data[0:2], byteorder='big', signed=False))
                self.ups_state['battery_lifetime'] = int.from_bytes(bytes=msg_data[2:4], byteorder='big', signed=False)#Battery expected lifetime in number of days
                self.ups_state['battery_near_eol_alarm_notification'] = int.from_bytes(bytes=msg_data[4:6], byteorder='big', signed=False)#Alarm triggers this number of days before estimated battery replacement. Default: 183 days
                self.ups_state['battery_near_eol_alarm_reminder'] = int.from_bytes(bytes=msg_data[6:8], byteorder='big', signed=False)#Near-EOL alarm is repeated every x days. Default: 14 days
                
            elif msg_id == 0x48:
                self.ups_state['battery_sku'] = msg_data.decode()
                
            elif msg_id == 0x49:
                self.ups_state['ups_name'] = msg_data.decode()
                
            elif msg_id == 0x4a:
                self.ups_state['allowed_operating_mode'] = int.from_bytes(bytes=msg_data[0:2], byteorder='big', signed=False)#Bitfield
                self.ups_state['power_quality_config'] = int.from_bytes(bytes=msg_data[2:4], byteorder='big', signed=False)#Bitfield
                
                battery_replacetest_interval_raw = int.from_bytes(bytes=msg_data[4:6], byteorder='big', signed=False)#Bitfield
                self.ups_state['battery_replacetest_interval_raw'] = battery_replacetest_interval_raw
                self.ups_state['battery_replacetest_interval'] = []
                if battery_replacetest_interval_raw & 1 == 1:
                    self.ups_state['battery_replacetest_interval'].append("DISABLED")#Testing is disabled
                if battery_replacetest_interval_raw & 2 == 2:
                    self.ups_state['battery_replacetest_interval'].append("STARTUP")#Testing is done only at every startup of UPS
                if battery_replacetest_interval_raw & 4 == 4:
                    self.ups_state['battery_replacetest_interval'].append("EACH 7 DAYS SINCE STARTUP")#Test every 7 days since startup
                if battery_replacetest_interval_raw & 8 == 8:
                    self.ups_state['battery_replacetest_interval'].append("EACH 14 DAYS SINCE STARTUP")#Test every 14 days since startup
                if battery_replacetest_interval_raw & 16 == 16:
                    self.ups_state['battery_replacetest_interval'].append("EACH 7 DAYS SINCE LAST")#Test every 7 days since last test
                if battery_replacetest_interval_raw & 32 == 32:
                    self.ups_state['battery_replacetest_interval'].append("EACH 14 DAYS SINCE LAST")#Test every 14 days since last test
                
                self.ups_state['battery_replacement_due'] = self.convert_to_datetime(int.from_bytes(bytes=msg_data[6:8], byteorder='big', signed=False))
                self.ups_state['low_runtime_alarm_config'] = int.from_bytes(bytes=msg_data[8:10], byteorder='big', signed=False)#Amount of seconds remaining when low-runtime-alarm will trigger
                self.ups_state['voltage_accept_max'] = int.from_bytes(bytes=msg_data[10:12], byteorder='big', signed=False)
                self.ups_state['voltage_accept_min'] = int.from_bytes(bytes=msg_data[12:14], byteorder='big', signed=False)
                
                voltage_sens = int.from_bytes(bytes=msg_data[15:16], byteorder='big', signed=False)
                self.ups_state['voltage_sensitivity_raw'] = voltage_sens
                if voltage_sens == 1:
                    self.ups_state['voltage_sensitivity'] = "HIGH"
                elif voltage_sens == 2:
                    self.ups_state['voltage_sensitivity'] = "MEDIUM"
                elif voltage_sens == 4:
                    self.ups_state['voltage_sensitivity'] = "LOW"
                
            elif msg_id == 0x4b:
                self.ups_state['apparent_power_rating'] = int.from_bytes(bytes=msg_data[0:2], byteorder='big', signed=False)
                self.ups_state['real_power_rating'] = int.from_bytes(bytes=msg_data[2:4], byteorder='big', signed=False)
                
                voltage_config = int.from_bytes(bytes=msg_data[4:6], byteorder='big', signed=False)#Input voltage setting
                self.ups_state['voltage_config_raw'] = voltage_config
                if voltage_config == 1:
                    self.ups_state['voltage_config'] = 100
                elif voltage_config == 2:
                    self.ups_state['voltage_config'] = 120
                elif voltage_config == 4:
                    self.ups_state['voltage_config'] = 200
                elif voltage_config == 8:
                    self.ups_state['voltage_config'] = 208
                elif voltage_config == 16:
                    self.ups_state['voltage_config'] = 220
                elif voltage_config == 32:
                    self.ups_state['voltage_config'] = 230
                elif voltage_config == 64:
                    self.ups_state['voltage_config'] = 240
                elif voltage_config == 2048:
                    self.ups_state['voltage_config'] = 115
                
            elif msg_id == 0x4c:
                self.ups_state['power_on_delay'] = int.from_bytes(bytes=msg_data[0:2], byteorder='big', signed=False)#TurnOnCountdownSetting: Amount of seconds between outlet ON command and switching on
                self.ups_state['power_off_delay'] = int.from_bytes(bytes=msg_data[2:4], byteorder='big', signed=False)#TurnOffCountdownSetting: Amount of seconds between outlet OFF command and switching off
                self.ups_state['reboot_delay'] = int.from_bytes(bytes=msg_data[4:8], byteorder='big', signed=False)#StayOffCountdownSetting: Amount of seconds to stay off during reboot sequence
                self.ups_state['runtime_minimum_return'] = self.convert_from_bp(msg_data[8:10], 0, signed=False)#Minimum runtime to have before switching outlets back on after outage, in seconds
                
                load_shed_config_raw = int.from_bytes(bytes=msg_data[10:12], byteorder='big', signed=False)#Bitfield with the main outlet group (MOG) load shedding behaviour options. Not all options are necessarily supported.
                self.ups_state['loadshed_config_raw'] = load_shed_config_raw
                self.ups_state['loadshed_config'] = []
                if load_shed_config_raw & 1 == 1:
                    self.ups_state['loadshed_config'].append("USE_OFF_DELAY")
                    #UseOffDelay- Modifier: When set, the load shed conditions that have this as a valid modifier will use the TurnOffCountdownSetting to shut the outlet off.
                if load_shed_config_raw & 2 == 2:
                    self.ups_state['loadshed_config'].append("MANUAL_RESTART_REQUIRED")
                    #ManualRestartRequired - Modifier - When set, the load shed conditions that have this as a valid modifier will use a turn off command instead of shutdown. 
                    #This results in a manual intervention to restart the outlet.
                if load_shed_config_raw & 4 == 4:
                    self.ups_state['loadshed_config'].append("RESERVED_BIT")
                if load_shed_config_raw & 8 == 8:
                    self.ups_state['loadshed_config'].append("TIME_ON_BATTERY")
                    #TimeOnBattery: The outlet group will shed based on the LoadShedTimeOnBatterySetting usage. When operating on battery greater than this time, the outlet will turn off.
                    #The modifier bits UseOffDelay and ManualRestartRequired are valid with this bit
                if load_shed_config_raw & 16 == 16:
                    self.ups_state['loadshed_config'].append("RUNTIME_REMAINING")
                    #RunTimeRemaining: The outlet group will shed based on the LoadShedRuntimeRemainingSetting usage. When operating on battery and the runtime remaining is
                    #less than or equal to this value, the outlet will turn off. The modifier bits UseOffDelay and ManualRestartRequired are valid with this bit.
                if load_shed_config_raw & 16 == 16:
                    self.ups_state['loadshed_config'].append("ON_OVERLOAD")
                    #UPSOverload - When set, the outlet will turn off immediately (no off delay possible) when the UPS is in overload. The outlet will require a manual command
                    #to restart. Not applicable for the Main Outlet Group (MOG)
                
                self.ups_state['loadshed_runtime_remaining'] = self.convert_from_bp(msg_data[12:14], 0, signed=False)#Outlet switches off (load shedding) when runtime drops to this value, in second
                self.ups_state['loadshed_runtime_limit'] = self.convert_from_bp(msg_data[14:16], 0, signed=False)#Outlet switches off (load shedding) after maximum time on battery, in seconds

            elif msg_id == 0x4d:
                self.ups_state['outlet_name'] = msg_data.decode()
                
            elif msg_id == 0x4e:
#                 interaction_value = int.from_bytes(bytes=msg_data[0:2], byteorder='big', signed=False)
#                 self.ups_state['interaction_setting'] = interaction_value
#                 self.ups_state['interaction_config_raw'] = []
#                 if interaction_value & 
#                
                #Alarm ON/OFF = 0x0005 / 0x0006 (Bit 
                #LCD Read-only = 0x1000 / 0x0000 (Bit 16)
                
                self.ups_state['InterfaceDisable_BF'] = int.from_bytes(bytes=msg_data[4:6], byteorder='big', signed=False)
            
            elif msg_id == 0x5c:
                self.ups_state['CommunicationMethod_EN'] = int.from_bytes(bytes=msg_data[8:10], byteorder='big', signed=False)#No idea
            
            elif msg_id == 0x6c:
                battery_lifetime_status_raw = int.from_bytes(bytes=msg_data[6:8], byteorder='big', signed=False)#Another bitfield, 1=OK?
                self.ups_state['battery_lifetime_status_raw'] = battery_lifetime_status_raw
                self.ups_state['battery_lifetime_status'] = []
                if battery_lifetime_status_raw & 1 == 1:
                    self.ups_state['battery_lifetime_status'].append("OK")#Battery life still OK
                if battery_lifetime_status_raw & 2 == 2:
                    self.ups_state['battery_lifetime_status'].append("NEAR EOL")#Near end-of-life
                if battery_lifetime_status_raw & 4 == 4:
                    self.ups_state['battery_lifetime_status'].append("OVER EOL")#Over end-of-life
                if battery_lifetime_status_raw & 8 == 8:
                    self.ups_state['battery_lifetime_status'].append("NEAR EOL ACK")#Near end of life was confirmed by user
                if battery_lifetime_status_raw & 16 == 16:
                    self.ups_state['battery_lifetime_status'].append("OVER EOL ACK")#Over end of life was confirmed by user
                
            elif msg_id == 0x6d:
                self.ups_state['battery_voltage'] = self.convert_from_bp(msg_data[0:2], 5, signed=True)
                self.ups_state['battery_soc'] = self.convert_from_bp(msg_data[2:4], 9, signed=False)
                
                #Simple self-test
                self.ups_state['battery_replacetest_cmd'] = int.from_bytes(bytes=msg_data[4:6], byteorder='big', signed=False)
                
                #Simple self-test
                battery_replacetest_status = int.from_bytes(bytes=msg_data[6:8], byteorder='big', signed=False)
#                 if self.ups_state.get('battery_replacetest_status_raw', 0) != battery_replacetest_status:#Show changes immediately
#                     print('battery_replacetest_status_raw is now ' + str(battery_replacetest_status))
                self.ups_state['battery_replacetest_status_raw'] = battery_replacetest_status
                self.ups_state['battery_replacetest_status'] = []
                if battery_replacetest_status == 0:
                    self.ups_state['battery_replacetest_status'].append("UNKNOWN")#Empty data
                
                if battery_replacetest_status & 1 == 1:
                    self.ups_state['battery_replacetest_status'].append('PENDING')#Test will start soon
                if battery_replacetest_status & 2 == 2:
                    self.ups_state['battery_replacetest_status'].append('IN PROGRESS')#Test is running
                if battery_replacetest_status & 4 == 4:
                    self.ups_state['battery_replacetest_status'].append('PASSED')#Battery passed replacement test
                if battery_replacetest_status & 8 == 8:
                    self.ups_state['battery_replacetest_status'].append('FAILED')#Battery failed replacement test
                if battery_replacetest_status & 16 == 16:
                    self.ups_state['battery_replacetest_status'].append('REFUSED')#UPS cannot test now, refused
                if battery_replacetest_status & 32 == 32:
                    self.ups_state['battery_replacetest_status'].append('ABORTED')#Test aborted
                if battery_replacetest_status & 64 == 64:
                    self.ups_state['battery_replacetest_status'].append('SOURCE PROTOCOL')#Start or stopping of test was triggered from protocol
                if battery_replacetest_status & 128 == 128:
                    self.ups_state['battery_replacetest_status'].append('SOURCE UI')#Start or stopping of test was triggered from user interface (UPS front panel)
                if battery_replacetest_status & 256 == 256:
                    self.ups_state['battery_replacetest_status'].append('SOURCE INTERNAL')#Start or stopping of test was triggered internally
                if battery_replacetest_status & 512 == 512:
                    self.ups_state['battery_replacetest_status'].append('INVALID STATE')#Invalid UPS Operating state to perform the test
                if battery_replacetest_status & 1024 == 1024:
                    self.ups_state['battery_replacetest_status'].append('INTERNAL FAULT')#Internal fault such as battery missing, inverter failure, overload, ...
                if battery_replacetest_status & 2048 == 2048:
                    self.ups_state['battery_replacetest_status'].append('SOC UNACCEPTABLE')#SOC is too low to do the test
                
                runtime_calibration_status = int.from_bytes(bytes=msg_data[10:12], byteorder='big', signed=False)
                self.ups_state['runtime_calibration_status_raw'] = runtime_calibration_status
                self.ups_state['runtime_calibration_status'] = []
                if battery_replacetest_status & 1 == 1:
                    self.ups_state['runtime_calibration_status'].append('PENDING')#Test will start soon
                if battery_replacetest_status & 2 == 2:
                    self.ups_state['runtime_calibration_status'].append('IN PROGRESS')#Test is running
                if battery_replacetest_status & 4 == 4:
                    self.ups_state['runtime_calibration_status'].append('PASSED')#Calibration completed
                if battery_replacetest_status & 8 == 8:
                    self.ups_state['runtime_calibration_status'].append('FAILED')#Calibration failed
                if battery_replacetest_status & 16 == 16:
                    self.ups_state['runtime_calibration_status'].append('REFUSED')#Test refused (too small load connected?)
                if battery_replacetest_status & 32 == 32:
                    self.ups_state['runtime_calibration_status'].append('ABORTED')#Test aborted
                if battery_replacetest_status & 64 == 64:
                    self.ups_state['battery_replacetest_status'].append('SOURCE PROTOCOL')#Start or stopping of test was triggered from protocol
                if battery_replacetest_status & 128 == 128:
                    self.ups_state['battery_replacetest_status'].append('SOURCE UI')#Start or stopping of test was triggered from user interface (UPS front panel)
                if battery_replacetest_status & 256 == 256:
                    self.ups_state['battery_replacetest_status'].append('SOURCE INTERNAL')#Start or stopping of test was triggered internally
                if battery_replacetest_status & 512 == 512:
                    self.ups_state['battery_replacetest_status'].append('INVALID STATE')#Invalid UPS Operating state to perform the test
                if battery_replacetest_status & 1024 == 1024:
                    self.ups_state['battery_replacetest_status'].append('INTERNAL FAULT')#Internal fault such as battery missing, inverter failure, overload, ...
                if battery_replacetest_status & 2048 == 2048:
                    self.ups_state['battery_replacetest_status'].append('SOC UNACCEPTABLE')#SOC is too low to do the test
                if battery_replacetest_status & 4096 == 4096:
                    self.ups_state['battery_replacetest_status'].append('LOAD CHANGED')#The connected load varied too much to be able to calibrate
                if battery_replacetest_status & 8192 == 8192:
                    self.ups_state['battery_replacetest_status'].append('AC INPUT NOT ACCEPTABLE')#AC Input not acceptable so test was aborted
                if battery_replacetest_status & 16384 == 16384:
                    self.ups_state['battery_replacetest_status'].append('LOAD TOO LOW')#Connected load is too small to perform the calibration
                if battery_replacetest_status & 32768 == 32768:
                    self.ups_state['battery_replacetest_status'].append('OVERCHARGE IN PROGRESS')#A battery overcharge is in progress so calibration would be inaccurate
                
                self.ups_state['runtime_remaining'] = int(self.convert_from_bp(msg_data[14:16], 0, signed=False))#In seconds
            
            elif msg_id == 0x6e:
                self.ups_state['runtime_remaining_2'] = int(self.convert_from_bp(msg_data[0:4], 0, signed=False))#In seconds
                
            elif msg_id == 0x6f:
                self.ups_state['temperature'] = self.convert_from_bp(msg_data[0:2], 7, signed=True)

                self.ups_state['user_interface_cmd'] = int.from_bytes(msg_data[2:4], byteorder='big', signed=False) 
                               
                user_interface_status_raw = int.from_bytes(msg_data[4:6], byteorder='big', signed=False)
                self.ups_state['user_interface_status_raw'] = user_interface_status_raw
                self.ups_state['user_interface_status'] = []
                if user_interface_status_raw & 1 == 1:
                    self.ups_state['user_interface_status'].append("CONT. TEST IN PROGRESS")
                if user_interface_status_raw & 2 == 2:
                    self.ups_state['user_interface_status'].append("AUDIBLE ALARM IN PROGRESS")
                if user_interface_status_raw & 4 == 4:
                    self.ups_state['user_interface_status'].append("AUDIBLE ALARM MUTED")
                
                self.ups_state['voltage_out'] = self.convert_from_bp(msg_data[6:8], 6, signed=False)
                self.ups_state['current_out'] = self.convert_from_bp(msg_data[8:10], 5, signed=False)
                self.ups_state['frequency_out'] = self.convert_from_bp(msg_data[10:12], 7, signed=False)
                self.ups_state['apparent_power_pctused'] = self.convert_from_bp(msg_data[12:14], 8, signed=False)
                self.ups_state['real_power_pctused'] = self.convert_from_bp(msg_data[14:16], 8, signed=False)

            elif msg_id == 0x70:
                input_status = int.from_bytes(bytes=msg_data[2:4], byteorder='big', signed=False)
                self.ups_state['input_status_raw'] = input_status
                self.ups_state['input_status'] = []
                if input_status & 1 == 1:
                    self.ups_state['input_status'].append("ACCEPTABLE")
                if input_status & 2 == 2:
                    self.ups_state['input_status'].append("PENDING ACCEPTABLE")
                if input_status & 4 == 4:
                    self.ups_state['input_status'].append("LOW VOLTAGE")
                if input_status & 8 == 8:
                    self.ups_state['input_status'].append("HIGH VOLTAGE")
                if input_status & 16 == 16:
                    self.ups_state['input_status'].append("DISTORTED")
                if input_status & 32 == 32:
                    self.ups_state['input_status'].append("BOOST")
                if input_status & 64 == 64:
                    self.ups_state['input_status'].append("TRIM")
                if input_status & 128 == 128:
                    self.ups_state['input_status'].append("LOW FREQUENCY")
                if input_status & 256 == 256:
                    self.ups_state['input_status'].append("HIGH FREQUENCY")
                if input_status & 512 == 512:
                    self.ups_state['input_status'].append("PHASE NOT LOCKED")
                if input_status & 1024 == 1024:
                    self.ups_state['input_status'].append("DELTA PHASE OUT OF RANGE")
                if input_status & 2048 == 2048:
                    self.ups_state['input_status'].append("NEUTRAL NOT CONNECTED")
                if input_status & 4096 == 4096:
                    self.ups_state['input_status'].append("NOT ACCEPTABLE")
                if input_status & 8192 == 8192:
                    self.ups_state['input_status'].append("PLUG RATING EXCEEDED")
                
                self.ups_state['voltage_in'] = self.convert_from_bp(msg_data[4:6], 6, signed=False)
                self.ups_state['frequency_in'] = self.convert_from_bp(msg_data[6:8], 7, signed=False)
                self.ups_state['green_mode'] = int.from_bytes(bytes=msg_data[8:10], byteorder='big', signed=True)

                powsys_error = int.from_bytes(bytes=msg_data[10:12], byteorder='big', signed=False)
                self.ups_state['powsys_error_raw'] = powsys_error
                self.ups_state['powsys_error'] = []
                if powsys_error & 1 == 1:
                    self.ups_state['powsys_error'].append("OUTPUT OVERLOAD")
                if powsys_error & 2 == 2:
                    self.ups_state['powsys_error'].append("OUTPUT SHORT CIRCUIT")
                if powsys_error & 4 == 4:
                    self.ups_state['powsys_error'].append("OUTPUT OVERVOLTAGE")
                if powsys_error & 8 == 8:
                    self.ups_state['powsys_error'].append("TRANSFORMER DC IMBALANCE")
                if powsys_error & 16 == 16:
                    self.ups_state['powsys_error'].append("OVERTEMPERATURE")
                if powsys_error & 32 == 32:
                    self.ups_state['powsys_error'].append("BACKFEEDING")
                if powsys_error & 64 == 64:
                    self.ups_state['powsys_error'].append("AVR RELAY FAULT")
                if powsys_error & 128 == 128:
                    self.ups_state['powsys_error'].append("PFC INPUT RELAY FAULT")
                if powsys_error & 256 == 256:
                    self.ups_state['powsys_error'].append("OUTPUT RELAY FAULT")
                if powsys_error & 512 == 512:
                    self.ups_state['powsys_error'].append("BYPASS RELAY FAULT")
                if powsys_error & 1024 == 1024:
                    self.ups_state['powsys_error'].append("FAN FAULT")
                if powsys_error & 2048 == 2048:
                    self.ups_state['powsys_error'].append("PFC FAULT")
                if powsys_error & 4096 == 4096:
                    self.ups_state['powsys_error'].append("DC BUS OVERVOLTAGE")
                if powsys_error & 4096 == 4096:
                    self.ups_state['powsys_error'].append("INVERTER FAULT")

                general_error = int.from_bytes(bytes=msg_data[12:14], byteorder='big', signed=False)
                self.ups_state['general_error_raw'] = general_error
                self.ups_state['general_error'] = []
                if general_error & 1 == 1:
                    self.ups_state['general_error'].append("SITE WIRING FAULT")
                if general_error & 2 == 2:
                    self.ups_state['general_error'].append("EEPROM ERROR")
                if general_error & 4 == 4:
                    self.ups_state['general_error'].append("AD CONVERTER ERROR")
                if general_error & 8 == 8:
                    self.ups_state['general_error'].append("LOGIC PSU FAULT")
                if general_error & 16 == 16:
                    self.ups_state['general_error'].append("INTERNAL COMM FAULT")
                if general_error & 32 == 32:
                    self.ups_state['general_error'].append("UI BUTTON FAULT")
                if general_error & 128 == 128:
                    self.ups_state['general_error'].append("EPO ACTIVE")

                batt_error = int.from_bytes(bytes=msg_data[14:16], byteorder='big', signed=False)
                self.ups_state['battery_error_raw'] = batt_error
                self.ups_state['battery_error'] = []
                if batt_error & 1 == 1:
                    self.ups_state['battery_error'].append("DISCONNECTED")
                if batt_error & 2 == 2:
                    self.ups_state['battery_error'].append("OVERVOLTAGE")
                if batt_error & 4 == 4:
                    self.ups_state['battery_error'].append("NEEDS REPLACEMENT")
                if batt_error & 8 == 8:
                    self.ups_state['battery_error'].append("OVERTEMPERATURE")
                if batt_error & 16 == 16:
                    self.ups_state['battery_error'].append("CHARGER FAULT")
                if batt_error & 32 == 32:
                    self.ups_state['battery_error'].append("TEMP SENSOR FAULT")
                if batt_error & 64 == 64:
                    self.ups_state['battery_error'].append("BATTERY BUS SOFT START FAULT")
                if batt_error & 128 == 128:
                    self.ups_state['battery_error'].append("HIGH TEMPERATURE")
                if batt_error & 256 == 256:
                    self.ups_state['battery_error'].append("GENERAL ERROR")
                if batt_error & 512 == 512:
                    self.ups_state['battery_error'].append("COMM ERROR")
                
                    
            elif msg_id == 0x71:
                self.ups_state['ups_cmd'] = int.from_bytes(msg_data[0:2], byteorder='big', signed=False)#Bitfield
                
                #This ID is actually used to send commands to the outlet. No idea what the read values say, probably not relevant
                self.ups_state['outlet_cmd'] = int.from_bytes(msg_data[8:10], byteorder='big', signed=False)
            
            elif msg_id == 0x72:
                status_value = int.from_bytes(msg_data[0:2], byteorder='big', signed=False)
                self.ups_state['outlet_status_raw'] = status_value
                self.ups_state['outlet_status'] = []
                if status_value & 1 == 1:
                    self.ups_state['outlet_status'].append("OUTLET ON")
                if status_value & 2 == 2:
                    self.ups_state['outlet_status'].append("OUTLET OFF")
                if status_value & 4 == 4:
                    self.ups_state['outlet_status'].append("REBOOTING")
                if status_value & 8 == 8:
                    self.ups_state['outlet_status'].append("SHUTTING DOWN")
                if status_value & 16 == 16:
                    self.ups_state['outlet_status'].append("SLEEPING")
                #From here on unsure because different sources give different values/explanations
                if status_value & 128 == 128:
                    self.ups_state['outlet_status'].append("OUTLET OVERLOAD")
                if status_value & 256 == 256:
                    self.ups_state['outlet_status'].append("PENDING OUTLET ON")#Waiting to turn outlet on
                if status_value & 512 == 512:
                    self.ups_state['outlet_status'].append("PENDING OUTLET OFF")#Waiting to turn outlet off
                if status_value & 1024 == 1024:
                    self.ups_state['outlet_status'].append("WAIT ON AC")#Wait for grid AC to turn on outlet
                if status_value & 2048 == 2048:
                    self.ups_state['outlet_status'].append("WAIT ON MIN RUNTIME")#Waiting on enough charge to reach minimum runtime, before turning on outlet
                if status_value & 4096 == 4096:
                    self.ups_state['outlet_status'].append("LOW RUNTIME")#indicates the run time is below the setting for the outlet group
                
            elif msg_id == 0x76:
                status_value = int.from_bytes(msg_data[8:10], byteorder='big', signed=False)
                self.ups_state['ups_status_raw'] = status_value
                self.ups_state['ups_status'] = []
                if status_value & 1 == 1:
                    self.ups_state['ups_status'].append("RESERVED BIT")
                if status_value & 2 == 2:
                    self.ups_state['ups_status'].append("ONLINE")
                if status_value & 4 == 4:
                    self.ups_state['ups_status'].append("ON BATTERY")
                if status_value & 8 == 8:
                    self.ups_state['ups_status'].append("BYPASS ON")
                if status_value & 16 == 16:
                    self.ups_state['ups_status'].append("OUTPUT OFF")
                if status_value & 32 == 32:
                    self.ups_state['ups_status'].append("FAULT")
                if status_value & 64 == 64:
                    self.ups_state['ups_status'].append("INPUT BAD")#Missing or bad AC power input
                if status_value & 128 == 128:
                    self.ups_state['ups_status'].append("TESTING")#A test is in progress
                if status_value & 256 == 256:
                    self.ups_state['ups_status'].append("PENDING OUTPUT ON")
                if status_value & 512 == 512:
                    self.ups_state['ups_status'].append("PENDING OUTPUT OFF")
                if status_value & 8192 == 8192:
                    self.ups_state['ups_status'].append("GREEN MODE")
                if status_value & 16384 == 16384:
                    self.ups_state['ups_status'].append("InformationalAlert")
                    
                status_chg_cause_raw = int.from_bytes(msg_data[10:12], byteorder='big', signed=False)
                self.ups_state['status_chg_cause_raw'] = status_chg_cause_raw
                #These are documented in the APC Modbus documentation
                if status_chg_cause_raw == 0:
                    self.ups_state['status_chg_cause'] = "SystemInitialization"
                if status_chg_cause_raw == 1:
                    self.ups_state['status_chg_cause'] = "HighInputVoltage"
                if status_chg_cause_raw == 2:
                    self.ups_state['status_chg_cause'] = "LowInputVoltage"
                if status_chg_cause_raw == 3:
                    self.ups_state['status_chg_cause'] = "DistortedInput"
                if status_chg_cause_raw == 4:
                    self.ups_state['status_chg_cause'] = "RapidChangeOfInputVoltage"
                if status_chg_cause_raw == 5:
                    self.ups_state['status_chg_cause'] = "HighInputFrequency"
                if status_chg_cause_raw == 6:
                    self.ups_state['status_chg_cause'] = "LowInputFrequency"
                if status_chg_cause_raw == 7:
                    self.ups_state['status_chg_cause'] = "FreqAndOrPhaseDifference"
                if status_chg_cause_raw == 8:
                    self.ups_state['status_chg_cause'] = "AcceptableInput"
                if status_chg_cause_raw == 9:
                    self.ups_state['status_chg_cause'] = "AutomaticTest"
                if status_chg_cause_raw == 10:
                    self.ups_state['status_chg_cause'] = "TestEnded"
                if status_chg_cause_raw == 11:
                    self.ups_state['status_chg_cause'] = "LocalUICommand"
                if status_chg_cause_raw == 12:
                    self.ups_state['status_chg_cause'] = "ProtocolCommand"
                if status_chg_cause_raw == 13:
                    self.ups_state['status_chg_cause'] = "LowBatteryVoltage"
                if status_chg_cause_raw == 14:
                    self.ups_state['status_chg_cause'] = "GeneralError"
                if status_chg_cause_raw == 15:
                    self.ups_state['status_chg_cause'] = "PowerSystemError"
                if status_chg_cause_raw == 16:
                    self.ups_state['status_chg_cause'] = "BatterySystemError"
                if status_chg_cause_raw == 17:
                    self.ups_state['status_chg_cause'] = "ErrorCleared"
                if status_chg_cause_raw == 18:
                    self.ups_state['status_chg_cause'] = "AutomaticRestart"
                if status_chg_cause_raw == 19:
                    self.ups_state['status_chg_cause'] = "DistortedInverterOutput"
                if status_chg_cause_raw == 20:
                    self.ups_state['status_chg_cause'] = "InverterOutputAcceptable"
                if status_chg_cause_raw == 21:
                    self.ups_state['status_chg_cause'] = "EPOInterface"
                if status_chg_cause_raw == 22:
                    self.ups_state['status_chg_cause'] = "InputPhaseDeltaOutOfRange"
                if status_chg_cause_raw == 23:
                    self.ups_state['status_chg_cause'] = "InputNeutralNotConnected"
                if status_chg_cause_raw == 24:
                    self.ups_state['status_chg_cause'] = "ATSTransfer"
                if status_chg_cause_raw == 25:
                    self.ups_state['status_chg_cause'] = "ConfigurationChange"
                if status_chg_cause_raw == 26:
                    self.ups_state['status_chg_cause'] = "AlertAsserted"
                if status_chg_cause_raw == 27:
                    self.ups_state['status_chg_cause'] = "AlertCleared"
                if status_chg_cause_raw == 28:
                    self.ups_state['status_chg_cause'] = "PlugRatingExceeded"
                if status_chg_cause_raw == 29:
                    self.ups_state['status_chg_cause'] = "OutletGroupStateChange"
                if status_chg_cause_raw == 30:
                    self.ups_state['status_chg_cause'] = "FailureBypassExpired"
                
                    
            elif msg_id == 0x79:
                self.ups_state['temperature_2'] = self.convert_from_bp(msg_data[4:6], 7, signed=True)
                self.ups_state['humidity_pct'] = self.convert_from_bp(msg_data[6:8], 9, signed=False)
                self.ups_state['temperature_3'] = self.convert_from_bp(msg_data[14:16], 7, signed=True)
                
            elif msg_id == 0x7a:
                self.ups_state['humidity_pct_2'] = self.convert_from_bp(msg_data[0:2], 9, signed=False)
                
            elif msg_id == 0x7e:
                self.ups_state['password_1'] = msg_data[8:12]
                self.ups_state['password_2'] = msg_data[12:16]
            
            elif msg_id == 0x7f:
                self.ups_state['challenge_status'] = msg_data[14:16]
                if self.state == CommState.MODE0:
                    #We have received all of the message IDs for the first time since reset.
                    #Now we need to answer the challenge string of the UPS.
                    challenge = self.calculate_challenge()
                    challenge_msg = self.create_msg_data(msg_id=0x7e, offset=12, msg_data=challenge)
                    self.s.write(challenge_msg)
                    rcv_data = self.receive_msg()
                    if rcv_data is not None and len(rcv_data) >0 and rcv_data[0] == 0x7e:
                        self.state = CommState.MODE1
                    else:
                        return False
                    
            #Default behavior is to request next data    
            self.next_apc_msg = APC_CMD_NEXT
            return True
        
        else:
            self.next_apc_msg = APC_CMD_RESET
            return False
        
    def calculate_challenge(self):
        ''' Calculate challenge from actual known ups state '''
        b0 = self.ups_state['series_id_raw'][1]
        b1 = self.ups_state['series_id_raw'][0]
        for header_byte in self.ups_state['header_raw']:
            b0 = (b0 + header_byte) % 255
            b1 = (b1 + b0) % 255
        for serial_nb_byte in self.ups_state['serial_nb_raw']:
            b0 = (b0 + serial_nb_byte) % 255
            b1 = (b1 + b0) % 255
        for pw1_byte in self.ups_state['password_1'][0:2]:
            b0 = (b0 + pw1_byte) % 255
            b1 = (b1 + b0) % 255
             
        challenge = bytearray([1, 1, b0, b1])
        return challenge
    
    def convert_from_bp(self, data, frac_pos, signed=False):
        ''' Convert the binary point number in data to a float, given the fractional bit position '''
        data = int.from_bytes(data, byteorder='big', signed=signed)
        value = data / (2**frac_pos)
        return value
    
    def convert_to_bp(self, value, frac_pos):   
        ''' Convert the given data to binary point format, with the specified fractional bit position '''            
        data = int(value * 2**frac_pos).to_bytes(2, byteorder='big')
        return data
    
    def convert_to_datetime(self, value):
        ''' Convert from days since 1 Jan. 2000 to datetime object '''
        return datetime.datetime(2000,1,1) + datetime.timedelta(days=value)

from cmd import Cmd

class ApcCLI(Cmd):
    
    intro = 'ApcComm CLI. Type help or ? to list commands.\n'
    prompt = '(apc) '
    file = None
    
    def __init__(self, apc_comm):
        super(ApcCLI, self).__init__()
        self.apc_comm = apc_comm
    
    def print_keys(self, keylist):
        for key in sorted(keylist):
            print(str(key) + " = " + str(self.apc_comm.ups_state.get(key, "Unknown")))    
    
    def do_commstate(self, arg):
        'Show the communication thread state'
        print(self.apc_comm.state)
    
    def do_voltage(self, arg):
        'Show all actual voltages'
        self.print_keys(['voltage_in', 'voltage_out', 'battery_voltage'])
        
    def do_current(self, arg):
        'Show all actual currents'
        self.print_keys(['current_out'])
        
    def do_frequency(self, arg):
        'Show actual frequencies'
        self.print_keys(['frequency_in', 'frequency_out'])
        
    def do_runtime(self, arg):
        'Show runtime information and configuration'
        self.print_keys(['runtime_remaining', 'runtime_remaining_2', 'runtime_minimum_shown', 'runtime_remaining_outletoff', 'runtime_limit_outletoff'])
    
    def do_battery(self, arg):
        'Show battery information and error'
        self.print_keys(['battery_voltage', 'battery_soc', 'battery_error', 'battery_error_raw'])
    
    def do_status(self, arg):
        'Show UPS status fields'
        self.print_keys(['ups_status', 'outlet_status'])
    
    def do_all(self, arg):
        'Show all known parameters'
        self.print_keys(self.apc_comm.ups_state.keys())
        
    def do_set(self, arg):
        'Configure a certain parameter'
        args = arg.split(" ")
        if args[0] == 'runtime_limit_outletoff':
            data = apccomm.convert_to_bp(int(args[1]), frac_pos=0)
            raw_msg = apccomm.create_msg_data(msg_id=0x4c, offset=14, msg_data=data)
            self.send_msg(raw_msg)
        elif args[0] == 'outlet_cmd':
            #Send an outlet command
            if args[1] == 'CANCEL':#Cancel pending actions
                cmd = 1#Cancels pending actions to the targets selected. No modifiers are allowed.
                cmd += 256#Target is Main outlet
            elif args[1] == 'ON':#Turn on outlet immediately
                cmd = 0
                cmd += 2#Turn ON
                cmd += 32#Allow the output to turn on without AC input power conditions met
                cmd += 256#Target is Main outlet
                cmd += 16384#Command from serial
            elif args[1] == 'ON_DELAY':#Turn on outlet after delay
                cmd = 0
                cmd += 2#Turn ON
                cmd += 32#Allow the output to turn on without AC input power conditions met
                cmd += 64#Use OFF delay
                cmd += 256#Main outlet
                cmd += 16384#Command from serial
            elif args[1] == 'OFF':#Turn off outlet immediately
                cmd = 0
                cmd += 4#Turn OFF
                cmd += 256#Target is Main outlet
                cmd += 16384#Command from serial
            elif args[1] == 'OFF_DELAY':#Turn off outlet after delay
                cmd = 0
                cmd += 4#Turn OFF
                cmd += 128#Use OFF delay
                cmd += 256#Target is Main outlet
                cmd += 16384#Command from serial
            elif args[1] == 'SHUTDOWN':#Turn off outlet and wait for AC power returns, battery has charged enough, ...
                cmd = 0
                cmd += 8#Shutdown
                cmd += 256#Target is Main outlet
                cmd += 16384#Command from serial
            elif args[1] == 'REBOOT':#Turn off outlet and turn on again
                cmd = 0
                cmd += 16#Reboot
                cmd += 256#Target is Main outlet
                cmd += 16384#Command from serial
            elif args[1] == 'CANCEL':
                cmd = 1
                cmd += 256#Target is Main outlet
                cmd += 16384#Command from serial
            else:
                print("Unknown option")
                return
            cmd = int(cmd).to_bytes(length=2, byteorder='big', signed=False)
            raw_msg = apccomm.create_msg_data(msg_id=0x71, offset=0x08, msg_data=cmd)
            self.send_msg(raw_msg)
                
        elif args[0] == 'battery_replacetest_cmd':
            if args[1] == 'START':
                data = bytearray([0x00, 0x01])#Setting bit 0 will trigger the test. Note that there needs to be sufficient load connected for this test not to be refused.
            elif args[1] == 'STOP':
                data = bytearray([0x00, 0x02])#Setting bit 1 will cancel the test
            else:
                print("Unknown option")
                return
            raw_msg = apccomm.create_msg_data(msg_id=0x6d, offset=4, msg_data=data)
            self.send_msg(raw_msg)
            
        elif args[0] == 'runtime_calibration_cmd':
            if args[1] == 'START':
                data = bytearray([0x00, 0x01])#Setting bit 0 will trigger the test. Note that there needs to be sufficient load connected for this test not to be refused.
            elif args[1] == 'STOP':
                data = bytearray([0x00, 0x02])#Setting bit 1 will cancel the test
            else:
                print("Unknown option")
                return
            raw_msg = apccomm.create_msg_data(msg_id=0x6d, offset=8, msg_data=data)
            self.send_msg(raw_msg)
            
        elif args[0] == 'battery_test_interval':
            test_interval_choices = ["DISABLED", "STARTUP","EACH 7 DAYS SINCE STARTUP","EACH 14 DAYS SINCE STARTUP","EACH 7 DAYS SINCE LAST","EACH 14 DAYS SINCE LAST"]
            val = int(args[1])
            choice = val if val >=0 and val < len(test_interval_choices) else 0
            print("Setting battery_test_interval to " + test_interval_choices[choice])
            data = bytearray([0x00, 1**choice])
            raw_msg = apccomm.create_msg_data(msg_id=0x4a, offset=0, msg_data=data)
            self.send_msg(raw_msg)
            
        elif args[0] == 'ups_cmd':
            if args[1] == 'RESET':#Factory reset, to defaults
                data = bytearray([0x00, 0x08])
            else:
                print("Unknown option")
                return
            raw_msg = apccomm.create_msg_data(msg_id=0x71, offset=0, msg_data=data)
            self.send_msg(raw_msg)
            
        elif args[0] == 'user_interface_cmd':
            if args[1] == 'SHORT_TEST':#Perform the momentary local UI test, e.g., light all the LEDs and sound the beeper.
                data = bytearray([0x00, 0x01])
            elif args[1] == 'CONT_TEST':#Perform the continuous local UI test, e.g., light all the LEDs and sound the beeper until canceled. To cancel, trigger the short test
                data = bytearray([0x00, 0x02])
            elif args[1] == 'MUTE_ON':#Mute all the active alarms in the UPS
                data = bytearray([0x00, 0x04])   
            elif args[1] == 'MUTE_OFF':#Cancels any muting
                data = bytearray([0x00, 0x08])
            elif args[1] == 'ACK_ALARM':#Acknowledges active battery alarms
                data = bytearray([0x00, 0x20])
            else:
                print("Unknown option")
                return
            raw_msg = apccomm.create_msg_data(msg_id=0x6f, offset=2, msg_data=data)
            self.send_msg(raw_msg) 
            
        else:
            print("Unrecognized parameter \'" + args[0] + "\'")
            
    def do_write(self, arg):
        'Send a raw message to the UPS. Format: write <hex ID> <hex offset> <hex length> <hex data>'
        args = arg.split(" ")
        if len(args) == 4:
            msg_id = int(args[0], 16)
            msg_offset = int(args[1], 16)
            msg_len = int(args[2], 16)
            msg_data = int(args[3], 16).to_bytes(length=msg_len, byteorder='big', signed=False)
            raw_msg = apccomm.create_msg_data(msg_id, msg_offset, msg_data)
            self.send_msg(raw_msg)
        else:
            print("Invalid nb of arguments")
    
    def do_exit(self, arg):
        'Exit the application'
        self.apc_comm.running = False
        return True
    
    def send_msg(self, raw_msg):
        print("Sending " + raw_msg.hex())
        if not apccomm.send_apc_msg(raw_msg):
            print("Error sending")

if __name__ == '__main__':

    if len(sys.argv) != 2:
        print("APC UPS Serial test program\n2019 KlaasDC\n\nUsage: " + sys.argv[0] + " <serial port>\nExample: " + sys.argv[0] + " /dev/ttyS0")
        sys.exit(0)
    
    ser = serial.Serial(sys.argv[1], 9600, timeout=APC_RCV_TIMEOUT, parity=serial.PARITY_NONE)
    print("Starting on " + ser.name)
    
    apccomm = ApcComm(serial_port=ser)
    apccomm.start()    
    ApcCLI(apccomm).cmdloop()
    
    apccomm.running = False
    time.sleep(0.5)
    ser.close()