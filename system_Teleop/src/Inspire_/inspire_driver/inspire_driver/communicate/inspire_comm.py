"""
File containing the SRBL Inspire gripper class for controlling the gripper and obtaining sensor data.
Modified from the file which was written for GELLO, for a general purpose wrapper.
Written by Seongjun Koh (Soft Robotics and Bionics Lab, Seoul National University), based on the Inspire SDK documentation and sample code provided by the manufacturer.
Last modification: March 17, 2026
"""

import serial
import time
import threading

INSPIRE_regdict = {
    'ID'         : 1000,
    'baudrate'   : 1001,
    'mode'       : 1100,
    'clearErr'   : 1003,
    'forceClb'   : 1007,
    'angleSet'   : 1040, # Use this to set finger position, units of 0.1 degrees, -1 for no change
    'forceSet'   : 1046,
    'speedSet'   : 1052,
    'angleAct'   : 1064, # Use this to get finger position : angle Actual, units of 0.1 degrees
    'forceAct'   : 1070,
    'currAct'    : 1076, # Use this to get current data, mA
    'errCode'    : 1082,
    'statusCode' : 1088,
    'temp'       : 1094,
    'ip'         : 1700,
    'actionSeq'  : 2160,
    'actionRun'  : 2162, 
    'sensorData' : 3000
}

SRBL_INSPIRE_FINGER_LIST = ['finger_little', 'finger_ring', 'finger_middle', 'finger_index', 'finger_thumb_bending', 'finger_thumb_rotation']
SRBL_INSPIRE_PALM_LIST = ['palm_right', 'palm_middle', 'palm_left']

SRBL_INSPIRE_FINGER_LOWER_LIMIT = [900, 900, 900, 900, 1100, 600] # Lower limit of the finger joint position, units of 0.1 degrees
# 4 fingers : 900, thunmb bending : 1100, thumb rotation : 600
SRBL_INSPIRE_FINGER_UPPER_LIMIT = [1740, 1740, 1740, 1740, 1350, 1800] # Upper limit of the finger joint position, units of 0.1 degrees
# 4 fingers : 1740, thunmb bending : 1350, thumb rotation : 1800

class SRBL_Inspire_gripper:
    def __init__(self, device_name="/dev/ttyUSB1", baudrate=115200):
        self.ser = serial.Serial(device_name, baudrate, timeout=0.1)
        self.sleep_time = 0.005
        
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.get_command_list : list[dict[str, bytes]] = []
        self.set_command_list : dict[str, bytes]

        self.isSetPosition = False
        self.is_running = False
        self.worker_thread = threading.Thread(target=self.command_sendor, daemon=True)

        self.is_running = True
        self.worker_thread.start()

        self.received_angle = [0.0]*6
        self.received_sensor = [0.0]*29

    def __del__(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def destroy_thread(self):
        if self.is_running:
            self.worker_thread.join(timeout=1.0)
            self.is_running = False        

    def get_command_append(self, cmd, num, bytes:bytes):
        command = {cmd:bytes, "len":num}
        self.get_command_list.append(command)

    def set_command_append(self, cmd, num, bytes:bytes):
        command = {cmd:bytes, "len":num}
        self.get_command_list.append(command)

    def command_sendor(self):
        while True:
            if len(self.get_command_list):

                command = self.get_command_list.pop(0)
                k, v = command.items()
                cmd = k[0]
                data = k[1]
                num = v[1]

                val = []

                self.ser.write(data)

                time.sleep(self.sleep_time)
                recv = self.ser.read(num+8)
                # recv = [0]

                if len(recv) >= 4:
                    if num == (recv[3] & 0xFF) - 3:
                        num = (recv[3] & 0xFF) - 3

                        if(num == len(recv)-8):
                            for i in range(num):
                                value = (recv[7 + i])
                                val.append(value)

                match(cmd):
                    case "Get_Angle":
                        angle = []
                        recv_data = self.get_position_values(val)
                        if recv_data != []: self.received_angle = recv_data
                        self.send_get_sensor()

                    case "Get_Sensor":
                        sensor = []
                        recv_data = self.get_sensor_values(val)
                        
                        for i in range(5):
                            for k, v in recv_data[SRBL_INSPIRE_FINGER_LIST[i]].items():
                                sensor.append((float)(v))

                        for i in range(3):
                            for k, v in recv_data[SRBL_INSPIRE_PALM_LIST[i]].items():
                                sensor.append((float)(v))

                        self.received_sensor = sensor
                        if self.isSetPosition: 
                            self.send_set_position()
                            self.isSetPosition = False
                        else: self.send_get_angle()

                    case "Set_Position":
                        val = []
                        self.send_get_angle()
            else:
                time.sleep(0.005)

    def _writeRegister(self, id, add, num, val):
        bytes = [0xEB, 0x90]
        bytes.append(id)
        bytes.append(num + 3)
        bytes.append(0x12)
        bytes.append(add & 0xFF)
        bytes.append((add >> 8) & 0xFF)
        for i in range(num):
            bytes.append(val[i])
        checksum = 0x00
        for i in range(2, len(bytes)):
            checksum += bytes[i]
        checksum &= 0xFF
        bytes.append(checksum)

        cmd = "Set_Position"
        self.get_command_append(cmd, num, bytes)

    def _readRegister(self, id, add, num, cmd, mute=True):
        bytes = [0xEB, 0x90]
        bytes.append(id)
        bytes.append(0x04)
        bytes.append(0x11)
        bytes.append(add & 0xFF)
        bytes.append((add >> 8) & 0xFF)
        bytes.append(num)
        checksum = 0x00
        for i in range(2, len(bytes)):
            checksum += bytes[i]
        checksum &= 0xFF
        bytes.append(checksum)

        self.get_command_append(cmd, num, bytes)
        self.ser.write(bytes)
        recv = self.ser.read(num+8)

        if len(recv) < 4:
            return []

        if num != (recv[3] & 0xFF) - 3:
            return []

        num = (recv[3] & 0xFF) - 3
        val = []
        if(num == len(recv)-8):
            for i in range(num):
                value = (recv[7 + i])
                val.append(value)
        return val

    def send_get_angle(self):
        self._readRegister(1, INSPIRE_regdict['angleAct'], 12, "Get_Angle",True)

    def send_get_sensor(self):
        self._readRegister(1, INSPIRE_regdict['sensorData'], 68, "Get_Sensor",True)

    def send_set_position(self):
        target_data = self.target_position_val
        self._writeRegister(1, INSPIRE_regdict['angleSet'], 12, target_data)
        
    def _SRBL_bytes_to_int16(self, val):
        if len(val) < 2:
            raise ValueError("Not enough bytes to convert to int16")
        value = val[0] + (val[1] << 8)
        if value > 32767:
            value -= 65536
        return value

    def _SRBL_Inspire_proximity(self, vals):
        if len(vals) != 3:
            raise ValueError("Invalid proximity sensor data")
        value = vals[0] + (vals[1] << 8) + (vals[2] << 16)
        return value

    def get_position_values(self, val):
        if len(val) < 12:
            return []
        val_act = []
        for i in range(6):
            value_act = self._SRBL_bytes_to_int16(val[i*2:(i*2)+2]) / 10.0
            val_act.append(value_act)
        return val_act

    def move_fingers(self, targets, limit=True):
        if limit:
            for i in range(6):
                if targets[i] != -1:
                    targets[i] = int(min(SRBL_INSPIRE_FINGER_UPPER_LIMIT[i], max(SRBL_INSPIRE_FINGER_LOWER_LIMIT[i], targets[i])))
        val_reg = []
        for i in range(6):
            val_reg.append(targets[i] & 0xFF)
            val_reg.append((targets[i] >> 8) & 0xFF)
            
        self.target_position_val = val_reg
        self.isSetPosition = True

    def get_sensor_values(self, val):
        sensor_vals = {SRBL_INSPIRE_FINGER_LIST[i]: {} for i in range(5)}
        if len(val) < 68:
            return []
        SJ_tmp_flag = False

        for i in range(5):
            idx = 10 * i
            sensor_vals[SRBL_INSPIRE_FINGER_LIST[i]]['normal'] = self._SRBL_bytes_to_int16(val[idx:idx+2]) / 100.0 # convert to N
            sensor_vals[SRBL_INSPIRE_FINGER_LIST[i]]['tangential'] = self._SRBL_bytes_to_int16(val[idx+2:idx+4]) / 100.0 # convert to N
            sensor_vals[SRBL_INSPIRE_FINGER_LIST[i]]['tangential_dir'] = self._SRBL_bytes_to_int16(val[idx+4:idx+6])
            sensor_vals[SRBL_INSPIRE_FINGER_LIST[i]]['proximity'] = self._SRBL_Inspire_proximity(val[idx+6:idx+9])
            if SJ_tmp_flag:
                print(f"========")
                print(f"raw: {val[idx+6:idx+10]}")
                print(f"sum: {val[idx+6] + (val[idx+7] << 8) + (val[idx+8] << 16) + (val[idx+9] << 24)}")
                for i in range(4):
                    print(f"{val[idx+9-i]:x}", end=' ')
                print(f"\nh/l: {self._SRBL_bytes_to_int16(val[idx+8:idx+10])} / {self._SRBL_bytes_to_int16(val[idx+6:idx+8])}")
                SJ_tmp_flag = False
        for i in range(len(SRBL_INSPIRE_PALM_LIST)):
            sensor_vals[SRBL_INSPIRE_PALM_LIST[i]] = {}
            idx = 50 + 6 * i
            sensor_vals[SRBL_INSPIRE_PALM_LIST[i]]['normal'] = self._SRBL_bytes_to_int16(val[idx:idx+2]) / 100.0 # convert to N
            sensor_vals[SRBL_INSPIRE_PALM_LIST[i]]['tangential'] = self._SRBL_bytes_to_int16(val[idx+2:idx+4]) / 100.0 # convert to N
            sensor_vals[SRBL_INSPIRE_PALM_LIST[i]]['tangential_dir'] = self._SRBL_bytes_to_int16(val[idx+4:idx+6])
       
        return sensor_vals

    def get_current_values(self, val):
        if len(val) < 12:
            raise RuntimeError("Failed to read gripper current data")
        current_vals = []
        for i in range(6):
            current_vals.append(self._SRBL_bytes_to_int16(val[i*2:i*2+2]) / 1000.0) # convert mA to A
        return current_vals

    def get_velocity_values(self):
        """
        Inspire does not provide velocity data, so this function is not implemented.
        If needed, velocity can be estimated by numerical differentiation of position data.
        """
        pass
