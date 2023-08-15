import time
import serial
import numpy

DWM_LISTENER_PORT = "/dev/ttyACM0"
DWM_DATA_RECORD_PATH = (
    f'src/uwb_serial_real_test/serial_data_record/dwm_distance_{time.time()}'
)

# Globals to save the data
timestamps_record = []
distances_record = []

def configure_serial_port(serial_port):
    # Serial port settings
    dwm_listener_serial = serial.Serial(
        port = serial_port,
        baudrate = 115200,
        parity = serial.PARITY_ODD,
        stopbits = serial.STOPBITS_TWO,
        bytesize = serial.SEVENBITS
    )
    print('##### Serial Port Configured #####')

    return dwm_listener_serial

def main(args=None):
    dwm_serial = configure_serial_port(DWM_LISTENER_PORT)

     # close the serial port in case the previous run didn't closed it properly
    dwm_serial.close()
    # sleep for one sec
    time.sleep(1)
    # open serial port
    dwm_serial.open()

    try:
        while(True):
            dwm_readline = dwm_serial.read_until().decode("utf-8")

            if dwm_readline:
                uwb_distance_msg = dwm_readline.split(';')
                distance_value = uwb_distance_msg[2]
                timestamp_value = time.time()
                
                # Print the data
                print(
                    f'Distance from {uwb_distance_msg[0]} to '
                    f'{uwb_distance_msg[1]} is: {distance_value} at '
                    f'time: {timestamp_value}'
                )

                # Save into a list (timestamps and distances values)
                timestamps_record.append(timestamp_value)
                distances_record.append(distance_value)

    except Exception as e:
        print(f'Reading line failed because of: {e}')

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('Saving data into a numpy array')
        dwm_data = numpy.column_stack(
            (timestamps_record, distances_record)
        )
        print(f'Data saved shape: {dwm_data.shape}')
        numpy.save(
            DWM_DATA_RECORD_PATH,
            dwm_data
        )
        print(f'##### Ending script #####')
