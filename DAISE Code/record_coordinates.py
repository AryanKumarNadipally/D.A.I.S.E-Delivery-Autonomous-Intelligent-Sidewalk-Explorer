import time
import serial
import csv
import adafruit_gps
import sys
import select

uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)

gps = adafruit_gps.GPS(uart, debug=False)

gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")

gps.send_command(b"PMTK220,1000")

csv_file = open('gps_coordinates.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['timestamp', 'latitude', 'longitude'])

print("Press 'a' to start recording and 'z' to stop recording.")

recording = False

def check_keypress():

    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None

try:
    while True:

        gps.update()

        key = check_keypress()
        if key:
            if key.lower() == 'a':
                recording = True
                print("Recording started...")
            elif key.lower() == 'z':
                recording = False
                print("Recording stopped.")
                break

        if recording and gps.has_fix:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
            latitude = gps.latitude
            longitude = gps.longitude
            csv_writer.writerow([timestamp, latitude, longitude])
            print(f"Recorded: {timestamp}, {latitude}, {longitude}")

        time.sleep(0.5)

finally:

    csv_file.close()
    print("CSV file closed.")
