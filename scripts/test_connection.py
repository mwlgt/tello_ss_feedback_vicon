from djitellopy import Tello
import time

# Connect via ip
tello = Tello('192.168.1.103')

# tello = Tello()

tello.connect()

#Get battery percentage
battery = tello.get_battery()


# tello.takeoff()

# # wait 3 seconds
# time.sleep(3)

# tello.land()

# tello.connect_to_wifi("Lab_Robotica","coke.fanta.sprite")

print(battery)

tello.end()


