# Rover remote boot. The stock boot is saved as /boot_stock.py: hold the top-left button while
# powering on to start the stock Mini-T remote instead.
import machine
import rover_remote

if machine.Pin(rover_remote.PIN_BUTTON, machine.Pin.IN, machine.Pin.PULL_UP).value() == 0:
    exec(open('/boot_stock.py').read())
else:
    rover_remote.main()
