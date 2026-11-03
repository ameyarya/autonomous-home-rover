# Rover remote boot. The stock boot is saved as /boot_stock.py: power on with the left
# shoulder 3-position switch away from the middle to start the stock Mini-T remote instead.
import machine
import rover_remote

_switch = 4095 - machine.ADC(machine.Pin(rover_remote.PIN_SWITCH), atten=machine.ADC.ATTN_11DB).read()
if rover_remote.switch_centred(_switch):
    rover_remote.main()
else:
    exec(open('/boot_stock.py').read())
