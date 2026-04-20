import gpiozero


relay = gpiozero.OutputDevice(17, active_high=False, initial_value=False)

while True:
    if relay.is_active:
        relay.off()
        print("Relay is OFF")