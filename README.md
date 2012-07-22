PMU Firmware
============

Power Button, soft shutdown, soft wake up controller

Based on ATtiny13A

Big Issue:

ATtiny is not instantly on, the board would be very short turned on before the ATtiny13A got itself initalized and pull down the shutdown pin as default. 
--- Update: If we set the fuse to "disable reset", the issue is gone!
