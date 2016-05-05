# Vending machine

Raplacement for very limited functionality from original controller based on PLC(array).


Ustawienie fuses:

c:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avrdude" -C"C:\Program Files (x86)\Arduino\hardware\tools\avr/etc/avrdude.conf" -v -patmega328P -carduino -PCOM5 -b19200 -U lfuse:w:0xE2:m


Prawidłowe dla Arduino as ISP:

check:
"c:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avrdude" "-Cc:\Program Files (x86)\Arduino\hardware\tools\avr/etc/avrdude.conf" -v -patmega328p -cstk500v1 -PCOM5 -b19200

"c:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avrdude" "-Cc:\Program Files (x86)\Arduino\hardware\tools\avr/etc/avrdude.conf" -v -patmega328p -cstk500v1 -PCOM5 -b19200 -D "-Uflash:w:C:\workspace\arduino\builds\avr_flow\pro_8MHzatmega328/avr_flow.hex:i"
