This is rework of https://github.com/andrewgabler/VolvoDIMLib volvdimlibrary


reworked things/added

1. added high beam
2. added brake light
3. added new blinker system more stable and simple
4. added cruise light
5. added oil lamp light
6. added battery light
7. added blinking abs and tc light
8. added solid abs and tc light
9. added fog light
10. removed some things like error messages
11. modified setspeed to work with cruise light
12. modified defaultdata speed/keepalive byte 2 from 0x00 to 0x40 so cruise light works
13. removed other brightness things and maded one totalbrigness
14. removed genblinking no need because of the new blinker system
15. modified setTime to work with battery light so it does not conflict
16. modified clocktodecimal void to work with 24 hour too
