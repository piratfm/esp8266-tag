# esp8266-tag
Simple WiFi MAC tracking tag.

This software for esp8266 module tracks specific MAC-address existance on the local network and send stats to data collector.


use user_config.h to set needed parameters, like wifi network params,  target mac to search, timing, stats collector keys.

The variables for stats collector will be presented: existance, presence.

existance - array of found hosts, splitted by "|".
presence - counter of found or notfound searched MAC.

Example of FULL SCAN data:

    existance = 03,00:08:01:02:03:04,192.168.0.1|03,00:08:01:02:03:05,192.168.0.2|03,00:08:01:02:03:06,192.168.0.3|03,00:08:01:02:03:07,192.168.0.4|03,00:08:01:02:03:08,192.168.0.6
So you have such sub-arrays:
03,00:08:01:02:03:04,192.168.0.1
03,00:08:01:02:03:05,192.168.0.2
03,00:08:01:02:03:06,192.168.0.3
03,00:08:01:02:03:07,192.168.0.4
03,00:08:01:02:03:08,192.168.0.6

element[1]: first element of sub array - is flag, where bit0 = means arp reply received, bit1 - is icmp ping reply received.
element[2]: MAC-address of alive host.
element[3]: ip-address of alive host.

    presence = 1,51
number[0]: counter of succcessfully founded of searched MAC
number[1]: counter of unsucccessful scans (will be resetted when MAC been found).


Scanning is doing unotil whole sublent scanned or until searched MAC is found.


Example of PARTIAL SCAN data:

    existance = 03,00:08:01:02:03:08,192.168.0.6
    presence = 17,0




