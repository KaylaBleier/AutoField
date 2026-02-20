#Opens serial to F9P
# Parses UBX (NAV-PVT) and/or NMEA
# Outputs a single “pose estimate” struct repeatedly:
#       lat, lon, fixType, numSV, hAcc, etc.
# Publishes it to the rest of the rover (queue/socket).