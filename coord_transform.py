#Converts lat/lon → local meters
# Strong recommendation: Local ENU relative to the first good RTK FIX (or your “start” point)
# API like:
#      enu = wgs84_to_enu(lat, lon, origin_lat, origin_lon)
#      returns x_east_m, y_north_m

