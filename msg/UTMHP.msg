# UTMHP - UTM High Precision Position
# Custom message for UTM coordinate output with high-precision GNSS data

std_msgs/Header header

uint32 itow             # GPS Millisecond time of week [ms]
uint8 num_sv            # Number of satellites used in navigation solution
uint8 fix_status        # RTK fix status: 0=No RTK, 1=Float, 2=Fixed
float64 lon_hp          # High-precision longitude [deg] (from UBX-NAV-HPPOSLLH)
float64 lat_hp          # High-precision latitude [deg] (from UBX-NAV-HPPOSLLH)
float64 utm_easting     # UTM easting coordinate [m]
float64 utm_northing    # UTM northing coordinate [m]
float64 height          # Height above ellipsoid [m] (from UBX-NAV-HPPOSLLH)
uint32 h_acc            # Horizontal Accuracy Estimate [mm]
uint32 v_acc            # Vertical Accuracy Estimate [mm]
