std_msgs/Header header

uint32 itow                 # GPS Millisecond time of week [ms]
uint8 num_sv                # Number of satellites used in navigation solution
uint8 position_rtk_status   # RTK fix status: 0=No RTK, 1=Float, 2=Fixed
float64 longitude           # High-precision longitude [deg]
float64 latitude            # High-precision latitude [deg]
float64 utm_easting         # UTM easting coordinate [m]
float64 utm_northing        # UTM northing coordinate [m]
float64 height              # Height above ellipsoid [m]
uint32 h_acc                # Horizontal Accuracy Estimate [mm]
uint32 v_acc                # Vertical Accuracy Estimate [mm]

uint8 heading_rtk_status    # 2:fixed, 1:float, 0:no carrier phase solution
float64 heading_deg         # calculate heading using NE component [degree]


