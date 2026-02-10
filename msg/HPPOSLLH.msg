# NAV-HPPOSLLH (0x01 0x14)
# High precision geodetic position solution
#
# This message outputs the Geodetic position in the currently selected ellipsoid.
# The default is the WGS84 Ellipsoid.
#

std_msgs/Header header

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 20

# UBX Protocol Fields
uint8 version           # Message version (0x00 for this version)
uint8 flags             # Additional flags (X1 bitfield)
                        # bit0: invalidLlh - 1 = Invalid lon, lat, height, hMSL, lonHp, latHp, heightHp and hMSLHp
uint32 itow             # GPS Millisecond time of week [ms]
int32 lon               # Longitude [deg * 1e-7]
int32 lat               # Latitude [deg * 1e-7]
int32 height            # Height above ellipsoid [mm]
int32 h_msl             # Height above mean sea level [mm]
int8 lon_hp             # High precision component of longitude [deg * 1e-9]
                        # Must be in the range -99..+99.
                        # Precise longitude in deg * 1e-7 = lon + (lon_hp * 1e-2)
int8 lat_hp             # High precision component of latitude [deg * 1e-9]
                        # Must be in the range -99..+99.
                        # Precise latitude in deg * 1e-7 = lat + (lat_hp * 1e-2)
int8 height_hp          # High precision component of height above ellipsoid [mm * 0.1]
                        # Must be in the range -9..+9.
                        # Precise height in mm = height + (height_hp * 0.1)
int8 h_msl_hp           # High precision component of height above mean sea level [mm * 0.1]
                        # Must be in range -9..+9.
                        # Precise height in mm = h_msl + (h_msl_hp * 0.1)
uint32 h_acc            # Horizontal accuracy estimate [mm * 0.1]
uint32 v_acc            # Vertical accuracy estimate [mm * 0.1]
