# NAV-PVT (0x01 0x07)
# Navigation Position Velocity Time Solution
#
# Note that during a leap second there may be more (or less) than 60 seconds in
# a minute; see the description of leap seconds for details.
#
# This message combines Position, velocity and time solution in LLH,
# including accuracy figures
#
# WARNING: For firmware version 7, this message is a different length.
#

std_msgs/Header header

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 7

# UBX Protocol Fields
uint32 itow             # GPS Millisecond time of week [ms]
uint16 year             # Year (UTC)
uint8 month             # Month, range 1..12 (UTC)
uint8 day               # Day of month, range 1..31 (UTC)
uint8 hour              # Hour of day, range 0..23 (UTC)
uint8 min               # Minute of hour, range 0..59 (UTC)
uint8 sec               # Seconds of minute, range 0..60 (UTC)
uint8 valid             # Validity flags (X1 bitfield)
                        # bit0: validDate - 1 = valid UTC Date
                        # bit1: validTime - 1 = valid UTC Time of Day
                        # bit2: fullyResolved - 1 = UTC time of day has been fully resolved
                        # bit3: validMag - 1 = valid magnetic declination
uint32 t_acc            # Time accuracy estimate [ns]
int32 nano              # Fraction of second, range -1e9..1e9 [ns]
uint8 fix_type          # GNSS fix Type:
                        # 0 = no fix
                        # 1 = dead reckoning only
                        # 2 = 2D-fix
                        # 3 = 3D-fix
                        # 4 = GNSS + dead reckoning combined
                        # 5 = time only fix
uint8 flags             # Fix status flags (X1 bitfield)
                        # bit0: gnssFixOK - 1 = valid fix (within DOP & accuracy masks)
                        # bit1: diffSoln - 1 = differential corrections applied
                        # bits4..2: psmState - Power save mode state
                        # bit5: headVehValid - 1 = heading of vehicle is valid
                        # bits7..6: carrSoln - Carrier phase range solution status
                        #   0 = no carrier phase range solution
                        #   1 = carrier phase range solution with floating ambiguities
                        #   2 = carrier phase range solution with fixed ambiguities
uint8 flags2            # Additional flags (X1 bitfield)
                        # bit5: confirmedAvai - 1 = UTC Date and Time of Day validity confirmation available
                        # bit6: confirmedDate - 1 = UTC Date validity could be confirmed
                        # bit7: confirmedTime - 1 = UTC Time of Day could be confirmed
uint8 num_sv            # Number of SVs used in Nav Solution
int32 lon               # Longitude [deg * 1e-7]
int32 lat               # Latitude [deg * 1e-7]
int32 height            # Height above Ellipsoid [mm]
int32 h_msl             # Height above mean sea level [mm]
uint32 h_acc            # Horizontal Accuracy Estimate [mm]
uint32 v_acc            # Vertical Accuracy Estimate [mm]
int32 vel_n             # NED north velocity [mm/s]
int32 vel_e             # NED east velocity [mm/s]
int32 vel_d             # NED down velocity [mm/s]
int32 g_speed           # Ground Speed (2-D) [mm/s]
int32 head_mot          # Heading of motion 2-D [deg * 1e-5]
uint32 s_acc            # Speed Accuracy Estimate [mm/s]
uint32 head_acc         # Heading Accuracy Estimate (both motion & vehicle) [deg * 1e-5]
uint16 p_dop            # Position DOP [1 / 0.01]
uint16 flags3           # Additional flags (X2 bitfield)
                        # bit0: invalidLlh - 1 = Invalid lon, lat, height and hMSL
                        # bits4..1: lastCorrectionAge - Age of the most recently received differential correction
                        # bit13: authTime - 1 = Time is authenticated
                        # bit14: nmaFixStatus - NMA fix status (0 = Not Verified, 1 = Verified)
int32 head_veh          # Heading of vehicle (2-D) [deg * 1e-5], only valid when headVehValid is set
int16 mag_dec           # Magnetic declination [deg * 1e-2], only supported in ADR 4.10 and later
uint16 mag_acc          # Magnetic declination accuracy [deg * 1e-2], only supported in ADR 4.10 and later

# Custom calculated fields (not in UBX spec, for application use)
uint8 fix_status        # 2:fixed, 1:float, 0:no carrier phase solution
float64 lon_deg         # Longitude converted to [deg]
float64 lat_deg         # Latitude converted to [deg]
