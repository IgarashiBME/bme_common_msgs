# NAV-RELPOSNED (0x01 0x3C)
# Relative positioning information in NED frame
#
# This message contains the relative position vector from the reference station
# to the rover, including accuracy figures, in the local topological system
# defined at the reference station.
#

std_msgs/Header header

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 60

# UBX Protocol Fields
uint8 version           # Message version (0x01 for this version)
uint16 ref_station_id   # Reference station ID. Must be in the range 0..4095.
uint32 itow             # GPS Millisecond time of week [ms]
int32 rel_pos_n         # North component of relative position vector [cm]
int32 rel_pos_e         # East component of relative position vector [cm]
int32 rel_pos_d         # Down component of relative position vector [cm]
int32 rel_pos_length    # Length of the relative position vector [cm]
int32 rel_pos_heading   # Heading of the relative position vector [deg * 1e-5]
int8 rel_pos_hpn        # High-precision North component of relative position vector [mm * 0.1]
                        # Must be in the range -99..+99.
                        # Full North component in cm = rel_pos_n + (rel_pos_hpn * 1e-2)
int8 rel_pos_hpe        # High-precision East component of relative position vector [mm * 0.1]
                        # Must be in the range -99..+99.
                        # Full East component in cm = rel_pos_e + (rel_pos_hpe * 1e-2)
int8 rel_pos_hpd        # High-precision Down component of relative position vector [mm * 0.1]
                        # Must be in the range -99..+99.
                        # Full Down component in cm = rel_pos_d + (rel_pos_hpd * 1e-2)
int8 rel_pos_hp_length  # High-precision component of the length of relative position vector [mm * 0.1]
                        # Must be in the range -99..+99.
                        # Full length in cm = rel_pos_length + (rel_pos_hp_length * 1e-2)
uint32 acc_n            # Accuracy of relative position North component [mm * 0.1]
uint32 acc_e            # Accuracy of relative position East component [mm * 0.1]
uint32 acc_d            # Accuracy of relative position Down component [mm * 0.1]
uint32 acc_length       # Accuracy of length of the relative position vector [mm * 0.1]
uint32 acc_heading      # Accuracy of heading of the relative position vector [deg * 1e-5]
uint32 flags            # Flags (X4 bitfield)
                        # bit0: gnssFixOK - A valid fix (within DOP & accuracy masks)
                        # bit1: diffSoln - 1 if differential corrections were applied
                        # bit2: relPosValid - 1 if relative position components and accuracies are valid
                        # bits4..3: carrSoln - Carrier phase range solution status:
                        #   0 = no carrier phase range solution
                        #   1 = carrier phase range solution with floating ambiguities
                        #   2 = carrier phase range solution with fixed ambiguities
                        # bit5: isMoving - 1 if the receiver is operating in moving base mode
                        # bit6: refPosMiss - 1 if extrapolated reference position was used
                        # bit7: refObsMiss - 1 if extrapolated reference observations were used
                        # bit8: relPosHeadingValid - 1 if relPosHeading is valid
                        # bit9: relPosNormalized - 1 if relative position components are normalized

# Custom calculated fields (not in UBX spec, for application use)
uint8 fix_status        # 2:fixed, 1:float, 0:no carrier phase solution
float64 heading_rad     # calculate heading using NE component [radian]
float64 heading_deg     # calculate heading using NE component [degree]
float64 qgc_heading     # right-handed, Z-down, X-front, Y-right [radian]
