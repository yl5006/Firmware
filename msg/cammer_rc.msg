uint64 timestamp

uint8 RC_INPUT_SOURCE_UNKNOWN = 0
uint8 RC_INPUT_SOURCE_PX4FMU_PPM = 1
uint8 RC_INPUT_SOURCE_PX4IO_PPM = 2
uint8 RC_INPUT_SOURCE_PX4IO_SPEKTRUM = 3
uint8 RC_INPUT_SOURCE_PX4IO_SBUS = 4
uint8 RC_INPUT_SOURCE_PX4IO_ST24 = 5
uint8 RC_INPUT_SOURCE_MAVLINK = 6
uint8 RC_INPUT_SOURCE_QURT = 7
uint8 RC_INPUT_SOURCE_PX4FMU_SPEKTRUM = 8
uint8 RC_INPUT_SOURCE_PX4FMU_SBUS = 9
uint8 RC_INPUT_SOURCE_PX4FMU_ST24 = 10
uint8 RC_INPUT_SOURCE_PX4FMU_SUMD = 11
uint8 RC_INPUT_SOURCE_PX4FMU_DSM = 12
uint8 RC_INPUT_SOURCE_PX4IO_SUMD = 13

uint8 RC_INPUT_MAX_CHANNELS = 18 	# Maximum number of R/C input channels in the system. S.Bus has up to 18 channels.

uint64 timestamp_last_signal		# last valid reception time
uint32 channel_count			# number of channels actually being seen
bool rc_failsafe			# explicit failsafe flag: true on TX failure or TX out of range , false otherwise. Only the true state is reliable, as there are some 
uint16[18] values			# measured pulse widths for each of the supported channels
