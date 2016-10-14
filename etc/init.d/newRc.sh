#
# PX4FMU startup and command generator
#
# ceated by duyong
#
#  NOTE: COMMENT LINES ARE REMOVED BEFORE STORED IN ROMFS.
#
#	Applications
COMMAND_APP=mainapp
export COMMAND_CONFIG=/fs/microsd/etc/init.d/mainapp_rcS
export MODE=autostart

param set SYS_AUTOCONFIG 1
#
#
# Start CDC/ACM serial driver
#
#sercon
#
# Default to auto-start mode.
#
export MODE="autostart"
export TUNE_ERR="ML<<CP4CP4CP4CP4CP4"
export LOG_FILE=/fs/microsd/bootlog.txt
echo "" > $COMMAND_CONFIG
#
# Try to mount the microSD card.
#
# REBOOTWORK this needs to start after the flight control loop
#

mkdir -p /fs/microsd/
mkdir -p /fs/microsd/etc/init.d/

if false; then
if mount -t vfat /dev/mmcsd0 /fs/microsd
then
	# Start playing the startup tune
	echo "tone_alarm start" >> $COMMAND_CONFIG
else
	echo "tone_alarm MBAGP" >> $COMMAND_CONFIG 
	if mkfatfs /dev/mmcsd0
	then
		if mount -t vfat /dev/mmcsd0 /fs/microsd
		then
			echo "[i] microSD card formatted"
		else
			echo "[i] format failed"
			echo "tone_alarm MNBG" >> $COMMAND_CONFIG 
			export LOG_FILE=/dev/null
		fi
	else
		export LOG_FILE=/dev/null
	fi
fi
fi

	echo "tone_alarm start" >> $COMMAND_CONFIG
#
# Look for an init script on the microSD card.
# Disable autostart if the script found.
#
:<<!EOF!
export FRC=/fs/microsd/etc/rc.txt
if [ -f $FRC ]
then
	echo "[i] Executing script: $FRC"
	sh $FRC
	export MODE=custom
fi
unset FRC
!EOF!


if [ "$MODE"x = "autostart"x ]
then

	#
	# Start the ORB (first app to start)
	#
	echo "uorb start" >> $COMMAND_CONFIG 

	#
	# Load parameters
	#

export PARAM_FILE=/fs/microsd/params
#	if mtd start
#	then
#		export PARAM_FILE=/fs/mtd_params
#	fi

	echo "param select $PARAM_FILE" >> $COMMAND_CONFIG
	echo "param load" >> $COMMAND_CONFIG
#	if param load
#	then
#	else
#		if param reset
#		then
#		fi
#	fi
	#
	# Start system state indicator
	#
	echo "rgbled start -b 2" >> $COMMAND_CONFIG
#	if rgbled start
#	then
#	else
#		if blinkm start
#		then
#			blinkm systemstate
#		fi
#	fi

	# Currently unused, but might be useful down the road
	#if pca8574 start
	#then
	#fi

	#
	# Set AUTOCNF flag to use it in AUTOSTART scripts
	#
	if param compare SYS_AUTOCONFIG 1
	then
		# Wipe out params except RC*
		#echo "param reset_nostart RC*" >> $COMMAND_CONFIG
		export AUTOCNF=yes
	else
		export AUTOCNF=no
	fi

	#
	# Set default values
	#
	export HIL=no
	export VEHICLE_TYPE=none
	export MIXER=none
	export MIXER_AUX=none
	export OUTPUT_MODE=none
	export PWM_OUT=none
	export PWM_RATE=none
	export PWM_DISARMED=none
	export PWM_MIN=none
	export PWM_MAX=none
	export PWM_AUX_OUT=none
	export PWM_AUX_RATE=none
	export PWM_ACHDIS=none
	export PWM_AUX_DISARMED=none
	export PWM_AUX_MIN=none
	export PWM_AUX_MAX=none
	export FAILSAFE_AUX=none
	export MK_MODE=none
	export FMU_MODE=pwm
	export AUX_MODE=pwm
	export MAVLINK_F=default
	export EXIT_ON_END=no
	export MAV_TYPE=none
	export FAILSAFE=none
	export USE_IO=yes

	#
	# Set USE_IO flag
	#
	if param compare SYS_USE_IO 1
	then
		if ver hwcmp PX4FMU_V4
		then
			export USE_IO=no
		fi

		if ver hwcmp MINDPX_V2
		then
			export USE_IO=no
		fi
		
		if ver hwcmp LINUXTEST
		then
			export USE_IO=no
  	fi
	else
		export USE_IO=no
	fi


	

#	# should set to 0.8 for mindpx-v2 borad.
	if param compare INAV_LIDAR_ERR 0.5
	then
		if ver hwcmp MINDPX_V2
		then
			param set INAV_LIDAR_ERR 0.8
			param save
		fi
	fi

	#
	# Set parameters and env variables for selected AUTOSTART
	#
	if param compare SYS_AUTOSTART 0
	then
		echo "[i] No autostart"
	else
		if ver hwcmp LINUXTEST
		then
			source /fs/microsd/etc/init.d/rc.autostart
			echo "############ $VEHICLE_TYPE ##########"
		else
			sh /etc/init.d/rc.autostart
		fi
	fi
	unset MODE
	#echo "auto start /etc/init.d/rc.autostart"



	#
	# Wipe incompatible settings for boards not having two outputs
	if ver hwcmp PX4FMU_V4
	then
		export MIXER_AUX=none
	fi

	#
	# Override parameters from user configuration file
	#
	export FCONFIG=/fs/microsd/etc/config.txt
	if [ -f $FCONFIG ]
	then
		echo "[i] Custom: $FCONFIG"
		sh $FCONFIG
	fi
	unset FCONFIG

	#
	# If autoconfig parameter was set, reset it and save parameters
	#
	if [ $AUTOCNF = yes ]
	then
		#echo "param set SYS_AUTOCONFIG 0" >> $COMMAND_CONFIG
		echo "param save" >> $COMMAND_CONFIG
	fi
	unset AUTOCNF

	echo "param save"

	export IO_PRESENT=no

#	if [ $USE_IO = yes ]
#	then
		#
		# Check if PX4IO present and update firmware if needed
		#
#		if [ -f /etc/extras/px4io-v2.bin ]
#		then
#			export IO_FILE=/etc/extras/px4io-v2.bin
#		else
#			export IO_FILE=/etc/extras/px4io-v1.bin
#		fi

#		if px4io checkcrc ${IO_FILE}
#		then
#			echo "PX4IO CRC OK" >> $LOG_FILE

#			export IO_PRESENT=yes
#		else
#			tone_alarm MLL32CP8MB

#			if px4io start
#			then
#				# try to safe px4 io so motor outputs dont go crazy
#				if px4io safety_on
#				then
#					# success! no-op
#				else
#					# px4io did not respond to the safety command
#					px4io stop
#				fi
#			fi

#			if px4io forceupdate 14662 ${IO_FILE}
#			then
#				usleep 500000
#				if px4io checkcrc $IO_FILE
#				then
#					echo "PX4IO CRC OK after updating" >> $LOG_FILE
#					tone_alarm MLL8CDE

#					export IO_PRESENT=yes
#				else
#					echo "PX4IO update failed" >> $LOG_FILE
#					tone_alarm $TUNE_ERR
#				fi
#			else
#				echo "PX4IO update failed" >> $LOG_FILE
#				tone_alarm $TUNE_ERR
#			fi
#		fi
		unset IO_FILE

#		if [ $IO_PRESENT = no ]
#		then
#			echo "ERROR: PX4IO not found" >> $LOG_FILE
#			tone_alarm $TUNE_ERR
#		fi
#	fi

	#
	# Set default output if not set
	#
	if [ $OUTPUT_MODE = none ]
	then
		if [ $USE_IO = yes ]
		then
			export OUTPUT_MODE=io
		else
			export OUTPUT_MODE=fmu
		fi
	fi

	if [ $OUTPUT_MODE = io -a $IO_PRESENT != yes ]
	then
		# Need IO for output but it not present, disable output
		export OUTPUT_MODE=none

		# Avoid using ttyS0 for MAVLink on FMUv1
#		if ver hwcmp PX4FMU_V1
#		then
			export FMU_MODE=serial
#		fi
	fi

	if [ $OUTPUT_MODE = ardrone ]
	then
		export FMU_MODE=gpio_serial
	fi

	if [ $HIL = yes ]
	then
		export OUTPUT_MODE=hil
#		if ver hwcmp PX4FMU_V1
#		then
#			export FMU_MODE=serial
#		fi
		unset HIL
	else
		unset HIL
		echo "gps start -d /dev/ttyS2" >> $COMMAND_CONFIG 
	fi

	# waypoint storage
	# REBOOTWORK this needs to start in parallel
#	if dataman start
#	then
#	fi

	echo "dataman start" >> $COMMAND_CONFIG 
	#
	# Sensors System (start before Commander so Preflight checks are properly run)
	#
	echo "/fs/microsd/etc/init.d/rc.sensors"
	cat /fs/microsd/etc/init.d/rc.sensors
	source /fs/microsd/etc/init.d/rc.sensors 

	# Needs to be this early for in-air-restarts
	if [ $OUTPUT_MODE = hil ]
	then
	    echo "commander start -hil"  >> $COMMAND_CONFIG 
	else
	    echo "commander start" >> $COMMAND_CONFIG 
	fi

	#
	# Start CPU load monitor
	#
	echo "load_mon start" >> $COMMAND_CONFIG

	#
	# Start primary output
	#
	export TTYS1_BUSY=no

	#
	# Check if UAVCAN is enabled, default to it for ESCs
	#
#	if param greater UAVCAN_ENABLE 2MIXER_AUX
#	then
#		export OUTPUT_MODE=uavcan_esc
#	fi
:<<!EOF!
	# If OUTPUT_MODE = none then something is wrong with setup and we shouldn't try to enable output
#	if [ $OUTPUT_MODE != none ]
#	then
#		if [ $OUTPUT_MODE = uavcan_esc ]
#		then
#			if param compare UAVCAN_ENABLE 0
#			then
#				echo "[i] OVERRIDING UAVCAN_ENABLE = 1"
#				param set UAVCAN_ENABLE 1
#			fi
#		fi

#		if [ $OUTPUT_MODE = io -o $OUTPUT_MODE = uavcan_esc ]
#		then
#			if px4io start
#			then
#				sh /etc/init.d/rc.io
#			else
#				echo "PX4IO start failed" >> $LOG_FILE
#				tone_alarm $TUNE_ERR
#			fi
#		fi
		if [ $OUTPUT_MODE = fmu -o $OUTPUT_MODE = ardrone ]
		then
			if fmu mode_$FMU_MODE
			then
			else
				echo "FMU start failed" >> $LOG_FILE
				tone_alarm $TUNE_ERR
			fi

			if ver hwcmp PX4FMU_V1
			then
				if [ $FMU_MODE = pwm -o $FMU_MODE = gpio ]
				then
					export TTYS1_BUSY=yes
				fi
				if [ $FMU_MODE = pwm_gpio -o $OUTPUT_MODE = ardrone ]
				then
					export TTYS1_BUSY=yes
				fi
			fi
		fi

		if [ $OUTPUT_MODE = mkblctrl ]
		then
			export MKBLCTRL_ARG=""
			if [ $MKBLCTRL_MODE = x ]
			then
				export MKBLCTRL_ARG="-mkmode x"
			fi
			if [ $MKBLCTRL_MODE = + ]
			then
				export MKBLCTRL_ARG="-mkmode +"
			fi

			if mkblctrl $MKBLCTRL_ARG
			then
			else
				echo "ERROR: MK start failed" >> $LOG_FILE
				tone_alarm $TUNE_ERR
			fi
			unset MKBLCTRL_ARG
		fi
		unset MK_MODE

		if [ $OUTPUT_MODE = hil ]
		then
			if pwm_out_sim mode_port2_pwm8
			then
			else
				echo "PWM SIM start failed" >> $LOG_FILE
				tone_alarm $TUNE_ERR
			fi
		fi

		#
		# Start IO or FMU for RC PPM input if needed
		#
		if [ $IO_PRESENT = yes ]
		then
			if [ $OUTPUT_MODE != io ]
			then
				if px4io start
				then
					sh /etc/init.d/rc.io
				else
					echo "PX4IO start failed" >> $LOG_FILE
					tone_alarm $TUNE_ERR
				fi
			fi
		else
			if [ $OUTPUT_MODE != fmu -a $OUTPUT_MODE != ardrone ]
			then
				if fmu mode_$FMU_MODE
				then
				else
					echo "FMU mode_$FMU_MODE start failed" >> $LOG_FILE
					tone_alarm $TUNE_ERR
				fi

				if ver hwcmp PX4FMU_V1
				then
					if [ $FMU_MODE = pwm -o $FMU_MODE = gpio ]
					then
						export TTYS1_BUSY=yes
					fi
					if [ $FMU_MODE = pwm_gpio -o $OUTPUT_MODE = ardrone ]
					then
						export TTYS1_BUSY=yes
					fi
				fi
			fi
		fi
	fi

	if [ $MAVLINK_F = default ]
	then
		# Normal mode, use baudrate 57600 (default) and data rate 1000 bytes/s
		if [ $TTYS1_BUSY = yes ]
		then
			# Start MAVLink on ttyS0, because FMU ttyS1 pins configured as something else
			export MAVLINK_F="-r 20000 -d /dev/ttyS0"

			# Exit from nsh to free port for mavlink
			export EXIT_ON_END=yes
		else
			export MAVLINK_F="-r 20000"
			# Avoid using ttyS1 for MAVLink on FMUv4
			if ver hwcmp PX4FMU_V4
			then
				export MAVLINK_F="-r 1200 -d /dev/ttyS1"
				# Start MAVLink on Wifi (ESP8266 port)
				mavlink start -r 20000 -m config -b 921600 -d /dev/ttyS0
			fi
		fi
	fi
 
	mavlink start $MAVLINK_F
	
!EOF!
	unset MAVLINK_F

echo "fmu mode_pwm" >> $COMMAND_CONFIG 
echo "mavlink start -r 40000 -d /dev/ttyS4" >> $COMMAND_CONFIG 
#echo "mavlink start -u 14556 -t 192.168.103.123 -r 800000" >> $COMMAND_CONFIG 
  
	#
	# MAVLink onboard / TELEM2
	#
	if ver hwcmp PX4FMU_V1
	then
#	:
#	else
		# XXX We need a better way for runtime eval of shell variables,
		# but this works for now
		if param compare SYS_COMPANION 10
		then
			echo "frsky_telemetry start -d /dev/ttyO5" >> $COMMAND_CONFIG
		fi
		if param compare SYS_COMPANION 921600
		then
			echo "mavlink start -d /dev/ttyO5 -b 921600 -m onboard -r 80000 -x" >> $COMMAND_CONFIG
		fi
		if param compare SYS_COMPANION 57600
		then
			echo "mavlink start -d /dev/ttyO5 -r 20000 -b 57600" >> $COMMAND_CONFIG
		fi
		if param compare SYS_COMPANION 157600
		then
			echo "mavlink start -d /dev/ttyO5 -b 57600 -m osd -r 1000"  >> $COMMAND_CONFIG
		fi
		if param compare SYS_COMPANION 257600
		then
			echo "mavlink start -d /dev/ttyO5 -b 57600 -m magic -r 5000 -x"  >> $COMMAND_CONFIG
		fi
		if param compare SYS_COMPANION 357600
		then
			echo "mavlink start -d /dev/ttyO5 -b 57600 -r 1000"  >> $COMMAND_CONFIG
		fi
		if param compare SYS_COMPANION 1921600
		then
			echo "mavlink start -d /dev/ttyO5 -b 921600 -r 20000"  >> $COMMAND_CONFIG
		fi
		# Sensors on the PWM interface bank
		# clear pins 5 and 6
		if param compare SENS_EN_LL40LS 1
		then
			export AUX_MODE=pwm4
		fi
		if param greater TRIG_MODE 0
		then
			# Get FMU driver out of the way
			export MIXER_AUX=none
			export AUX_MODE=none
			#echo "camera_trigger start"  >> $COMMAND_CONFIG
		fi
	fi


	#
	# Starting stuff according to UAVCAN_ENABLE value
	#
#	if param greater UAVCAN_ENABLE 0
#	then
#		if uavcan start
#		then
#		else
#			tone_alarm $TUNE_ERR
#		fi
#	fi

#	if param greater UAVCAN_ENABLE 1
#	then
#		if uavcan start fw
#		then
#		else
#			tone_alarm $TUNE_ERR
#		fi
#	fi

	#
	# Optional drivers
	#

	# Sensors on the PWM interface bank
#	if param compare SENS_EN_LL40LS 1
#	then
#		if pwm_input start
#		then
#			if ll40ls start pwm
#			then
#			fi
#		fi
#	fi

	# sf0x lidar sensor
#	if param compare SENS_EN_SF0X 1
#	then
#		sf0x start
#	fi

#	if ver hwcmp PX4FMU_V4
#	then
#		frsky_telemetry start -d /dev/ttyS6
#	fi

	if ver hwcmp PX4FMU_V2
	then
		# Check for flow sensor - as it is a background task, launch it last
		px4flow start &
	fi
	
	if ver hwcmp LINUXTEST
	then
#	echo "px4flow start" >> $COMMAND_CONFIG 
	:
	fi
	
	if ver hwcmp MINDPX_V2
	then
		#mindxp also need flow
		px4flow start &
	fi

	# Start USB shell if no microSD present, MAVLink else
	if [ $LOG_FILE = /dev/null ]
	then
		# Try to get an USB console
		nshterm /dev/ttyACM0 &
	else
#		mavlink start -r 800000 -d /dev/ttyACM0 -m config -x
	:
	fi



:<<!EOF!
	#
	# Logging
	#
	if ver hwcmp PX4FMU_V1
	then
		if sdlog2 start -r 30 -a -b 2 -t
		then
		fi
	else
		# check if we should increase logging rate for ekf2 replay message logging
		if param greater EKF2_REC_RPL 0
		then
			if param compare SYS_LOGGER 0
			then
				if sdlog2 start -r 500 -e -b 18 -t
				then
				fi
			else
				if logger start -r 500
				then
				fi
			fi
		else
			if param compare SYS_LOGGER 0
			then
				if sdlog2 start -r 100 -a -b 9 -t
				then
				fi
			else
				if logger start -b 9
				then
				fi
			fi
		fi
	fi
!EOF!
	echo "sdlog2 start -r 3 -e -t -a -b 200" >> $COMMAND_CONFIG 

	#
	# Start up ARDrone Motor interface
	#
	if [ $OUTPUT_MODE = ardrone ]
	then
		ardrone_interface start -d /dev/ttyS1
	fi

	#
	# Fixed wing setup
	#
	if [ $VEHICLE_TYPE = fw ]
	then
		echo "FIXED WING"

		if [ $MIXER = none ]
		then
			# Set default mixer for fixed wing if not defined
			export MIXER=AERT
		fi

		if [ $MAV_TYPE = none ]
		then
			# Use MAV_TYPE = 1 (fixed wing) if not defined
			export MAV_TYPE=1
		fi
		param set MAV_TYPE $MAV_TYPE
		echo "param set MAV_TYPE $MAV_TYPE" >> $COMMAND_CONFIG

		# Load mixer and configure outputs
		source /fs/microsd/etc/init.d/rc.interface

		# Start standard fixedwing apps
		source /fs/microsd/etc/init.d/rc.fw_apps
	fi

	#
	# Multicopters setup
	#
	if [ $VEHICLE_TYPE = mc ]
	then
		echo "MULTICOPTER"

		if [ $MIXER = none ]
		then
			echo "Mixer undefined"
		fi

		if [ $MAV_TYPE = none ]
		then
			# Use mixer to detect vehicle type
			if [ $MIXER = quad_x -o $MIXER = quad_+ ]
			then
				export MAV_TYPE=2
			fi
			if [ $MIXER = quad_w -o $MIXER = sk450_deadcat ]
			then
				export MAV_TYPE=2
			fi
			if [ $MIXER = quad_h ]
			then
				export MAV_TYPE=2
			fi
			if [ $MIXER = tri_y_yaw- -o $MIXER = tri_y_yaw+ ]
			then
				export MAV_TYPE=15
			fi
			if [ $MIXER = hexa_x -o $MIXER = hexa_+ ]
			then
				export MAV_TYPE=13
			fi
			if [ $MIXER = hexa_cox ]
			then
				export MAV_TYPE=13
			fi
			if [ $MIXER = octo_x -o $MIXER = octo_+ ]
			then
				export MAV_TYPE=14
			fi
			if [ $MIXER = octo_cox -o $MIXER = octo_cox_w ]
			then
				export MAV_TYPE=14
			fi
			if [ $MIXER = coax ]
			then
				export MAV_TYPE=3
			fi
		fi

		# Still no MAV_TYPE found
		if [ $MAV_TYPE = none ]
		then
			echo "Unknown MAV_TYPE"
			echo "param set MAV_TYPE 2" >> $COMMAND_CONFIG
			param set MAV_TYPE 2
		else
			echo "param set MAV_TYPE $MAV_TYPE" >> $COMMAND_CONFIG
			param set MAV_TYPE $MAV_TYPE
		fi

		# Load mixer and configure outputs
		#sh /etc/init.d/rc.interface
		source /fs/microsd/etc/init.d/rc.interface
		# Start standard multicopter apps
		#sh /etc/init.d/rc.mc_apps
		source /fs/microsd/etc/init.d/rc.mc_apps
	fi

	#
	# VTOL setup
	#
	if [ $VEHICLE_TYPE = vtol ]
	then
		echo "VTOL"

		if [ $MIXER = none ]
		then
			echo "VTOL mixer undefined"
		fi

		if [ $MAV_TYPE = none ]
		then
			# Use mixer to detect vehicle type
			if [ $MIXER = caipirinha_vtol ]
			then
				export MAV_TYPE=19
			fi
			if [ $MIXER = firefly6 ]
			then
				export MAV_TYPE=21
			fi
			if [ $MIXER = quad_x_pusher_vtol ]
			then
				export MAV_TYPE=22
			fi
		fi

		# Still no MAV_TYPE found
		if [ $MAV_TYPE = none ]
		then
			echo "Unknown MAV_TYPE"
			param set MAV_TYPE 19
			echo "param set MAV_TYPE 19" >> $COMMAND_CONFIG
		else
			param set MAV_TYPE $MAV_TYPE
			echo "param set MAV_TYPE $MAV_TYPE" >> $COMMAND_CONFIG
		fi

		# Load mixer and configure outputs
		source /fs/microsd/etc/init.d/rc.interface

		# Start standard vtol apps
		source /fs/microsd/etc/init.d/rc.vtol_apps
	fi

	#
	# Rover setup
	#
	if [ $VEHICLE_TYPE = rover ]
	then
		# 10 is MAV_TYPE_GROUND_ROVER
		export MAV_TYPE=10

		# Load mixer and configure outputs
		source /fs/microsd/etc/init.d/rc.interface

		# Start standard rover apps
		source /fs/microsd/etc/init.d/rc.axialracing_ax10_apps

		echo "param set MAV_TYPE 10" >> $COMMAND_CONFIG
		param set MAV_TYPE 10
	fi

	#
	# For snapdragon, we need a passthrough mode
	# Do not run any mavlink instances since we need the serial port for
	# communication with Snapdragon.
	#
	if [ $VEHICLE_TYPE = passthrough ]
	then
		mavlink stop-all
		commander stop

		# Stop multicopter attitude controller if it is running, the controls come
		# from Snapdragon.
		#if mc_att_control stop
		#then
		#fi

		# Start snapdragon interface on serial port.
		if ver hwcmp PX4FMU_V2
		then
			# On Pixfalcon use the standard telemetry port (Telem 1).
			snapdragon_rc_pwm start -d /dev/ttyS1
			px4io start
		fi

		if ver hwcmp PX4FMU_V4
		then
			# On Pixracer use Telem 2 port (TL2).
			snapdragon_rc_pwm start -d /dev/ttyS2
			fmu mode_pwm4
		fi

		pwm failsafe -c 1234 -p 900
		pwm disarmed -c 1234 -p 900

		# Arm straightaway.
		pwm arm
		# Use 400 Hz PWM on all channels.
		pwm rate -a -r 400
	fi

	unset MIXER
	unset MAV_TYPE
	unset OUTPUT_MODE

	#
	# Start the navigator
	#
	echo "navigator start" >> $COMMAND_CONFIG 

	if param greater THROW_ENABLE 0
		then
			echo "throwtofly start" >> $COMMAND_CONFIG
	fi
	#
	# Generic setup (autostart ID not found)
	#
	if [ $VEHICLE_TYPE = none ]
	then
		echo "[i] No autostart ID found"
	fi

	# Start any custom addons
	export FEXTRAS=/fs/microsd/etc/extras.txt
	if [ -f $FEXTRAS ]
	then
		echo "[i] Addons script: $FEXTRAS"
		sh $FEXTRAS
	fi
	unset FEXTRAS

	# Run no SD alarm
	if [ $LOG_FILE = /dev/null ]
	then
		# Play SOS
		# tone_alarm error
		echo "no SD card"
	fi

# End of autostart
fi
       
# There is no further script processing, so we can free some RAM
# XXX potentially unset all script variables.
unset TUNE_ERR

# Boot is complete, inform MAVLink app(s) that the system is now fully up and running
echo "mavlink boot_complete" >>  $COMMAND_CONFIG 

if [ $EXIT_ON_END = yes ]
then
	echo "NSH EXIT"
	exit
fi
unset EXIT_ON_END
#unset COMMAND_CONFIG

#/home/opt/mainapp /fs/microsd/etc/init.d/mainapp_rcS

