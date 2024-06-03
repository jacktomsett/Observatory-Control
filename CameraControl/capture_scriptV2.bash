#!/bin/bash

#Set some default variables
INFOFILENAME=$(date --iso-8601=seconds)_capture_settings.txt
NPFCALC=/home/jack/camera_control/NPF


#Parse input
#ISO=$1
#FNUMBER=$1

while getopts "i:f:e:t:hm" option;
do
	case $option in
		i)
			#Here we should check that $OPTARG is a number
			ISO=$OPTARG
			;;
		f)
			#Again, optarg should be a number
			FNUMBER=$OPTARG
			;;
		e)
			EXPOSURE=$OPTARG
			;;
		t)
			echo "Target mode selected. Target requested: " $OPTARG
			echo "Catalog will be searched for target and time and position data will be pulled from the camera"
			TARGET=$OPTARG
			;;
		h)
			echo "Display help text here"
			exit 1
			;;
		m)
			echo "Skipping setting default settings for autofocus, capture mode and image quaility"
			;;
		?)	echo "Unrecognised option provided"
			#Display usage here
			;;
	esac
done

## The final parameter should be the argument that specified the number of photos to take
# I want to find a robust way of error checking this, but for now it will just be assumed
# that the last parameter provided is this argument and not one of the options or its 
# argument
NPHOTOS=${@: -1}
#Check that an integer has been provided
intexpr='^[0-9]+$'
if ! [[ $NPHOTOS =~ $intexpr ]];
then
	echo "ERROR: Number of photos requested is not an integer"
	echo Requested $NPHOTOS photos.
	#Display usage
	exit 1
fi
echo Sequence of $NPHOTOS photos requested.


## Check if any incompatible options have been specified. Incompatible
# options include:
#	Target and exposure
#	Settings file and exposure, iso or f-number settings
if [[ ! -z ${TARGET+x} && ! -z ${EXPOSURE+x} ]];
then
	echo "Cannot specify target and exposure simultaneously"
	#Display usage here
	exit 1
fi



#Variable to store wether option arguments are acceptable
OPTSVALID=true

### Connect to camera...
gphoto2 --quiet --auto-detect


### Ensure camera is in manual mode:
CAMERAMODE=$(gphoto2 --quiet --get-config=/main/capturesettings/expprogram)
CAMERAMODE=$( echo "$CAMERAMODE" | sed -n 4p )
if [ "$CAMERAMODE" != "Current: M" ];
then
	echo "Error, camera must be in manual mode"
	exit 1
fi



### Set some default camera settings
# image storage location must be done regardless otherwise the camera
# wont take pictures and it cannot be changed from the camera itself.
# The rest almost always want to be on but can be skipped with the
# manual flag

# Set image storage location to camera memory card
# If this is not done it will try to store images to the cameras RAM
gphoto2 --quiet --set-config /main/settings/capturetarget=1

if [ ! -z ${MANUAl+x} ];
then
	# Ensure camera image quality is set to RAW
	gphoto2 --quiet --set-config /main/capturesettings/imagequality=3

	# Ensure automatic white balancing is turned off
	gphoto2 --quiet --set-config /main/imgsettings/whitebalance=1

	# Ensure automatic ISO is turned off
	gphoto2 --quiet --set-config /main/imgsettings/autoiso=1

	# Disable long exposure noise reduction
	gphoto2 --quiet --set-config /main/capturesettings/longexpnr=1

	# Turn off assist light
	gphoto2 --quiet --set-config /main/capturesettings/assistlight=1

	# Ensure flash is disabled
	#gphoto2 --quiet --set-config /main/capturesettings/flashmode=0

	# Disable autofocus     TODO:: Develop our own autofocus mechanism
	gphoto2 --quiet --set-config /main/capturesettings/focusmode2=4

	# Put camera into single shot mode
	gphoto2 --quiet --set-config /main/capturesettings/capturemode=0


fi



### Apply any capture settings specified in options
# Multiple different options can specify the same settings, so we check
# wether environment variables are checked

# Exposure
# This one is slightly different than the others because it determines
# if we are letting the camera time the exposures or if the script will
if [ -z ${EXPOSURE+x} ];
then
	EXPOSUREMODE=camera
else
	EXPOSUREMODE=script
	gphoto2 --quiet  --set-config /main/capturesettings/shutterspeed=52
fi

# ISO
if [ ! -z ${ISO+x} ];
then
	## Get list of possible ISO values from camera
	ISOOPTIONS=($(gphoto2 --quiet --get-config=/main/imgsettings/iso | sed '1,4 d' | sed 's/.\{10\}//'))
	unset ISOOPTIONS[-1]
	## Search list for option that matches reuested
	count=0
	for OPTION in ${ISOOPTIONS[@]}
	do
		if [ $ISO -eq $OPTION ];
		then
			SELECT=$count
		fi
		count=$(( count + 1 ))
	done
	unset count

	## Check if requested option was found, if it was then set it
	##  if not print error
	if [ -z ${SELECT+x} ];
	then
		echo "Requested ISO option not available. Possible options are:"
		echo ${ISOOPTIONS[@]}
		## We set options as invalid here. We don't exit the script
		## until later so that all options can have their validity
		## checked so that user can be provided with all the info
		## they need
		OPTSVALID=false
	else
		gphoto2 --quiet --set-config /main/imgsettings/iso=$SELECT
		unset SELECT
	fi
fi

# Aperture/F number
## The Aperture is specified in terms of the fnumber, perhaps later I
## will add a conversion to allow aperture to be specified but for now
## the desired f-number will be selected
if [ ! -z ${FNUMBER+x} ];
then
	#Get list of possible f-numbers from camera
	FOPTIONS=($(gphoto2 --quiet --get-config=/main/capturesettings/f-number | sed '1,4 d' | sed 's/.\{10\}//'))
	unset FOPTIONS[-1]
	#Search list for option that matches requested option
	count=0
	for OPTION in ${FOPTIONS[@]}
	do
		if [ $OPTION == $FNUMBER ];
		then
			SELECT=$count
		fi
		count=$(( count + 1 ))
	done
	unset count
	#Check if requested option was found, if it was then set it,
	# if not the print error
	if [ -z ${SELECT+x} ];
	then
		echo "Requested f-number is not available. Possible f-numbers are:"
		echo ${FOPTIONS[@]}
		OPTSVALID=false
	else
		gphoto2 --quiet --set-config /main/capturesettings/f-number=$SELECT
		unset SELECT
	fi
fi
# Focal Length
## This seems to be determined by manually adjusting the lens, it is
## not controlled by software, neither on the computer or camera




## Target
if [ ! -z ${TARGET+x} ];
then

	#Obtain target declination from catalog, return invalid option if it is not found
	DECLINATION=60
	#Obtain focal length and f-number from the camera
	CAMERASETTINGS=$(gphoto2 --quiet --list-all-config)
	FOCALLENGTH=$(echo "$CAMERASETTINGS" | sed -n '/\/main\/capturesettings\/focallength/{ n; n; n; n; p}' | sed 's/^.\{9\}//' )
	FNUMBER=$(echo "$CAMERASETTINGS" | sed -n '/\/main\/capturesettings\/f-number/{ n; n; n; n; p}' | sed 's/^.\{11\}//' )

	#Obtain the camera info file name
	CAMERAFILE='/home/jack/camera_control/D3500.json'

	#Get exposure time
	#Should check the function exit code to check valid result

	EXPOSURE=$($NPFCALC $FOCALLENGTH $FNUMBER $DECLINATION $CAMERAFILE 1 )
	echo Exposure calculated: $EXPOSURE
	EXPOSUREMODE=script
	gphoto2 --quiet --set-config /main/capturesettings/shutterspeed=52

fi
## If any of the provided options are invalid, exit script here:
if [ $OPTSVALID == "false" ];
then
	exit 1
fi




### Decide if we are using camera or computer to time exposures
# Camera is more accurate but can only have set exposure lengths.
# Computer is more flexible but less accurate.
# Current iteration will use the computer to time the exposures if
# an exposure value is provided as an option and if not then it will
# use whatever the camera is currently set to. In the future it would
# be nice to be able to use the camera to time exposures that are
# already set.


for i in $(seq 1 $NPHOTOS)
do
	echo Taking image $i
	if [ $EXPOSUREMODE == "script" ]
	then
		#Open shutter, then wait for desired length of time
		gphoto2 --quiet --set-config /main/actions/bulb=1 --wait-event=${EXPOSURE}s
	else
		#Simpler command to instruct camera to take sequence of images
		gphoto2 --quiet --trigger-capture
	fi
done

### Write capture settings to file
echo "$CAMERASETTINGS" > $INFOFILENAME





