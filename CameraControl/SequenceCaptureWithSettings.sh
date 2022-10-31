#!/bin/bash

#Idea is for user to supply options and based on information provided a photo
#sequence will be triggered with the correct settings.
#intended arguments are:
# -t = target, Messier Catalog Number for desired target. Mandatory
# -N = number of frames. Mandatory
# -F = filename. Mandatory. Name of folder to store images in
# -C = Camera, Which camera data (pixel size, iso value) to use (list still to be made). Optional, 
# If not specified then will default to Nikon D3500.
# -L = Lens. Which lens data (focal length, aperture) to use. only two so far
# (still need to implement). Kit lens and Telescope. If using kit lens then
# camera needs to be polled for aperture and focal length.Default to kit
# -S = simplenpf. optional. specifies to use simple npf rule
# -I = ISO. Optional. Specify ISO value to use. if none selected will default to
#preferred ISO for camera
# -T = Time (and date) of observation time. Optiona. Default to current time
# -P = Position (observer) two arguments separated with a comma Latt,Long. values
# Should be in decimal form. Optional. Defaults to Camera Settings


##QUESTION... Do we check inputs here? (ie recognised camera and lens) or in the 
#c++ program. I think in c++

#Set default parameters
CAMERA=Nikon
LENS=Kit
NPF=full

while getopts 'C:L:SI:' OPTION; do
    case "$OPTION" in
        C)
            CAMERA="$OPTARG" 
            echo Requested camera: $CAMERA
        L)
            LENS="$OPTARG" 
            echo Requested lens: $LENS
        S)
            NPF=simple ;;
            echo Simple NPF rule selected
        I)
            ISO="$OPTARG" 
            echo Requested ISO: $ISO
    esac

done

if [$LENS -eq Kit]
then
    echo Obtaining Lens settings from camera...
    FL=focallength
fi
EXPOSURE=$(CalculateExposure $TARGET $CAMERA $LENS $LATT $LONG $TIME $NPF)
