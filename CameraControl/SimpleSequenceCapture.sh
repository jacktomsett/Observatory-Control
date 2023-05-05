#!/bin/bash

#Basic sequence capture script. Assumes all settings have been made on the camera.
#Experimenting with adding command line arguments to set camera settings.
function SET_ISO {
    echo "iso set to $OPTARG"
    ISO=$OPTARG
}


#Practise using getopts...
#Define default variable first. Then overwrite any specified by command line with getopts
LOCATION="/home/pi/Observatory_Images"
SEQUENCENAME=$(date +%Y-%m-%d_%H-%M)
FILENAME=frame
N=4
WAITTIME=1



while getopts ':hn:f:i:e:' OPTION; do           #This loops through all provided command line paramters. On each iteration of the loop a new paramter will be stored in the variable "OPTION". The text in single quotes tells it what options the script recognises. The leading colon tells getopts to suppress error messages when an unrecognised option is detections (which you might want if for example you are providing your own error messages). Colons after a character tell bash that that option requires an argument.
    case "$OPTION" in
        h)
            echo "Display help" ;;
        n)
            N="$OPTARG" ;;
        f)
            SEQUENCENAME="$OPTARG" ;;
        i)
            case "$OPTARG" in
                100)
                    SET_ISO ;;
                200)
                    SET_ISO ;;
                400)
                    SET_ISO ;;
                800)
                    SET_ISO ;;
                1600)
                    SET_ISO ;;
                3200)
                    SET_ISO ;;
                6400)
                    SET_ISO ;;
                12800)
                    SET_ISO ;;
                25600)
                    SET_ISO ;;
                *)
                    echo "Error: Incompatible value for ISO specified. Supported ISO's are: 100,200,400,800,1600,3200,6400,128000,256000"
                    echo "Keeping camera setting..."
                    ;;
                esac
                ;;
        e)
            case "$OPTARG" in




            esac
            ;;

        ?)
            echo "Usage: $(basename $0) [-h] [-n] [-f] [-i] [-e]" ;;
    esac
done


cd $LOCATION
mkdir $SEQUENCENAME

for ((i=1;i<=N;i++));
do
    FULLFILEPATH="$SEQUENCENAME/$FILENAME-$i"
    #gphoto2 --folder=$SEQUENCENAME --capture-image-and-download --filename="test_sequence/%Y%m%d%H%M%S.jpg"
    echo Image $FULLFILEPATH transferred to Pi
    #sleep $WAITTIME
done
