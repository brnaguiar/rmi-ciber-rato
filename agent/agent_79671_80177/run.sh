#!/bin/bash

challenge="4"
host="localhost"
robname="myagent"
pos="0"
outfile="myrob" #22  

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    4)
        python3 mainC4.py -p "$pos" -r "$robname" -h "$host" -f "$outfile" 
        # how to call agent for challenge 3
        #./mainC3 -h "$host" -p "$pos" -r "$robname" -f "$outfile" # assuming -f is the option for the path
        ;;
esac 