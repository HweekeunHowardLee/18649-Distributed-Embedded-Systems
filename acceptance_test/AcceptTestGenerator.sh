# 18649 2014 Fall
# group #4
# Zhu Zhang (zhuzhang), Hua Liu (hual1), Howard Lee (hweekeul)
# sd1b1.mf

#!/bin/bash

# This template generates passenger files for acceptance testing the elevator.
#
# Passengers all enter on floor one in a randomized hall, and exit on floor 2.
# You should update this so that passengers can begin and end in any valid
# hallway. If you want to add additional parameters (e.g. timing, uppeak
# bias, etc.) you may do so. Be sure that your usage accurately describes all
# parameters used.
#
# .Pass files should start with a comment describing the test, including what
# arguments were used to generate it.
# Subsequent lines are passenger injections and have the following format:
#
#           Time	Start Floor	Start Hallway	End Floor	End Hallway

MIN_ARG_NUM=1;

if [ $# -lt $MIN_ARG_NUM ]
    then
        #Replace these with actual usage directions
        echo "<USAGE INFO>";
        echo "./AcceptTestGenerator.sh passenger_num";
        echo "Example:";
        echo "./AcceptTestGenerator.sh 40";
else
        PASS_NUM=$1; #Read the arguments into local variables
        for ((k = 1; k <= 40; k++));
          do
            OUTFILE="Random$k.pass";
            echo ";Random Generated Pass file. Number of Passengers = $PASS_NUM" > $OUTFILE;
            for ((i = 1; i <= $PASS_NUM; i++));
                do
                    n1=$(($RANDOM % 2 ));
                    n2=$(($RANDOM % 2 ));
                    s="$((i * 5))s "; #Passenger insertion time
                    f_in=$(($(($RANDOM % 8))+1));
                    f_out=$(($(($RANDOM % 8))+1));
                    s="$s $f_in "; #Passenger starts on floor X...
                    if [ "$f_in" = "1" -o "$f_in" = "7" ]
                      then
                        case $n1 in #... in a random hallway
                            0 )
                                s="$s FRONT ";
                                ;;
                            1 )
                                s="$s BACK ";
                                ;;
                        esac
                    elif [ "$f_in" = "2" ]
                      then
                        s="$s BACK "
                    else
                      s="$s FRONT "
                    fi
                    s="$s $f_out "; # and ends on floor Y
                    if [ "$f_out" = "1" -o "$f_out" = "7" ]
                      then
                      case $n2 in #... out a random hallway
                        0 )
                        s="$s FRONT ";
                        ;;
                        1 )
                        s="$s BACK ";
                        ;;
                      esac
                    elif [ "$f_out" = "2" ]
                      then
                      s="$s BACK "
                    else
                      s="$s FRONT "
                    fi
                    echo "$s" >> $OUTFILE
                done
          done

fi
