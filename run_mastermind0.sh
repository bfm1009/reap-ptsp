#!../../../../../usr/bin/bash

# Run this with "./run_mastermind.sh [Agate username]" from the root directory of the repository

set -e # Stop executing this script if a command fails

USERNAME=$1 # Command line argument 1
FILENAME="simple.ptsp" # Name of the ptsp problem file

cd version0/
javac -cp /home/csu/$USERNAME/UNHTravSalesProj Mastermind.java
java -cp /home/csu/$USERNAME/UNHTravSalesProj Mastermind.java $FILENAME -1 100000
cp $FILENAME controls.txt states.txt ../animator/
cd ../animator/
python animator.py $FILENAME controls.txt