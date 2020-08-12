#!../../../../../usr/bin/bash

# Command line arguments
USERNAME=$1 # Agate username
TRIALS=${2:-1} # Number of trials to run for each problem (OPTIONAL: Default is 1)
# Any number of other arguments for which problem files (from the problems folder) to run (OPTIONAL: Default is all of them)

re='^[0-9]+$'
if ! [[ $TRIALS =~ $re ]] # If TRIALS got a filename (trial command argument was neglected), set it to 1 instead
then
    TRIALS=1
    skip=1
else
    skip=2
fi

args=("$@") # Create array from all command line arguments
PROBLEM_FILES=("${args[@]:skip}") # Set PROBLEM_FILES equal to args minus the first 1-2 command line arguments (just the file names)

cd version1/ # Change directory to where Mastermind is located
mkdir -p results/ # Create results directory if it doesn't exist already
cd problems/ # Change to the directory where the PTSP problems are

if [[ ${#PROBLEM_FILES[@]} -eq 0 ]] # If no problem files were given, run through all of the ones in the problems folder
then
    PROBLEM_FILES=(*)
fi

cd ../ # Return to version1 directory

# Generate random seeds for uniform testing
RANDOM=8008
SEEDS=()

for ((i=0;i<$TRIALS;i++))
do
    SEEDS+=($RANDOM)
done

# Variables to keep track of the numbers of successes and failures
SUCCESSES=0
FAILURES=0

# Iterate through each problem file, running the Mastermind and Animator automagically
for FILE in ${PROBLEM_FILES[@]}
do
    FILENAME=$(echo $FILE | cut -d'.' -f 1)

    # Run a user-defined number of trials (each trial being a different seed) for each problem
    for SEED in ${SEEDS[@]}
    do
        echo SOLVING $FILE
        TRIAL_NUM=-1
        javac -cp /home/csu/$USERNAME/UNHTravSalesProj Mastermind.java # Compiles Mastermind

        # If running Mastermind fails, stop right there (and go on to next iteration)
        if ! java -cp /home/csu/$USERNAME/UNHTravSalesProj Mastermind.java problems/$FILE -1 10000 5 $SEED # Runs Mastermind using the given arguments
        then
            echo "Failed on $FILE for seed $SEED" >> successFail.txt
            ((FAILURES++))
            continue
        fi
        ((SUCCESSES++))

        cd results/ # Changes to results directory
        mkdir -p $FILENAME/ # Makes directory for problem instance (Ex. problem0) if it doesn't exist already
        cd $FILENAME/ # Changers directory again into the problem instance folder

        # Find the highest-numbered existing trial for this problem
        for f in */
        do
            FOLDERNUM=${f//[!0-9]/}
            if [[ $FOLDERNUM -gt $TRIAL_NUM ]]
            then
                TRIAL_NUM=$FOLDERNUM
            fi
        done

        # Add one to that to determine the current trial number
        TRIAL_NUM=$((TRIAL_NUM + 1))

        mkdir trial$TRIAL_NUM/ # Makes a folder for the current trial
        cd ../../ # Moves back out of the problem and results directories
        cp problems/$FILE results/$FILENAME/ # Copies the problem file into its designated folder (only for the first trial)
        # cp problems/$FILE ../animator/ # Copy the problem file over to the animator
        # cp controls.txt ../animator/ # Copy the controls over to the animator
        mv controls.txt results/$FILENAME/trial$TRIAL_NUM/ # Moves the controls from version1 to the trial folder
        # cd ../animator/ # Changes directories to the animator
        # python animator.py $FILE controls.txt false # Runs the animator with the given controls and problem files and tells it not to display the animation
        # rm $FILE controls.txt # Delete the problem and control files from the animator directory
        # mv animation.gif ../version1/results/$FILENAME/trial$TRIAL_NUM/ # Move the resulting gif from the animator to the trial folder
        # cd ../version1 # Reset directory for next iteration of inner loop
    done
done
echo "Successes: $SUCCESSES" >> successFail.txt
echo "Failures: $FAILURES" >> successFail.txt
mv successFail.txt results/