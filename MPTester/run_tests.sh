#!../../../../../../usr/bin/bash

# Command line argument
USERNAME=$1 # Agate username

javac -cp /home/csu/$USERNAME/UNHTravSalesProj MasterTest.java # Compiles MasterTest
javac Quotient.java # Compiles Quotient
rm -f averages.csv # Delete the averages file if it exists
mkdir -p results/ # Create results directory if it doesn't exist already
cd problems/ # Change to the directory where the PTSP problems are
PROBLEM_FILES=(*) # Make an array with the names of all the problem files in the problems directory
cd ../ # Return to MPTester directory

SEEDS=()
ITERATIONS=50

# Generate random seeds for testing
for ((i=0;i<$ITERATIONS;i++))
do
    SEEDS+=($RANDOM)
done

# Iterate through each problem file, running the MasterTest on each
for FILE in ${PROBLEM_FILES[@]}
do
    FILENAME=$(echo $FILE | cut -d'.' -f 1)

    # Run each problem with the specified number of seeds
    for SEED in ${SEEDS[@]}
    do
        echo SOLVING $FILE
        java -cp /home/csu/$USERNAME/UNHTravSalesProj MasterTest.java problems/$FILE 100000 5 false $SEED # Runs MasterTest using the given arguments
    done
    
    python violinplots.py problems/$FILENAME.csv "Runtime" # Create PNG for comparing runtimes
    python violinplots.py problems/$FILENAME.csv "Solution Cost" # Create PNG for comparing solution costs
    java Quotient.java problems/$FILENAME.csv $ITERATIONS # Run the quotient finding/averaging on the file
    cd results/ # Changes to results directory
    mkdir -p $FILENAME/ # Makes directory for problem instance (Ex. problem0) if it doesn't exist already
    cd ../ # Return to the MPTester directory
    cp problems/$FILE results/$FILENAME/ # Copy the problem file into the problem folder
    mv problems/$FILENAME.csv Runtime.png SolutionCost.png results/$FILENAME # Move the csv file and plot PNGs into the problem folder
done

# Create PNGs for the data from all problems
python violinplots.py averages.csv "Motion Planning Algorithm" "Runtime"
python violinplots.py averages.csv "Motion Planning Algorithm" "Solution Cost"
python violinplots.py ratios.csv "Distribution" "DIRT:RRT Runtime Ratio"
python violinplots.py ratios.csv "Distribution" "DIRT:RRT Solution Cost Ratio"
python violinplots.py scatter.csv "RRT Runtime" "DIRT Runtime" scatter
python violinplots.py scatter.csv "RRT Solution Cost" "DIRT Solution Cost" scatter