#!/bin/bash

ORIG_FOLDER=$PWD

if [ $# -gt 1 ]; then
  echo "Usage: $0 [<path to dataset>]"
  echo "The path defaults to $HOME/Dataset"
  exit 1
fi

if [ $# -eq 1 ]; then
    DATASET_PATH=$1 
else
    DATASET_PATH=$HOME/Dataset
fi

echo "Storing the data to: " 
echo $DATASET_PATH

# get the file path for rospy_tutorials
mkdir -p $DATASET_PATH
cd $DATASET_PATH
echo "Downloading bags"
wget https://robotics.upo.es/datasets/echord/siar/siar_2017-10-11-11-05-03.bag
wget https://robotics.upo.es/datasets/echord/siar/siar_2017-10-17-10-12-16.bag
wget https://robotics.upo.es/datasets/echord/siar/siar_2018-06-12-10-54-51.bag
wget https://robotics.upo.es/datasets/echord/siar/siar_2018-12-13-11-43-26.bag

cd $ORIG_FOLDER