#! /bin/bash
for f in ground_truth*.sh; do  
  if [[ $f != *"arsi"* ]]; then
    
    bash "$f"  1 1
  fi
done
