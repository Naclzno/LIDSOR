#!/bin/bash

source dirs.sh

if [ $weather_type == "snow" ]; then
  # Snow
  i=1 
  for og_file in $og_dir/*.txt 
  do 
    filename=$(basename $og_file .txt)  
    ep_filename="${filename}_ep.txt"    
    en_filename="${filename}_en.txt"    
    
    echo "Processing [FILE $i: $filename] ..." 
    # echo -n "ROR used " 
    # $exec_dir/ror -f $og_file -ep $ror_ep_dir/$ep_filename -en $ror_en_dir/$en_filename -n 3 -r 0.12 -dt 100 -it 50
    # echo -n "SOR used "
    # $exec_dir/sor -f $og_file -ep $sor_ep_dir/$ep_filename -en $sor_en_dir/$en_filename -k 12 -m 0.15 -dt 100 -it 50
    # echo -n "DROR used "
    # # $exec_dir/dror -f $og_file -ep $dror_ep_dir/$ep_filename -en $dror_en_dir/$en_filename -n 3 -m 1.5 -a 0.2 -dt 100 -it 50 
    # $exec_dir/dror -f $og_file -ep $dror_ep_dir/$ep_filename -en $dror_en_dir/$en_filename -n 3 -m 1.75 -a 0.2 -r 0.04 -dt 100 -it 50 
    # echo -n "DSOR used "
    # $exec_dir/dsor -f $og_file -ep $dsor_ep_dir/$ep_filename -en $dsor_en_dir/$en_filename -k 12 -m 0.15 -r 0.16 -dt 100 -it 50 
    # echo -n "DDIOR used "
    # $exec_dir/ddior -f $og_file -ep $ddior_ep_dir/$ep_filename -en $ddior_en_dir/$en_filename -k 15 -a 0.2 -dt 100 -it 50 
    echo -n "LIDSOR used "
    $exec_dir/lidsor -f $og_file -ep $lidsor_ep_dir/$ep_filename -en $lidsor_en_dir/$en_filename -k 12 -m 0.12 -r 0.12 -d 100 -i 50 

    i=$(expr $i + 1)
  done 

elif [ $weather_type == "rain" ]; then
  # Rain
  i=1
  for og_file in $og_dir/*.txt
  do
    filename=$(basename $og_file)
    echo "Processing [FILE $i: $filename] ..."
    # echo -n "ROR used "
    # $exec_dir/ror -f $og_file -ep $ror_ep_dir/$filename -en $ror_en_dir/$filename -n 3 -r 0.2
    # echo -n "SOR used "
    # $exec_dir/sor -f $og_file -ep "$sor_ep_dir/$filename" -en "$sor_en_dir/$filename" -k 7 -m 0.15
    # echo -n "DROR used "
    # $exec_dir/dror -f $og_file -ep $dror_ep_dir/$filename -en $dror_en_dir/$filename -n 3 -m 3 -a 0.2 -r 0.16
    # echo -n "DSOR used "
    # $exec_dir/dsor -f $og_file -ep "$dsor_ep_dir/$filename" -en "$dsor_en_dir/$filename" -k 5 -m 0.12 -r 0.16
    # echo -n "DDIOR used "
    # $exec_dir/ddior -f $og_file -ep "$ddior_ep_dir/$filename" -en "$ddior_en_dir/$filename" -k 5 -a 0.2
    echo -n "LIDSOR used "
    $exec_dir/lidsor -f $og_file -ep "$lidsor_ep_dir/$filename" -en "$lidsor_en_dir/$filename" -k 5 -m 0.12 -r 0.16 -i 25 -d 60
    i=$(expr $i + 1)
  done
fi