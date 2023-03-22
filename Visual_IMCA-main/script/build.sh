#! /bin/bash

if [ ! -d "./vs-build" ]; then
    mkdir ./vs-build
fi

cd ./vs-build

title='[ EXECUTE ]'

cmake -DCMAKE_CXX_COMPILER=/usr/bin/g++ -DCMAKE_C_COMPILER=/usr/bin/gcc -GNinja .. && ninja

if [ $? -eq 0 ]; then
    clear
    yes "=" | sed '5q' | tr -d '\n'
    echo -n ${title}
    yes "=" | sed '5q' | tr -d '\n'
    echo ''
    echo ''
    ./ROOT_IMCA
fi

