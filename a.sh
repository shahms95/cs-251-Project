#!/bin/sh
while [ 1 ]; do
    make		
    timeout 260 bin/cs251_base
done

