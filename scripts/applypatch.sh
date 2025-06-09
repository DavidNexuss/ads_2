#!/bin/bash

cd patch && 
find . -type f | while read -r line; do 
  mkdir -vp ../lib/$(dirname "$line" )
  cp -vp $line "../lib/$line"
done
