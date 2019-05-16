#!/bin/bash
mkdir ./temp
wget --no-check-cert -O ./temp/data.zip https://www.dropbox.com/s/vr7n98bjafpinoo/data.zip?dl=0
cd ./temp
unzip data.zip
cp -r opencv-3.4.3 ../libraries/
cp example.ogv ../examples
cd ../
rm -rf ./temp