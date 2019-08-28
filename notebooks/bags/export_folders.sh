#!/bin/bash
echo "========== Start"
for f in $(ls | grep bag_); do
  echo "========== Start extracting ${f}"
  cd ${f}
  ../export_images.py
  cd -
done
echo "========== Done"
