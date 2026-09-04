#!/bin/bash
./kill.sh
./gz_ca1.sh >gz_ca1.log 2>&1 &
./ca1.sh

