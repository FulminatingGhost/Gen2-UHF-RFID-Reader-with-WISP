#! /bin/sh
rm -rf .git
git init
git checkout -b parallel_cutoff
git remote add origin https://github.com/SpiritFlag/Gen2-UHF-RFID-Reader-with-WISP
git pull origin parallel_cutoff
