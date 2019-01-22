#! /bin/sh
rm -r debug_data
mkdir debug_data
rm debug_message
python reader.py >> debug_message
cat result
