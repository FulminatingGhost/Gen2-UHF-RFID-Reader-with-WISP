#! /bin/sh
rm -r debug_data
mkdir debug_data
rm debug_message result
python reader.py >> debug_message
cat result
rm a
