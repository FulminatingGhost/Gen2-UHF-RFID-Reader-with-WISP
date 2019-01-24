#git pull origin decoding
cd ../build
make
make test
sudo make install
sudo ldconfig
cd ../apps
