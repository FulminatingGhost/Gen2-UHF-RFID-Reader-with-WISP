git pull origin parallel
cd ../build
make
make test
sudo make install
sudo ldconfig
cd ../apps
