git pull origin exp
cd ../build
make
make test
sudo make install
sudo ldconfig
cd ../apps
