git pull origin flip
cd ../build
make
make test
sudo make install
sudo ldconfig
cd ../apps
