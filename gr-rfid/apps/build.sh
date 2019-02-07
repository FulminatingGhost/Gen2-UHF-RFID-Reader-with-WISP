git pull origin parallel_cutoff
cd ../build
make
make test
sudo make install
sudo ldconfig
cd ../apps
