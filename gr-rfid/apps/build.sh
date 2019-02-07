git pull origin parallel_cdf
cd ../build
make
make test
sudo make install
sudo ldconfig
cd ../apps
