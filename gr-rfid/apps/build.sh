git pull origin parallel_clustering
cd ../build
make
make test
sudo make install
sudo ldconfig
cd ../apps
