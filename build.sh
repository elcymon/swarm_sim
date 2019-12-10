mkdir -p sources/plugins/build

cd sources/plugins/build
cmake ../
make

cp *.so ../../../compiled_plugins

cd ../world_governor
mkdir -p build
cd build
cmake ../
make

cp world_governor ../../../../