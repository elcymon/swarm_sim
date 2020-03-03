mkdir -p sources/plugins/build

cd sources/plugins/build
cmake ../
make

cp -f *.so ../../../compiled_plugins

cd ../world_governor
mkdir -p build
cd build
cmake ../
make

cp -f world_governor ../../../../