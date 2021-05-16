
mkdir -p build

cd build

for cfg in Debug
do
    mkdir -p ${cfg}
    pushd ${cfg} > dev/null
    cmake -DCMAKE_BUILD_TYPE=${cfg} -G"Unix Makefiles" ../..
    popd > dev/null
done
