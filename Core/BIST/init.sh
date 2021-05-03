
mkdir -p build

cd build

for cfg in Debug Release
do
    mkdir -p ${cfg}
    pushd ${cfg} > dev/null
    cmake -DCMAKE_BUILD_TYPE=${cfg} -G"Unix Makefiles" ../..
    #cmake -DCMAKE_BUILD_TYPE=${cfg} -G"CodeBlocks - Unix Makefiles" ../..
    #cmake -DCMAKE_BUILD_TYPE=${cfg} -G"CodeLite - Unix Makefiles" ../..
    #cmake -DCMAKE_BUILD_TYPE=${cfg} -G"Eclipse CDT4 - Unix Makefiles" ../..
    #cmake -DCMAKE_BUILD_TYPE=${cfg} -G"KDevelop3 - Unix Makefiles" ../..
    popd > dev/null
done
