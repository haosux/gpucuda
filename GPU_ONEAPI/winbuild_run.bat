mkdir build
cd build
cmake ..
cmake --build . --config Release
set PATH=%PATH%;C:\Program Files (x86)\PCL\bin;C:\Program Files (x86)\PCL-ONEAPI\bin;C:\Program Files (x86)\Intel\oneAPI\compiler\2023.0.0\windows\bin;C:\Program Files (x86)\Intel\oneAPI\ipp\latest\redist\intel64;C:\Program Files (x86)\Intel\oneAPI\ipp\latest\redist\intel64\tl\tbb;C:\Users\RS\Downloads\opencv_4.5.5\build\x64\vc15\bin

ping -n 5 127.0.0.1 > nul  

set SYCL_DEVICE_FILTER=ext_oneapi_level_zero:gpu

REM .\Release\hello_sycl.exe 

REM .\Release\oneapi_octree.exe 

.\Release\perf_octree.exe 

PAUSE