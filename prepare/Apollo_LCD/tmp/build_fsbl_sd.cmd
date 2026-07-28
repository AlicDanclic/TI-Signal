@echo off
call E:\Vitis2\2025.1\Vitis\settings64.bat
set PATH=E:\Vitis2\2025.1\tps\win64\cmake-3.24.2\bin;E:\Vitis2\2025.1\Vitis\bin;E:\Vitis2\2025.1\gnu\aarch32\nt\gcc-arm-none-eabi\bin;%PATH%
set ESW_REPO=E:\Vitis2\2025.1\Vitis\data\embeddedsw
set XILINX_VITIS=E:\Vitis2\2025.1\Vitis
cd /d E:\Apollo_LCD\vitis_sd_workspace\platform_sd\zynq_fsbl
cmake -S zynq_fsbl_bsp -B zynq_fsbl_bsp\libsrc\build_configs\gen_bsp -G Ninja -DCMAKE_TOOLCHAIN_FILE=zynq_fsbl_bsp\cortexa9_toolchain.cmake -DCMAKE_SPECS_FILE=E:/Vitis2/2025.1/Vitis/data/embeddedsw/scripts/specs/arm/Xilinx.spec -DCMAKE_MODULE_PATH=E:/Apollo_LCD/vitis_sd_workspace/platform_sd/zynq_fsbl/zynq_fsbl_bsp -DCMAKE_PREFIX_PATH=E:/Apollo_LCD/vitis_sd_workspace/platform_sd/zynq_fsbl/zynq_fsbl_bsp/libsrc -DCMAKE_INCLUDE_PATH=E:/Apollo_LCD/vitis_sd_workspace/platform_sd/zynq_fsbl/zynq_fsbl_bsp/include -DCMAKE_LIBRARY_PATH=E:/Apollo_LCD/vitis_sd_workspace/platform_sd/zynq_fsbl/zynq_fsbl_bsp/lib
if errorlevel 1 exit /b 1
cmake --build zynq_fsbl_bsp\libsrc\build_configs\gen_bsp
if errorlevel 1 exit /b 1
cmake --install zynq_fsbl_bsp\libsrc\build_configs\gen_bsp
if errorlevel 1 exit /b 1
powershell -NoProfile -Command "Copy-Item -Path 'E:\Apollo_LCD\vitis_sd_workspace\platform_sd\zynq_fsbl\zynq_fsbl_bsp\libsrc\build_configs\gen_bsp\include\*' -Destination 'E:\Apollo_LCD\vitis_sd_workspace\platform_sd\zynq_fsbl\zynq_fsbl_bsp\include' -Force"
if exist build rmdir /s /q build
cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=zynq_fsbl_bsp\cortexa9_toolchain.cmake -DCMAKE_SPECS_FILE=E:/Vitis2/2025.1/Vitis/data/embeddedsw/scripts/specs/arm/Xilinx.spec -DCMAKE_MODULE_PATH=E:/Apollo_LCD/vitis_sd_workspace/platform_sd/zynq_fsbl/zynq_fsbl_bsp -DCMAKE_INCLUDE_PATH=E:/Apollo_LCD/vitis_sd_workspace/platform_sd/zynq_fsbl/zynq_fsbl_bsp/include -DCMAKE_LIBRARY_PATH=E:/Apollo_LCD/vitis_sd_workspace/platform_sd/zynq_fsbl/zynq_fsbl_bsp/lib
if errorlevel 1 exit /b 1
cmake --build build --verbose
