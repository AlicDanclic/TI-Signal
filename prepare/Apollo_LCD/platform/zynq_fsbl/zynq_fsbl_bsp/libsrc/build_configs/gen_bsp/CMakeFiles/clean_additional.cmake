# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "E:\\Apollo_LCD\\platform\\zynq_fsbl\\zynq_fsbl_bsp\\include\\diskio.h"
  "E:\\Apollo_LCD\\platform\\zynq_fsbl\\zynq_fsbl_bsp\\include\\ff.h"
  "E:\\Apollo_LCD\\platform\\zynq_fsbl\\zynq_fsbl_bsp\\include\\ffconf.h"
  "E:\\Apollo_LCD\\platform\\zynq_fsbl\\zynq_fsbl_bsp\\include\\sleep.h"
  "E:\\Apollo_LCD\\platform\\zynq_fsbl\\zynq_fsbl_bsp\\include\\xilffs.h"
  "E:\\Apollo_LCD\\platform\\zynq_fsbl\\zynq_fsbl_bsp\\include\\xilffs_config.h"
  "E:\\Apollo_LCD\\platform\\zynq_fsbl\\zynq_fsbl_bsp\\include\\xilrsa.h"
  "E:\\Apollo_LCD\\platform\\zynq_fsbl\\zynq_fsbl_bsp\\include\\xiltimer.h"
  "E:\\Apollo_LCD\\platform\\zynq_fsbl\\zynq_fsbl_bsp\\include\\xtimer_config.h"
  "E:\\Apollo_LCD\\platform\\zynq_fsbl\\zynq_fsbl_bsp\\lib\\libxilffs.a"
  "E:\\Apollo_LCD\\platform\\zynq_fsbl\\zynq_fsbl_bsp\\lib\\libxilrsa.a"
  "E:\\Apollo_LCD\\platform\\zynq_fsbl\\zynq_fsbl_bsp\\lib\\libxiltimer.a"
  )
endif()
