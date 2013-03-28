#!/bin/bash
echo "/**" 
echo " * this file is automatic generate by genregs.awk , Please do not edit it " 
echo " * base files are ../ucode/register.h ../ucode/c_always_on_pointer.h ../ucode/pctl.h "
echo " * ../ucode/c_stb_define.h ../ucode/secure_apb.h from VLSI team"
echo " */"
echo "#ifndef __MACH_MESON6_REG_ADDR_H_"
echo "#define __MACH_MESON6_REG_ADDR_H_"
gawk -f genregs.awk ../ucode/register.h ../ucode/c_always_on_pointer.h ../ucode/pctl.h ../ucode/c_stb_define.h ../ucode/secure_apb.h ../ucode/hdmi.h


echo "#endif"
