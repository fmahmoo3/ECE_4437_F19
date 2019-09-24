#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /Users/fazalmahmood/ti/tirtos_tivac_2_16_00_08/packages;/Users/fazalmahmood/ti/tirtos_tivac_2_16_00_08/products/tidrivers_tivac_2_16_00_08/packages;/Users/fazalmahmood/ti/tirtos_tivac_2_16_00_08/products/bios_6_45_01_29/packages;/Users/fazalmahmood/ti/tirtos_tivac_2_16_00_08/products/ndk_2_25_00_09/packages;/Users/fazalmahmood/ti/tirtos_tivac_2_16_00_08/products/uia_2_00_05_50/packages;/Users/fazalmahmood/ti/tirtos_tivac_2_16_00_08/products/ns_1_11_00_10/packages;/Users/fazalmahmood/ECE_4437_F19/Final_Robot_Project_Fall_19/.config
override XDCROOT = /Applications/ti/xdctools_3_32_00_06_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /Users/fazalmahmood/ti/tirtos_tivac_2_16_00_08/packages;/Users/fazalmahmood/ti/tirtos_tivac_2_16_00_08/products/tidrivers_tivac_2_16_00_08/packages;/Users/fazalmahmood/ti/tirtos_tivac_2_16_00_08/products/bios_6_45_01_29/packages;/Users/fazalmahmood/ti/tirtos_tivac_2_16_00_08/products/ndk_2_25_00_09/packages;/Users/fazalmahmood/ti/tirtos_tivac_2_16_00_08/products/uia_2_00_05_50/packages;/Users/fazalmahmood/ti/tirtos_tivac_2_16_00_08/products/ns_1_11_00_10/packages;/Users/fazalmahmood/ECE_4437_F19/Final_Robot_Project_Fall_19/.config;/Applications/ti/xdctools_3_32_00_06_core/packages;..
HOSTOS = MacOS
endif
